"""Independent 50 Hz motor watchdog. Runs under the LeRobot Python environment."""
import argparse
import json
import math
import os
import signal
import socket
import time
from .core import Guard, PERIOD, TTL, FEEDBACK_TTL, load_profile
from .bus import MockBus, LeRobotBus, SoftwareHoldBus
from .ipc import send, receive


def complete_sleep(bus, guard, message, q, cfg, motion_mode, now, sequence):
    """Release only a fresh, current sleep completion at the measured sleep pose."""
    if (not sequence or guard.state != 'ARMED' or motion_mode != 'sleep'
            or message.get('epoch') != guard.epoch
            or not 0 <= now-message.get('issued', 0) <= TTL):
        raise ValueError('Invalid sleep completion request')
    if max(abs(a-b) for a,b in zip(q[:6],cfg['sleep_rad'])) > math.radians(.5):
        raise ValueError('Sleep torque-off rejected: arm has not reached sleep pose')
    bus.disable_torque()
    guard.state = 'PARKED'
    guard.reason = 'Sleep reached; motor torque off; restart for a new session'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fd', type=int, required=True)
    ap.add_argument('--parent-pid', type=int, required=True)
    ap.add_argument('--profile', required=True)
    ap.add_argument('--side', choices=('left','right'), required=True)
    ap.add_argument('--manual-start',action='store_true')
    ap.add_argument('--sequence', action='store_true')
    ap.add_argument('--startup', action='store_true')
    ap.add_argument('--mode', choices=('mock','observe','live'), required=True)
    args = ap.parse_args()
    sock = socket.socket(fileno=args.fd); sock.setblocking(False)
    if args.sequence and (not args.startup or args.manual_start or args.mode=='observe'):
        ap.error('Sequence requires automatic startup in mock/live mode')
    if args.manual_start and (not args.startup or args.mode=='observe'):ap.error('Manual start requires startup and mock/live mode')
    profile = load_profile(args.profile, live=args.mode=='live', sides=(args.side,),require_motor_watchdog=not (args.manual_start or args.sequence))
    if args.startup and args.mode=='observe':
        ap.error('Observe mode never enables torque')
    cfg = profile['arms'][args.side]
    bus = None; q = None; fresh = 0.; stop = False; shutdown = False
    guard = Guard(cfg['lower_rad'],cfg['upper_rad'],sequence=args.sequence)
    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT,on_signal); signal.signal(signal.SIGTERM,on_signal)
    try:
        adapter=SoftwareHoldBus if args.manual_start or args.sequence else LeRobotBus
        bus = MockBus(profile,args.side,startup=args.startup) if args.mode=='mock' else adapter(profile,args.side,startup=args.startup)
        def startup_cancelled():
            nonlocal shutdown
            pending=receive(sock)
            shutdown |= any(m.get('op')=='shutdown' for m in pending)
            return stop or os.getppid()!=args.parent_pid or shutdown or any(m.get('op')=='hold' for m in pending)
        if args.manual_start:
            if args.mode=='live' and bus.register('Torque_Enable')!=[0]*9:
                raise RuntimeError('Manual start requires torque OFF; will not drop an already powered arm')
            deadline=time.monotonic()+5;last=None
            while time.monotonic()<deadline:
                if startup_cancelled():raise RuntimeError('Positioning cancelled before torque enable')
                remaining=int(deadline-time.monotonic())+1
                if remaining!=last:print(f'{args.side}: {remaining} ... position and support arm',flush=True);last=remaining
                if args.mode=='live' and any(bus.register('Torque_Enable')):raise RuntimeError('Torque changed during countdown')
                time.sleep(.02)
            print('0: stop moving; verifying current-position hold',flush=True)
        if args.startup:
            q, health = bus.startup_hold(startup_cancelled, **({'ignore_model_limits':True} if args.sequence else {}))
            send(sock,dict(type='startup',details=bus.startup_report))
        else:
            q, health = bus.read()
        fresh = time.monotonic()
        guard.hold(q,'Startup')
        if args.mode != 'observe' and not args.startup:
            bus.write(q)
            bus.configure_motion_caps()
            bus.watchdog()
        send(sock,dict(type='preflight',details=getattr(bus,'preflight',{'mock':True})))
        drift_since=None; drift_fault=False; torque_disabled=False
        seq = 0; last_issued = None; motion_mode = None; active_profile=None; previous_q=q[:]; previous_t=fresh
        while True:
            start = time.monotonic()
            if os.getppid()!=args.parent_pid:
                stop=True
            messages = receive(sock)
            # Process stop before reset or motion, even if commands are queued after it.
            stop_messages = [m for m in messages if m.get('op') in ('hold','shutdown')]
            shutdown |= any(m.get('op')=='shutdown' for m in stop_messages)
            q, health = bus.read(torque_off=torque_disabled); fresh = time.monotonic()
            if fresh-start > FEEDBACK_TTL:
                raise TimeoutError('Feedback cycle exceeded 100 ms')
            if torque_disabled:
                # Remain observable after parking, but never accept more commands.
                seq += 1
                send(sock,dict(type='feedback',q=q,goal=guard.goal,t=fresh,seq=seq,state='PARKED',
                    epoch=guard.epoch,reason=guard.reason,hold_command_confirmed=False,hold_confirmed=False,health=health))
                if stop or shutdown:break
                time.sleep(max(0.,PERIOD-(time.monotonic()-start)))
                continue
            if stop or stop_messages:
                guard.hold(q, 'Local shutdown' if stop else stop_messages[-1].get('reason','Stop'))
                last_issued = None
            elif args.mode != 'observe' and not drift_fault and any(
                    m.get('op')=='clutch' and m.get('epoch')==guard.epoch and
                    0 <= fresh-m.get('issued',0) <= TTL for m in messages):
                guard.clutch(q)
                # Ignore queued motion from before release; watchdog still runs.
            elif args.mode != 'observe' and not drift_fault:
                for m in messages:
                    if m.get('op') == 'arm' and 0 <= fresh-m.get('issued',0) <= TTL:
                        motion_mode=None
                        guard.arm(q,m['epoch']); guard.last_command = fresh
                        last_issued = None
                    elif m.get('op') == 'sleep_complete':
                        complete_sleep(bus,guard,m,q,cfg,motion_mode,fresh,args.sequence)
                        torque_disabled=True
                        break
                    elif m.get('op') == 'command':
                        dt = PERIOD if last_issued is None else m['issued']-last_issued
                        if not .005 <= dt <= TTL:
                            guard.hold(q,'Invalid command cadence')
                        else:
                            guard.command(m,q,fresh,dt)
                            motion_mode=m.get('mode')
                        last_issued = m['issued']
            if torque_disabled:continue
            target = guard.tick(q,time.monotonic())
            if args.mode != 'observe':
                requested_profile=motion_mode if args.sequence and guard.state=='ARMED' and motion_mode in ('opening','sleep') else None
                if requested_profile!=active_profile:
                    bus.opening_speed(False)
                    if requested_profile=='opening':bus.opening_speed(True)
                    elif requested_profile=='sleep':bus.opening_speed(True,counts=14,acceleration_counts=5)
                    active_profile=requested_profile
                bus.write(target)
            speed=max(abs(a-b) for a,b in zip(q[:6],previous_q[:6]))/max(fresh-previous_t,1e-6)
            settled=speed < .00873 and max(abs(a-b) for a,b in zip(q[:6],target[:6])) < .00873 and abs(q[6]-target[6])<.02
            if guard.state=='HOLD' and (max(abs(a-b) for a,b in zip(q[:6],target[:6])) > .0174533 or abs(q[6]-target[6]) > .02):
                drift_since=fresh if drift_since is None else drift_since
                if fresh-drift_since>.5 and not drift_fault:
                    drift_fault=True
                    guard.reason='Hold drift detected; fixed target retained; restart required'
                    send(sock,dict(type='fault',reason=guard.reason,hold_confirmed=False))
            else:
                drift_since=None
            previous_q=q[:]; previous_t=fresh
            seq += 1
            send(sock,dict(type='feedback',q=q,goal=target,t=fresh,seq=seq,state=guard.state,
                           epoch=guard.epoch,reason=guard.reason,hold_command_confirmed=guard.state=='HOLD',hold_confirmed=guard.state=='HOLD' and settled,health=health))
            if stop or shutdown:
                break
            time.sleep(max(0.,PERIOD-(time.monotonic()-start)))
    except BaseException as exc:
        confirmed = False
        # No endless retries, no position recapture loop, no torque-off cleanup.
        if bus and q and not getattr(bus,'torque_release_attempted',False) and args.mode != 'observe' and time.monotonic()-fresh <= FEEDBACK_TTL:
            try:
                bus.write(guard.hold(q,str(exc)))
                # A write/readback alone does not prove physical settling.
                confirmed = False
            except Exception:
                pass
        send(sock,dict(type='fault',reason=str(exc),hold_confirmed=confirmed))
    finally:
        if bus:
            try:
                if args.mode!='observe' and not getattr(bus,'torque_release_attempted',False):bus.opening_speed(False)
            finally:bus.close()
        sock.close()


if __name__ == '__main__':
    main()
