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
from .motion import motion_settings, latest_motion_messages, controller_source_fresh, ClutchBarrier
from .gripper import gripper_speed, gripper_acceleration


def complete_sleep(bus, guard, message, q, cfg, motion_mode, now, sequence):
    """Release only a fresh, current sleep completion at the measured sleep pose."""
    if (not sequence or guard.state != 'ARMED' or motion_mode != 'sleep'
            or message.get('epoch') != guard.epoch
            or not 0 <= now-message.get('issued', 0) <= TTL):
        raise ValueError('Invalid sleep completion request')
    if max(abs(a-b) for a,b in zip(q[:6],cfg['sleep_rad'])) > math.radians(.5):
        raise ValueError('Sleep torque-off rejected: arm has not reached sleep pose')
    bus.opening_speed(False)  # Restore temporary RAM profiles before torque release.
    bus.disable_torque()
    guard.state = 'PARKED'
    guard.reason = 'Sleep reached; motor torque off; restart for a new session'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fd', type=int, required=True)
    ap.add_argument('--parent-pid', type=int, required=True)
    ap.add_argument('--gripper-acceleration',type=float)
    ap.add_argument('--gripper-speed',type=float)
    ap.add_argument('--teleop-joint-speed',type=float)
    ap.add_argument('--teleop-joint-acceleration',type=float)
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
    for arg,key in [('teleop_joint_speed','teleop_joint_speed_deg_s'),('teleop_joint_acceleration','teleop_joint_acceleration_deg_s2')]:
        if getattr(args,arg) is not None:profile[key]=getattr(args,arg)
    if args.gripper_speed is not None:profile['gripper_speed']=args.gripper_speed
    profile['gripper_speed']=gripper_speed(profile)
    if args.gripper_acceleration is not None:profile['gripper_acceleration']=args.gripper_acceleration
    profile['gripper_acceleration']=gripper_acceleration(profile)
    motion=motion_settings(profile)
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
        drift_since=None; drift_fault=False; torque_disabled=False; gripper_range_fault=False
        previous_gripper_feedback_state=None
        barrier=ClutchBarrier();executed=None;last_target=q[:];previous_goal=q[:]
        seq = 0; last_issued = None; motion_mode = None; active_profile=None; previous_q=q[:]; previous_t=fresh
        while True:
            start = time.monotonic()
            if os.getppid()!=args.parent_pid:
                stop=True
            # Merge both sides of the blocking read. A newer target or stop can
            # arrive while encoders are being read; it must win this cycle.
            messages = receive(sock)
            read_started=time.monotonic()
            q, health = bus.read(torque_off=torque_disabled); fresh = time.monotonic()
            feedback_read_ms=1000*(fresh-read_started)
            messages = latest_motion_messages(messages+receive(sock))
            # Process stop before reset or motion, even if commands follow it.
            stop_messages = [m for m in messages if m.get('op') in ('hold','shutdown')]
            shutdown |= any(m.get('op')=='shutdown' for m in stop_messages)
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
            gripper_feedback=health.get('gripper_feedback') or {}
            gripper_state=gripper_feedback.get('state','in_range')
            if gripper_state!=previous_gripper_feedback_state:
                if gripper_state!='in_range':send(sock,dict(type='gripper_feedback',details=gripper_feedback))
                previous_gripper_feedback_state=gripper_state
            if gripper_state=='out_of_range' and not gripper_range_fault:
                reason=f"Gripper encoder outside calibration by {gripper_feedback['outside_counts']:.1f} counts; powered hold latched; K after valid feedback"
                guard.hold(q,reason);executed=None;last_issued=None;gripper_range_fault=True
                send(sock,dict(type='fault',reason=reason,code='gripper_range',recoverable=True,
                    gripper_feedback=gripper_feedback,hold_confirmed=False))
            if stop or stop_messages:
                guard.hold(q, 'Local shutdown' if stop else stop_messages[-1].get('reason','Stop'))
                last_issued = None;executed=None
            elif args.mode != 'observe':
                for m in messages:
                    if (m.get('op')=='restart' and args.sequence and m.get('epoch',-1)>guard.epoch
                            and 0<=fresh-m.get('issued',0)<=TTL):
                        if gripper_state=='out_of_range':
                            send(sock,dict(type='restart_rejected',reason='Gripper encoder still outside calibration; wait for valid feedback'))
                            continue
                        # bus.read() has just verified torque, health and electrical caps.
                        # Explicit recovery recaptures current encoders; no torque cycling.
                        if (max(abs(a-b) for a,b in zip(q[:6],previous_q[:6]))>4*math.pi/4096+1e-9
                                or abs(q[6]-previous_q[6])>.02):
                            send(sock,dict(type='restart_rejected',reason='Encoder motion detected; wait until stationary'))
                            continue
                        drift_fault=False;gripper_range_fault=False;drift_since=None;motion_mode=None
                        barrier=ClutchBarrier();executed=None;last_issued=None
                        guard.arm(q,m['epoch']);guard.last_command=fresh
                        send(sock,dict(type='restart_accepted',epoch=guard.epoch))
                if drift_fault or gripper_range_fault:
                    messages=[]  # Ordinary reset cannot clear these explicit-recovery latches.
                # Apply a fresh generation before checking release barriers. An old
                # clutch packet must not swallow a newer explicit reset/sleep arm.
                for m in messages:
                    if m.get('op')=='arm' and m['epoch']>guard.epoch and 0<=fresh-m.get('issued',0)<=TTL:
                        motion_mode=None;barrier=ClutchBarrier();executed=None
                        guard.arm(q,m['epoch']);guard.last_command=fresh;last_issued=None
                releases=[m for m in messages if m.get('op')=='clutch' and
                          m.get('epoch')==guard.epoch and 0<=fresh-m.get('issued',0)<=TTL]
                if releases:
                    for m in releases:barrier.release(m)
                    guard.clutch(q,gripper_active=all(m.get('gripper_active',False) for m in releases));executed=None
                for m in messages:
                    if releases:break  # Release wins over queued motion in this generation.
                    if m.get('op') == 'sleep_complete':
                        complete_sleep(bus,guard,m,q,cfg,motion_mode,fresh,args.sequence)
                        torque_disabled=True
                        break
                    elif m.get('op') == 'command':
                        # A reset/sleep transition can overtake an old in-flight result.
                        # Discard it without refreshing the watchdog or latching a new hold.
                        if m['epoch']<guard.epoch or (m['epoch']==guard.epoch and m['seq']<=guard.seq):continue
                        if not barrier.accepts(m):continue
                        if not controller_source_fresh(m,fresh,TTL):
                            guard.hold(q,'Controller target expired at motor execution');executed=None;continue
                        dt = PERIOD if last_issued is None else m['issued']-last_issued
                        if not 0 < dt <= TTL:
                            guard.hold(q,'Invalid command cadence')
                        else:
                            guard.command(m,q,fresh,dt)
                            motion_mode=m.get('mode');executed=m
                        last_issued = m['issued']
            if torque_disabled:continue
            target = guard.tick(q,time.monotonic())
            if guard.state!='ARMED':executed=None
            if args.mode != 'observe':
                requested_profile=motion_mode if guard.state=='ARMED' and motion_mode in ('opening','sleep','teleop') else None
                if requested_profile!=active_profile:
                    bus.opening_speed(False)
                    if requested_profile=='opening':bus.opening_speed(True)
                    elif requested_profile=='sleep':bus.opening_speed(True,counts=14,acceleration_counts=5)
                    elif requested_profile=='teleop':
                        settings=bus.teleop_speed(True,*motion)
                        send(sock,dict(type='motion_profile',details=settings))
                    active_profile=requested_profile
                execution_t=time.monotonic()
                if guard.state=='ARMED' and executed and (execution_t-executed['issued']>TTL or
                        not controller_source_fresh(executed,execution_t,TTL)):
                    target=guard.hold(q,'Target expired before serial write');executed=None
                goal_written=bus.write(target)
            else:
                execution_t=time.monotonic();goal_written=False
            interval=max(fresh-previous_t,1e-6)
            telemetry=dict(execution_t=execution_t,goal_written=goal_written is not False,
                feedback_read_ms=feedback_read_ms,
                command_seq=(executed['seq'] if executed else None),
                command_admitted_t=(executed.get('admitted_t') if executed else None),
                admission_to_execution_ms=(1000*(execution_t-executed['admitted_t'])
                    if executed and executed.get('admitted_t') is not None else None),
                target_age_ms=(1000*(execution_t-executed['issued']) if executed else None),
                input_age_ms=(1000*(execution_t-executed.get('input_t',executed['issued'])) if executed else None),
                controller_age_upper_bound_ms=(1000*(execution_t-executed['controller_t']) if executed and executed.get('controller_t') is not None else None),
                requested_joint_velocity_rad_s=(executed.get('requested_joint_velocity_rad_s') if executed else None),
                executed_goal_velocity_rad_s_estimate=[(a-b)/interval for a,b in zip(target[:6],last_target[:6])],
                measured_joint_velocity_rad_s_estimate=[(a-b)/interval for a,b in zip(q[:6],previous_q[:6])],
                following_error_rad=[a-b for a,b in zip(previous_goal[:6],q[:6])],
                following_error_reference='last executed goal versus current encoder feedback',
                velocity_source='finite difference; not Present_Velocity register',
                serial_write_and_readback_ms=1000*(time.monotonic()-execution_t))
            last_target=target[:];previous_goal=target[:]
            speed=max(abs(a-b) for a,b in zip(q[:6],previous_q[:6]))/max(fresh-previous_t,1e-6)
            settled=speed < .00873 and max(abs(a-b) for a,b in zip(q[:6],target[:6])) < .00873 and abs(q[6]-target[6])<.02
            if guard.state=='HOLD' and (max(abs(a-b) for a,b in zip(q[:6],target[:6])) > .0174533 or abs(q[6]-target[6]) > .02):
                drift_since=fresh if drift_since is None else drift_since
                if fresh-drift_since>.5 and not drift_fault:
                    drift_fault=True
                    guard.reason='Hold drift detected; fixed target retained; press K for explicit restart when encoders are stationary'
                    send(sock,dict(type='fault',reason=guard.reason,hold_confirmed=False,code='hold_drift',recoverable=True))
            else:
                drift_since=None
            previous_q=q[:]; previous_t=fresh
            seq += 1
            send(sock,dict(type='feedback',q=q,goal=target,t=fresh,seq=seq,state=guard.state,
                           epoch=guard.epoch,reason=guard.reason,telemetry=telemetry,hold_command_confirmed=guard.state=='HOLD',hold_confirmed=guard.state=='HOLD' and settled,health=health))
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
