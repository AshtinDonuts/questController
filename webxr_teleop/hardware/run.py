"""Quest/Mink hardware supervisor; motor I/O and IK are separate processes."""
import argparse
from datetime import datetime
import json
import multiprocessing as mp
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import termios
import time
import tty

from .core import SIDES, PERIOD, TTL, FEEDBACK_TTL, load_profile, validate_sequence, SettlingWindow
from .ipc import pair, send, receive
from .endpoint import bind_control

ROOT=Path(__file__).resolve().parents[2]
DEFAULT_PROFILE=Path(__file__).with_name('example_profile.json')
DEFAULT_SOCKET=f'/tmp/quest-aloha-{os.getuid()}/control.sock'


def spawn_worker(profile, side, mode, python, startup=False, manual_start=False, sequence=False):
    parent,child=pair()
    # Conda/ROS activation can export another interpreter's compiled packages.
    # The worker resolves packages from its own environment and this repo cwd.
    worker_env=os.environ.copy()
    for name in ('PYTHONPATH','PYTHONHOME','PYTHONUSERBASE'):
        worker_env.pop(name,None)
    worker_env['PYTHONNOUSERSITE']='1'
    proc=subprocess.Popen([python,'-m','webxr_teleop.hardware.worker','--profile',str(profile),
        '--side',side,'--mode',mode,'--fd',str(child.fileno()),'--parent-pid',str(os.getpid())]+(['--startup'] if startup else [])+(['--manual-start'] if manual_start else [])+(['--sequence'] if sequence else []),pass_fds=[child.fileno()],
        cwd=ROOT,start_new_session=True,env=worker_env)
    child.close()
    return parent,proc


def hand_dict(hand):
    return None if hand is None else dict(position=hand.position.tolist(),quaternion=hand.quaternion.tolist(),
        trigger=hand.trigger,grip=hand.squeeze,buttons=hand.buttons,t=hand.received_t)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action',nargs='?',choices=('run','status','stop','reset','sleep'),default='run')
    ap.add_argument('--twin',action=argparse.BooleanOptionalAction,default=None,help='Measured-feedback MuJoCo window (default on in live mode)')
    ap.add_argument('--sequence',action='store_true',help='Opening pose, ready hold, Quest control, and hold Y for sleep; requires explicit named poses')
    ap.add_argument('--side',choices=('left','right','both'),default='both')
    ap.add_argument('--manual-start',action='store_true',help='Single-arm five-second torque-off positioning, then software-watchdog powered hold')
    ap.add_argument('--startup',action='store_true',help='Explicit single-arm torque-on and verified current-position hold')
    ap.add_argument('--mode',choices=('mock','observe','live'),default='mock')
    ap.add_argument('--profile',type=Path,default=DEFAULT_PROFILE)
    ap.add_argument('--lerobot-python',default='/home/King/miniconda3/envs/lerobot/bin/python')
    ap.add_argument('--socket',default=DEFAULT_SOCKET)
    ap.add_argument('--port',type=int,default=8443)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--demo',action='store_true',help='Synthetic inputs, mock mode only')
    ap.add_argument('--seconds',type=float,default=0.)
    args=ap.parse_args()
    if args.action!='run':
        client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM); client.settimeout(2)
        client.connect(args.socket); client.sendall((args.action+'\n').encode())
        print(client.recv(65536).decode()); client.close(); return
    if args.demo and args.mode!='mock':
        ap.error('--demo is restricted to mock motors')
    if args.seconds<0:
        ap.error('--seconds must be nonnegative')
    if args.sequence and args.manual_start:
        ap.error('--sequence now starts automatically; omit --manual-start')
    if args.manual_start or args.sequence:args.startup=True
    if args.startup and (args.side=='both' or args.mode=='observe'):
        ap.error('--startup requires --side left/right and mock/live mode')
    sides=SIDES if args.side=='both' else (args.side,)
    profile=load_profile(args.profile,live=args.mode=='live',sides=sides,require_motor_watchdog=not (args.manual_start or args.sequence))
    if args.sequence:
        if args.mode=='observe': ap.error('--sequence requires mock/live mode')
        validate_sequence(profile,sides,live=args.mode=='live')
    from . import planner, render
    from ..server import VRServer
    if args.manual_start and args.mode=='live':
        input('Support the selected arm with torque OFF. Press Enter to begin the five-second positioning countdown: ')
    # Spawn avoids inheriting serial/network handles and the server's background thread.
    ctx=mp.get_context('spawn')
    workers={}; processes=[]; server=None; command_server=None; old_terminal=None
    stopping=False; state='DISARMED'; reason='Waiting for motor startup and stable powered hold' if args.sequence else 'Explicit local reset required'; epoch=0
    parking_requires_tracking=True
    parked=False
    opening_completed=False
    opening_started=None
    previous_grips={s:False for s in sides}
    opening_pending=args.sequence
    feedback={}; measured={s:profile['arms'][s]['initial_rad']+[profile['arms'][s]['initial_gripper']] for s in sides}
    align_requested=False
    aligned=False; generation=None; buttons={}; y_since=None; home_id=0; planner_pending=False
    last_result=None; home_release_required=False; fault=False; worker_armed=False; reset_at=None
    output=args.output or ROOT/'teleop_sessions'/datetime.now().strftime('hardware_%Y%m%d_%H%M%S_%f')
    output=output.resolve(); output.mkdir(parents=True,exist_ok=False)
    log=(output/'hardware.jsonl').open('w')
    twin_sock=None; twin_proc=None
    endpoint_lock=None
    socket_bound=False; stop_this_tick=False; home_button_ready=True
    def handle_signal(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,handle_signal); signal.signal(signal.SIGTERM,handle_signal)
    def event(kind, **kwargs):
        log.write(json.dumps(dict(t=time.monotonic(),event=kind) | kwargs,allow_nan=False)+'\n'); log.flush()
    def hold(why, is_fault=False):
        nonlocal state,reason,epoch,aligned,y_since,last_result,home_release_required,fault,worker_armed,stop_this_tick,align_requested
        stop_this_tick=True; align_requested=False
        if state not in ('HOLD','FAULT') or reason!=why:
            epoch+=1; event('hold',reason=why,epoch=epoch)
        state='FAULT' if is_fault or fault else 'HOLD'; fault|=is_fault
        reason=why; aligned=False; y_since=None; last_result=None; home_release_required=True; worker_armed=False
        for sock,_ in workers.values():
            send(sock,dict(op='hold',reason=why))
    def reset(now,hands,fresh):
        nonlocal state,reason,epoch,aligned,home_release_required,last_result,worker_armed,reset_at,align_requested
        if fault:
            return 'Reset rejected: fault latched; resolve the reported fault and restart'
        if args.sequence and (parked or not opening_completed or state in ('OPENING','PARKING','PARKED')):
            if parked: return 'Reset rejected: sleep completed; start a new session'
            if state=='HOLD': return 'Reset rejected: opening interrupted; restart after resolving the hold cause'
            return 'Reset rejected: wait for automatic opening to reach READY'
        if stop_this_tick:return 'Handover rejected: stop has priority; release controls and retry'
        if args.mode=='observe':return 'Handover rejected: observe mode cannot control motors'
        if not fresh:return 'Handover rejected: motor feedback is stale or unavailable'
        missing=[s for s in SIDES if not hands[s] or not 0<=now-hands[s]['t']<=TTL]
        if missing:return 'Handover rejected: controller tracking missing/stale: '+', '.join(missing)
        gripping=[s for s in SIDES if hands[s]['grip']]
        if gripping:return 'Handover rejected: release side grips: '+', '.join(gripping)
        if hands['right']['buttons'].get('b'):return 'Handover rejected: release right B stop'
        align_requested=False
        epoch+=1; state='ARMED'; reason='Press left X to align'; aligned=False
        home_release_required=False; last_result=None; worker_armed=True; reset_at=now
        for sock,_ in workers.values():
            send(sock,dict(op='arm',epoch=epoch,issued=now))
        event('reset',epoch=epoch)
        return 'Armed in hold; press left X to align'
    def start_sleep(now, source):
        nonlocal state,reason,epoch,aligned,align_requested,last_result,worker_armed,reset_at
        nonlocal home_id,y_since,home_button_ready,parking_requires_tracking,opening_pending
        if not args.sequence:return 'Sleep rejected: requires --sequence'
        if fault:return 'Sleep rejected: hardware/planner fault latched; resolve and restart'
        if stop_this_tick:return 'Sleep rejected: stop has priority; retry after release'
        if state not in ('OPENING','READY','HOLD','ARMED'):return 'Sleep rejected: sleep already active or session not ready'
        if not fresh:return 'Sleep rejected: motor feedback is stale or unavailable'
        epoch+=1;home_id+=1;state='PARKING';aligned=False;align_requested=False
        last_result=None;worker_armed=True;reset_at=now;y_since=None;home_button_ready=False
        opening_pending=False;parking_requires_tracking=False
        for sock,_ in workers.values():send(sock,dict(op='arm',epoch=epoch,issued=now))
        event('sleep_start',home_id=home_id,source=source)
        return 'Slow sleep trajectory; Quest motion ignored'
    try:
        command_server,endpoint_lock=bind_control(args.socket)
        socket_bound=True
        # Diagnose an occupied HTTPS port before starting any motor worker.
        if not args.demo:
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
                probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try:probe.bind(('0.0.0.0',args.port))
                except OSError as exc:
                    raise RuntimeError(f'Quest HTTPS port {args.port} is unavailable; close its existing server before relaunching') from exc
        for s in sides:
            workers[s]=spawn_worker(args.profile.resolve(),s,args.mode,
                sys.executable if args.mode=='mock' else args.lerobot_python,startup=args.startup,manual_start=args.manual_start,sequence=args.sequence)
        planner_sock,child=pair(); planner_proc=ctx.Process(target=planner.run,args=(child,profile,sides)); planner_proc.start(); child.close()
        video_sock,child=pair(); video_proc=ctx.Process(target=render.run,args=(child,profile,str(output/'video'),args.mode,sides)); video_proc.start(); child.close()
        processes=[(planner_sock,planner_proc),(video_sock,video_proc)]
        if args.twin if args.twin is not None else args.mode=='live':
            from . import twin
            twin_sock,child=pair();twin_proc=ctx.Process(target=twin.run,args=(child,profile,sides))
            twin_proc.start();child.close();processes.append((twin_sock,twin_proc))
            print('Digital twin: measured arm; orange = hand target, cyan = filtered target. Red lighting = stale feedback.',flush=True)
        if not args.demo:
            server=VRServer(port=args.port,hardware_freshness=True); server.start()
            print('Quest URL:',server.url,flush=True)
        if sys.stdin.isatty():
            old_terminal=termios.tcgetattr(sys.stdin); tty.setcbreak(sys.stdin.fileno())
        if args.sequence: print('Sequence: opening → READY; left X hands over + aligns; grip moves, trigger grips; hold left Y 1 s or terminal S → sleep (also from HOLD); B stops.',flush=True)
        if not args.sequence: print('Space: powered stop. R: reset. Quest X: align. Hold Y 1 s: home. B: stop.',flush=True)
        print('MP4:',output/'video/session.mp4',flush=True)
        event('startup',mode=args.mode,profile=profile,sides=sides,enable_torque=args.startup,manual_start=args.manual_start,sequence=args.sequence,watchdog_policy='software_hold' if args.manual_start or args.sequence else 'motor_and_software')
        start=time.monotonic(); next_tick=start; demo_reset=False; demo_start=None
        last_print=0.; clients=[]
        settling={s:SettlingWindow() for s in sides}
        settled={s:False for s in sides}; settled_seq={s:None for s in sides}
        while not stopping:
            stop_this_tick=False
            now=time.monotonic(); elapsed=now-start
            if args.seconds and elapsed>=args.seconds:
                break
            for s,(sock,proc) in workers.items():
                for msg in receive(sock):
                    if msg['type']=='feedback':
                        feedback[s]=msg; measured[s]=msg['q']; feedback[s]['available']=True
                    elif msg['type']=='fault':
                        event('worker_fault',side=s,**msg); hold(s+': '+msg['reason'],True)
                    else:
                        event('preflight',side=s,details=msg)
                if proc.poll() is not None and not fault:
                    hold(s+' motor worker exited',True)
            for s,(_,proc) in workers.items():
                if s in feedback and (proc.poll() is not None or now-feedback[s]['t']>FEEDBACK_TTL):
                    feedback[s]['available']=False
                    feedback[s]['hold_confirmed']=False
                    feedback[s]['hold_command_confirmed']=False
            fresh=all(s in feedback and 0<=now-feedback[s]['t']<=FEEDBACK_TTL for s in sides)
            for s in sides:
                if not fresh:
                    settling[s]=SettlingWindow(); settled[s]=False; settled_seq[s]=None
                elif feedback[s]['seq'] != settled_seq[s]:
                    f=feedback[s]
                    settled[s]=settling[s].update(f['q'],f['goal'],f['t'])
                    settled_seq[s]=f['seq']
            stationary=fresh and all(settled.values())
            if state=='DISABLING':
                if fresh and all(feedback[s].get('state')=='PARKED' and feedback[s]['health'].get('torque') is False for s in sides):
                    state='PARKED'; reason='Sleep reached; motor torque off; restart for a new session'
                    event('sleep_torque_off')
                elif now-reset_at>TTL:
                    hold('Sleep torque-off acknowledgement expired',True)
            if args.demo:
                if fresh and demo_start is None and (not args.sequence or state=='READY'): demo_start=now
                demo_elapsed=0. if demo_start is None else now-demo_start
                hands={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],trigger=0.,grip=False,buttons={},t=now) for s in SIDES}
                frame_generation=0
                hands['left']['buttons']['x']=1.0<demo_elapsed<1.3
                for s in sides:
                    hands[s]['grip']=2<demo_elapsed<6
                    hands[s]['position'][0]=(.025 if s=='right' else -.025)*max(0,min(1,(demo_elapsed-2.5)/2))
                hands['left']['buttons']['y']=7<demo_elapsed<8.4
                # Large ignored controller motions during homing.
                if 8.5<demo_elapsed<10:
                    hands['right']['position'][0]=.4
                    hands['right']['trigger']=1.
                if 12<demo_elapsed<12.3:
                    hands['right']['buttons']['b']=True
                if demo_elapsed>13:
                    hands['left']=None
            else:
                left,right,frame_generation=server.snapshot()
                hands={'left':hand_dict(left),'right':hand_dict(right)}
            tracking=all(hands[s] is not None and 0<=now-hands[s]['t']<=TTL for s in SIDES)
            if opening_pending and (stop_this_tick or state in ('HOLD','FAULT')):
                opening_pending=False
            if opening_pending and fresh and stationary and all(feedback[s].get('hold_command_confirmed') for s in sides):
                opening_pending=False; opening_started=now; epoch+=1; state='OPENING'; home_id+=1
                worker_armed=True; reset_at=now; last_result=None
                reason='Slow opening ceremony; Quest motion ignored'
                for sock,_ in workers.values(): send(sock,dict(op='arm',epoch=epoch,issued=now))
                event('opening_start',home_id=home_id)
            if generation is None:
                generation=frame_generation
            elif frame_generation!=generation:
                generation=frame_generation
                # Initial headset connection cannot invalidate a control reference
                # before startup feedback or an explicit reset exists.
                if (args.sequence and (state=='ARMED' or (state=='PARKING' and parking_requires_tracking))) or (not args.sequence and (state!='DISARMED' or feedback)):
                    hold('Quest reference/connection changed')
            if state in ('ARMED','HOMING','OPENING','PARKING') and (not fresh or (state!='OPENING' and not (state=='PARKING' and not parking_requires_tracking) and not tracking)):
                if not fresh:
                    hold('Feedback watchdog expired')
                else:
                    missing=[f'{s} missing' if hands[s] is None else f'{s} age {(now-hands[s]["t"])*1000:.0f} ms'
                             for s in SIDES if hands[s] is None or not 0<=now-hands[s]['t']<=TTL]
                    hold('Controller tracking lost: '+', '.join(missing))
            if state in ('ARMED','HOMING','OPENING','PARKING') and reset_at is not None and now-reset_at>TTL and any(feedback.get(s,{}).get('epoch')!=epoch for s in sides):
                hold('Motor reset acknowledgement expired')
            if state in ('ARMED','HOMING','OPENING','PARKING') and any(feedback.get(s,{}).get('state')=='HOLD' and feedback[s]['t']>now-.1 for s in sides):
                # Allow reset message one worker cycle to arrive; epoch is checked below via ack.
                if all(feedback.get(s,{}).get('epoch')==epoch for s in sides):
                    causes='; '.join(s+': '+feedback[s].get('reason','unknown') for s in sides if feedback[s].get('state')=='HOLD')
                    hold('Motor hold: '+causes)
            current={b:bool(hands[s] and hands[s]['buttons'].get(k)) for b,s,k in
                [('x','left','x'),('y','left','y'),('b','right','b')]}
            if not current['y']: home_button_ready=True
            if current['b']:
                hold('Quest B stop')
            do_reset=False; do_sleep=False
            if old_terminal is not None and select.select([sys.stdin],[],[],0)[0]:
                key=os.read(sys.stdin.fileno(),1)
                if key==b' ': hold('PC Space stop')
                elif key.lower()==b'r': do_reset=True
                elif key.lower()==b's': do_sleep=True
                elif key in (b'q',b'\x03'): stopping=True; hold('PC shutdown')
            try:
                client,_=command_server.accept()
            except BlockingIOError:
                client=None
            if client:
                client.setblocking(False); clients.append((client,now))
            for client,accepted in clients[:]:
                try:
                    raw=client.recv(64)
                except BlockingIOError:
                    if now-accepted<.2:
                        continue
                    raw=b''
                except ConnectionError:
                    raw=b''
                command=raw.decode(errors='replace').strip()
                if command=='stop': hold('Local stop command')
                elif command=='reset': reason=reset(now,hands,fresh)
                elif command=='sleep': reason=start_sleep(now,'local')
                try:
                    client.send(json.dumps(dict(state=state,reason=reason,feedback=feedback,processes=dict(planner=planner_proc.pid,renderer=video_proc.pid,motors={s:p.pid for s,(_,p) in workers.items()}))).encode())
                except (BlockingIOError,ConnectionError):
                    pass
                client.close(); clients.remove((client,accepted))
            if args.demo and demo_elapsed>.7 and not demo_reset and fresh:
                demo_reset=True; do_reset=True
            if do_reset:
                reason=reset(now,hands,fresh); print(reason,flush=True)
            if do_sleep:
                reason=start_sleep(now,'local'); print(reason,flush=True)
            if state=='OPENING' and opening_started is not None and now-opening_started>=10.:
                # Discard queued trajectory results via a new epoch; workers capture
                # current feedback once. Explicit reset/alignment still owns teleop.
                hold('Opening 10-second limit reached; holding current pose; release both grips and press left X')
                opening_completed=True; state='READY'
                event('opening_time_limit',duration=now-opening_started,target_reached=False)
            if (state=='HOLD' and args.sequence and opening_completed and not fault
                    and reason.startswith('Controller tracking')):
                reason=('Controller tracking restored; release both grips and press left X to re-arm'
                        if tracking else 'Controller tracking lost; waiting for fresh data from both controllers')
            # A deliberate new X press can recover a hold after opening. reset()
            # retains tracking, feedback, grip, stop and latched-fault checks.
            if state in ('READY','HOLD') and args.sequence and current['x'] and not buttons.get('x'):
                reason=reset(now,hands,fresh)
                if state=='ARMED':align_requested=True
            if args.sequence and state in ('OPENING','READY','HOLD','ARMED'):
                if home_button_ready and current['y'] and hands['left'] and 0<=now-hands['left']['t']<=TTL:
                    y_since=y_since or now
                    if now-y_since>=1:
                        reason=start_sleep(now,'quest')
                else:y_since=None
            mode='hold'
            if state=='ARMED':
                if home_release_required:
                    if tracking and all(not hands[s]['grip'] for s in sides):
                        home_release_required=False
                elif (align_requested or (current['x'] and not buttons.get('x'))) and tracking and all(not hands[s]['grip'] for s in SIDES):
                    align_requested=True; mode='align'
                elif aligned:
                    mode='teleop'
                if not args.sequence and home_button_ready and current['y'] and tracking and all(not hands[s]['grip'] for s in sides):
                    y_since=y_since or now
                    if now-y_since>=1 and stationary:
                        state='PARKING' if args.sequence else 'HOMING'; home_id+=1; mode='sleep' if args.sequence else 'home'; y_since=None; reason='Slow sleep trajectory; Quest motion ignored' if args.sequence else 'Slow synchronized home; Quest motion ignored'
                        home_button_ready=False
                        event('sleep_start' if args.sequence else 'home_start',home_id=home_id)
                elif not args.sequence:
                    y_since=None
            elif state in ('HOMING','OPENING','PARKING'):
                mode={'HOMING':'home','OPENING':'opening','PARKING':'sleep'}[state]
            for s,(sock,_) in workers.items():
                gripping=bool(hands[s] and hands[s]['grip'])
                if state=='ARMED' and previous_grips[s] and not gripping:
                    send(sock,dict(op='clutch',epoch=epoch,issued=now))
                previous_grips[s]=gripping
            buttons=current
            for result in receive(planner_sock):
                planner_pending=False
                if result['type']=='fault':
                    if result.get('code')=='serialization' and result.get('recoverable') is True:
                        if result.get('epoch')==epoch:hold('Planner reporting error: '+result['reason'])
                    else:hold('Planner: '+result['reason'],True)
                elif result['type']=='expired':
                    if result.get('epoch')==epoch and result.get('mode')==mode and state in ('ARMED','HOMING','OPENING','PARKING'):
                        hold('Planner deadline expired')
                elif result['epoch']==epoch and state in ('ARMED','HOMING','OPENING','PARKING'):
                    if not 0<=now-result['input_t']<=TTL:
                        hold('Expired planner result'); continue
                    # Results computed before a mode transition cannot override the new mode.
                    if result['mode']!=mode and not (result['mode']=='align' and mode=='hold'):
                        continue
                    last_result=result
                    if result['mode']=='opening':
                        reason=f"Opening (10 s maximum); maximum joint error {result['arrival_error_deg']:.2f} degrees; wait for READY"
                    if result['mode']=='align': aligned=True; align_requested=False; reason='Aligned; grip enables matching arm/gripper'
                    if result['mode']=='teleop':
                        reason=('Catching up; holding bounded target until motors follow' if any(d.get('catching_up') for d in result.get('diagnostics',{}).values()) else 'Aligned; grip enables matching arm/gripper')
                    if result['home_done'] and args.sequence:
                        completed=result['mode']
                        opening_completed |= completed=='opening'
                        parked |= completed=='sleep'
                        if completed=='sleep':
                            state='DISABLING'; reason='Sleep reached; disabling motor torque'
                            reset_at=now; last_result=None; worker_armed=False
                            for sock,_ in workers.values():
                                send(sock,dict(op='sleep_complete',epoch=epoch,issued=now))
                        else:
                            hold('Opening reached; release both grips and press left X')
                            state='READY'
                        mode='hold'; event('opening_done' if completed=='opening' else 'sleep_done')
                        continue
                    if result['home_done']:
                        state='ARMED'; mode='hold'; home_release_required=True
                        reason='Home reached; release both grips then squeeze to resume'; event('home_done')
                    for s,(sock,_) in workers.items():
                        send(sock,dict(op='command',epoch=epoch,seq=result['seq'],issued=result['issued'],
                            q=result['q'][s],mode=result['mode'],trajectory_start=result.get('trajectory_start',False),
                            trajectory_id=result.get('trajectory_id'),freeze=result['freeze'][s] or (mode=='teleop' and not hands[s]['grip'])))
            if not planner_proc.is_alive() and not fault:
                hold('Planner process exited',True)
            if fresh and not planner_pending and planner_proc.is_alive():
                # Hold jobs initialize/validate geometry even before arming.
                seq=int(now*1e9)
                planner_pending=send(planner_sock,dict(t=now,seq=seq,epoch=epoch,mode=mode,
                    home_id=home_id,measured=measured,hands=hands))
            row=dict(t=now,elapsed=elapsed,state=state,reason=reason,measured=measured,
                feedback=feedback,feedback_fresh=fresh,hands=hands,command=last_result)
            event('tick',**row)
            if twin_proc is not None and twin_proc.is_alive():send(twin_sock,row)
            if video_proc.is_alive(): send(video_sock,row)
            elif now-last_print>1: print('Video recorder unavailable; hardware watchdog remains active',flush=True)
            if server:
                server.publish(dict(type='status',tracking=tracking,calibrated=aligned,message=f'{state}: {reason}',phase=state.lower()))
            if now-last_print>1:
                print(f'{state}: {reason}',flush=True); last_print=now
            next_tick=max(next_tick+PERIOD,now)
            time.sleep(max(0,next_tick-time.monotonic()))
    finally:
        hold('Supervisor shutdown')
        for sock,proc in workers.values(): send(sock,dict(op='shutdown',reason='Supervisor shutdown'))
        for sock,proc in workers.values():
            try: proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try: proc.wait(timeout=.5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            sock.close()
        for sock,proc in processes:
            send(sock,dict(op='shutdown'))
            proc.join(timeout=3)
            if proc.is_alive():
                proc.kill(); proc.join(timeout=1)
            sock.close()
        if server: server.stop()
        if old_terminal is not None: termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_terminal)
        if command_server:
            command_server.close()
            if socket_bound: Path(args.socket).unlink(missing_ok=True)
        if endpoint_lock:endpoint_lock.close()
        event('shutdown'); log.close()


if __name__=='__main__':
    main()
