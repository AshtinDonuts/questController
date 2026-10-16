"""Standalone single-goal ALOHA diagnostic. No Quest, MuJoCo, or IK imports."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty
from .core import MOTORS, load_profile, vector
from .bus import SoftwareHoldBus


def pose_values(path, side):
    doc=json.loads(Path(path).read_text())
    if doc.get('robot_type')!='aloha':raise ValueError('Expected ALOHA pose')
    record=doc['joint_values'][side+'_follower']
    names=record['motor_names']; values=vector(record['positions'],len(names))
    if len(set(names))!=len(names):raise ValueError('Duplicate motor names')
    named=dict(zip(names,values))
    if record.get('positions_dict',named)!=named:raise ValueError('Pose records disagree')
    return [named[n] for n in MOTORS]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile',type=Path,default=Path(__file__).with_name('rig.json'))
    ap.add_argument('--pose',type=Path,default=Path('/home/King/alohaX_lerobot_fork/.cache/poses/ceremony_pose.json'))
    ap.add_argument('--side',choices=('left','right'),default='left')
    ap.add_argument('--velocity-counts',type=int,choices=range(1,19),default=18)
    ap.add_argument('--torque-off-on-arrival',action='store_true')
    ap.add_argument('--execute',action='store_true',required=True)
    args=ap.parse_args()
    profile=load_profile(args.profile,live=True,sides=(args.side,),require_motor_watchdog=False)
    values=pose_values(args.pose,args.side)
    output=Path('teleop_sessions')/datetime.now().strftime('direct_pose_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    bus=None; cancelled=False; terminal=None; success=False; last_raw=None; fresh=0.
    def stop(*_):
        nonlocal cancelled
        cancelled=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print('Telemetry:',output.resolve()/'feedback.jsonl',flush=True)
    print(f'One original LeRobot goal, motor velocity capped at ~{args.velocity_counts*1.374:.1f} deg/s. Space/Q/Ctrl+C stops into powered hold.',flush=True)
    with (output/'feedback.jsonl').open('w') as log:
        try:
            bus=SoftwareHoldBus(profile,args.side,startup=True)
            bus.startup_hold(lambda:cancelled,ignore_model_limits=True)
            last_raw=bus.raw.copy();fresh=time.monotonic()
            # Retain present gripper opening; no MuJoCo sign/offset conversion.
            values[-1]=float(bus.bus.apply_calibration(last_raw.copy(),list(MOTORS))[-1])
            target=bus.bus.revert_calibration(bus.np.array(values),list(MOTORS))
            for primary,shadow in ((1,2),(3,4)):
                if abs(int(target[primary])-int(target[shadow]))>2:
                    raise ValueError('Pose primary/shadow calibration mismatch')
                target[shadow]=target[primary]
            target[-1]=last_raw[-1]
            bus.opening_speed(True,counts=args.velocity_counts)
            caps=bus.register('Profile_Velocity')
            if sys.stdin.isatty():terminal=termios.tcgetattr(sys.stdin);tty.setcbreak(sys.stdin.fileno())
            if cancelled:raise RuntimeError('Cancelled before move')
            log.write(json.dumps(dict(event='target',raw=target.tolist(),calibrated=values,profile_velocity=caps))+'\n');log.flush()
            bus.write_raw_hold(target)  # One goal write; do not regenerate profiles every 20 ms.
            start=time.monotonic();settled=None;last_print=0.
            while True:
                if terminal and select.select([sys.stdin],[],[],0)[0]:
                    if sys.stdin.read(1).lower() in (' ','q','\x03'):cancelled=True
                if cancelled:raise RuntimeError('Operator stop')
                before=time.monotonic();q,health=bus.read();now=time.monotonic()
                if now-before>.1:raise RuntimeError('Feedback exceeded 100 ms')
                last_raw=bus.raw.copy();fresh=now
                error=bus.np.abs(target.astype('int64')-last_raw.astype('int64'))
                row=dict(t=now,elapsed=now-start,raw=last_raw.tolist(),error_deg=(error[:8]*360/4096).tolist(),health=health)
                log.write(json.dumps(row)+'\n');log.flush()
                if now-last_print>=1:
                    print(f'{now-start:.1f}s errors deg: '+str([round(x,2) for x in row['error_deg']]),flush=True);last_print=now
                if max(error[:8])<=6 and error[-1]<=2:
                    settled=now if settled is None else settled
                    if now-settled>=2:
                        if args.torque_off_on_arrival:
                            bus.disable_torque()
                            print('POSE REACHED. Motor torque OFF, verified.',flush=True)
                            log.write(json.dumps(dict(event='torque_off',verified=True))+'\n');log.flush()
                        else:print('POSE REACHED. Torque remains ON at the target.',flush=True)
                        success=True;break
                else:settled=None
                if now-start>90:raise RuntimeError('Pose move timed out')
                time.sleep(.05)
        except BaseException as exc:
            if bus and not getattr(bus,'torque_release_attempted',False) and last_raw is not None and time.monotonic()-fresh<=.1:
                try:bus.write_raw_hold(last_raw)
                except Exception:pass
            print('STOPPED:',str(exc),'Torque state unconfirmed after release attempt.' if bus and getattr(bus,'torque_release_attempted',False) else 'Torque has not been disabled.',flush=True)
            log.write(json.dumps(dict(event='fault',reason=str(exc)))+'\n')
        finally:
            if terminal:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,terminal)
            if bus:
                try:
                    if not getattr(bus,'torque_release_attempted',False):bus.opening_speed(False)
                finally:bus.close()
    if not success:raise SystemExit(1)


if __name__=='__main__':main()
