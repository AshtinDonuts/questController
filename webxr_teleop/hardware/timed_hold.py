"""Operator-started five-second torque-off movement window, then fixed hold."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import queue
import signal
import threading
import time
from .bus import LeRobotBus
from .core import load_profile


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--side',choices=('left','right'),default='left')
    ap.add_argument('--profile',type=Path,default=Path(__file__).with_name('rig.json'))
    args=ap.parse_args()
    out=Path('teleop_sessions')/datetime.now().strftime('timed_hold_%Y%m%d_%H%M%S');out.mkdir()
    class StationaryBus(LeRobotBus):
        reuse_stationary_watchdog=True
        preparing=True
        def inspect(self):
            if not self.preparing:return super().inspect()
            r={n:self.register(n) for n in ('Model_Number','Torque_Enable','Hardware_Error_Status')}
            if r['Model_Number']!=[1120]*7+[1020]*2 or any(r['Hardware_Error_Status']):
                raise RuntimeError('Motor identity or hardware error check failed')
            return r
    bus=None;attempted=False;cancel=False;commands=queue.Queue();report={}
    def save():
        temp=out/'status.tmp';temp.write_text(json.dumps(report,indent=2)+'\n');temp.replace(out/'status.json')
    def interrupted(*_):
        nonlocal cancel
        if attempted:print('\nHold connection kept active. Support the arm, then type RELEASE to disable torque and exit.',flush=True)
        else:cancel=True
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    def read_commands():
        while True:
            try:commands.put(input().strip())
            except EOFError:return
    try:
        bus=StationaryBus(load_profile(args.profile),args.side,startup=True)
        print('Current torque states:',bus.register('Torque_Enable'),flush=True)
        print('\nSUPPORT THE ARM FIRST. Press ENTER to switch torque OFF and start the five-second movement window.',flush=True)
        print('Status log:',out.resolve()/'status.json',flush=True)
        threading.Thread(target=read_commands,daemon=True).start()
        while commands.empty() and not cancel:time.sleep(.02)
        if cancel:return
        commands.get()
        if any(bus.register('Torque_Enable')):bus.bus.write('Torque_Enable',0)
        if bus.register('Torque_Enable')!=[0]*9:raise RuntimeError('Torque-off verification failed; no countdown')
        bus.preparing=False;bus.preflight=bus.inspect()
        print('All motors verified torque OFF.',flush=True)
        deadline=time.monotonic()+5;last=None
        while time.monotonic()<deadline:
            if cancel:return
            remaining=int(deadline-time.monotonic())+1
            if remaining!=last:print(f'{remaining} ... MOVE / SUPPORT THE ARM',flush=True);last=remaining
            # Read only; do not change torque or goals during movement window.
            if any(bus.register('Torque_Enable')):raise RuntimeError('Torque state changed during countdown')
            time.sleep(.04)
        print('0 — STOP MOVING. Capturing current positions and enabling HOLD.',flush=True)
        attempted=True
        try:
            q,health=bus.startup_hold(ignore_model_limits=True)
            anchor=bus.raw.copy()
            report.update(state='HOLDING',startup=bus.startup_report,anchor_raw=anchor.tolist(),health=health)
            print('HOLD VERIFIED — TORQUE ON. No further motion targets. Connection stays active.',flush=True)
        except Exception as exc:
            anchor=None;report.update(state='FAULT',error=str(exc))
            print('Startup fault:',exc,'No automatic torque change; keeping connection active.',flush=True)
        print('When safely supported, type RELEASE then Enter to disable torque and exit.',flush=True)
        save();last_error=None
        while True:
            if not commands.empty() and commands.get()=='RELEASE':
                bus.bus.write('Torque_Enable',0)
                if bus.register('Torque_Enable')!=[0]*9:raise RuntimeError('Torque-off readback failed')
                report['state']='TORQUE_OFF';save();print('All motors verified torque OFF.',flush=True);break
            try:
                q,health=bus.read();report.update(q=q,health=health,t=time.monotonic())
                if anchor is not None:
                    drift=int(max(abs(int(a)-int(b)) for a,b in zip(bus.raw,anchor)))
                    report['max_drift_ticks']=drift
                    if drift>6 and report['state']!='FAULT':
                        report.update(state='FAULT',error='Hold drift exceeded 6 ticks')
                        print('HOLD DRIFT DETECTED. No target changes or automatic torque release.',flush=True)
                last_error=None
            except Exception as exc:
                report.update(state='FAULT',error=str(exc),t=time.monotonic())
                if str(exc)!=last_error:print('Feedback fault:',exc,'No torque commands issued.',flush=True)
                last_error=str(exc)
            save();time.sleep(.04)
    finally:
        save()
        if bus:bus.close()

if __name__=='__main__':main()
