"""Supervised stationary-hold bus-watchdog test. Never disables torque or clears watchdog faults."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
from .core import load_profile
from .bus import LeRobotBus


def evaluate(before, samples):
    reasons=[]
    for s in samples:
        if s['torque'] != [1]*9:reasons.append('Torque did not remain enabled on all motors')
        if s['watchdog'] != [255]*9:reasons.append('Watchdog did not latch on all motors')
        if any(s['errors']):reasons.append('Hardware error reported')
        if s['goal'] != before['goal']:reasons.append('Stored hold goal changed')
        if max(abs(a-b) for a,b in zip(s['raw'],before['raw']))>5:reasons.append('Encoder drift exceeded 5 ticks (0.44 degrees)')
        if any(abs(v)>1 for v in s['velocity']):reasons.append('Motor velocity did not remain near zero')
    if not samples:reasons.append('No post-timeout feedback')
    return dict(passed=not reasons,reasons=sorted(set(reasons)))


def snapshot(bus):
    signed=lambda n:bus.np.asarray(bus.register(n),dtype=bus.np.uint32).view(bus.np.int32).tolist()
    return dict(t=time.monotonic(),raw=bus.read_raw().tolist(),goal=signed('Goal_Position'),
        torque=bus.register('Torque_Enable'),watchdog=bus.register('Bus_Watchdog'),
        errors=bus.register('Hardware_Error_Status'),velocity=signed('Present_Velocity'))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--side',choices=('left','right'),default='left')
    ap.add_argument('--profile',type=Path,default=Path(__file__).with_name('rig.json'))
    ap.add_argument('--output',type=Path)
    ap.add_argument('--supported-and-stationary',action='store_true',required=True,
                    help='Operator has supported the arm in a stationary pose')
    ap.add_argument('--ignore-model-limits',action='store_true',help='Stationary watchdog diagnostic only: bypass model angle bounds, retain raw hold checks')
    args=ap.parse_args();profile=load_profile(args.profile)
    out=args.output or Path('teleop_sessions')/datetime.now().strftime('watchdog_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    report=dict(side=args.side,profile=str(args.profile.resolve()),passed=False,torque_disabled=False,watchdog_cleared=False)
    bus=None
    try:
        class StationaryWatchdogBus(LeRobotBus):
            reuse_stationary_watchdog=True
        bus=StationaryWatchdogBus(profile,args.side,startup=True)
        # Preload, cap, arm watchdog, enable torque once, and verify stationary hold.
        bus.startup_hold(ignore_model_limits=args.ignore_model_limits)
        report['startup']=bus.startup_report
        report['model_limits_bypassed']=args.ignore_model_limits
        before=snapshot(bus);report['before']=before
        if before['torque'] != [1]*9 or before['watchdog'] != [15]*9 or max(abs(a-b) for a,b in zip(before['goal'],before['raw']))>2:
            raise RuntimeError('Pre-timeout hold/watchdog verification failed')
        print('Fixed hold established. Pausing ALL motor-bus traffic for 0.65 seconds.',flush=True)
        begin=time.monotonic();time.sleep(.65)
        report['bus_silence_seconds']=time.monotonic()-begin
        samples=[];report['samples']=samples
        for _ in range(5):
            samples.append(snapshot(bus));time.sleep(.05)
        report.update(evaluate(before,samples))
        print(json.dumps({k:report[k] for k in ('passed','reasons')},indent=2),flush=True)
        print('Torque left enabled. Watchdog fault left latched; no automatic recovery. Profile verification flag unchanged.',flush=True)
    except BaseException as exc:
        report['error']=str(exc);raise
    finally:
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Report:',out.resolve()/'result.json',flush=True)
        if bus:bus.close()


if __name__=='__main__':main()
