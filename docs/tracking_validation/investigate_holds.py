"""Read-only hold/timing investigation. No serial or network access."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
NAMES=('20261026_163032_965319','20261026_165811_759421')


def analyze(name):
    path=ROOT/'teleop_sessions'/f'hardware_{name}'/'hardware.jsonl'
    rows=[]
    for line in path.open():
        try:r=json.loads(line)
        except ValueError:continue  # A currently running logger may have an incomplete final line.
        rows.append(r)
        if r.get('event')=='hold' and 'Sleep settling timed out' in r.get('reason',''):break
    samples={};holds=[];seen_cmd=None;seen_fb=None;previous_active=False;last_fb=None;last_tick=None
    elapsed_active=0.;gripper_states=set();skipped=0
    def add(k,v):
        if v is not None:samples.setdefault(k,[]).append(v)
    for r in rows:
        if r.get('event')=='hold' and any(s in r.get('reason','').lower() for s in ('controller tracking','controller target expired')):
            holds.append(dict(elapsed_s=r['t']-rows[0]['t'],reason=r['reason']))
        if r.get('event')!='tick':continue
        c=r.get('command') or {};h=(r.get('hands') or {}).get('left');f=(r.get('feedback') or {}).get('left',{})
        active=r.get('state')=='ARMED' and c.get('mode')=='teleop'
        if active and previous_active:elapsed_active+=r['t']-last_tick
        if active:
            if h:add('controller_freshness_upper_bound_ms',1000*(r['t']-h['t']))
            if c['seq']!=seen_cmd:
                seen_cmd=c['seq'];add('planner_duration_ms',c.get('planner_duration_ms'))
                if c.get('controller_t',{}).get('left') is not None:
                    source_age=1000*(r['t']-c['controller_t']['left'])
                    if source_age>150 and h:
                        samples.setdefault('expired_forward_examples',[]).append(dict(elapsed_s=r['elapsed'],
                            result_source_age_ms=source_age,latest_controller_age_ms=1000*(r['t']-h['t']),
                            planner_job_age_ms=1000*(r['t']-c['input_t'])))
            if f and f['seq']!=seen_fb:
                seen_fb=f['seq'];t=f.get('telemetry') or {}
                add('serial_write_and_readback_ms',t.get('serial_write_and_readback_ms'))
                add('feedback_packet_bytes',len(json.dumps(f)))
                if previous_active and last_fb:
                    add('feedback_period_ms',1000*(f['t']-last_fb['t']))
                    skipped+=max(0,f['seq']-last_fb['seq']-1)
                last_fb=f
            state=(f.get('health',{}).get('gripper_feedback') or {}).get('state')
            if state:gripper_states.add(state)
        previous_active=active;last_tick=r['t']
    examples=samples.pop('expired_forward_examples',[])[:6]
    stats={k:dict(zip(('median','p95','p99','maximum'),map(float,np.percentile(v,[50,95,99,100])))) for k,v in samples.items()}
    p=rows[0]['profile']
    return dict(source=str(path.relative_to(ROOT)),active_seconds=elapsed_active,tracking_holds=holds,
        metrics=stats,expired_forward_examples=examples,gripper_states=sorted(gripper_states),skipped_feedback_sequences=skipped,
        arm_settings={k:p.get(k) for k in ('translation_scale','rotation_scale','teleop_joint_speed_deg_s','teleop_joint_acceleration_deg_s2','teleop_catch_up')})


if __name__=='__main__':
    print(json.dumps({name:analyze(name) for name in NAMES},indent=2))
