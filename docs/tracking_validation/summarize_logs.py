"""Read-only timing summary of recorded JSONL files; no robot connection."""
import json
import sys
from pathlib import Path
import numpy as np


def summarize(path):
    age=[];result_age=[];following=[];count=0
    for line in Path(path).open():
        r=json.loads(line);c=r.get('command') or {};h=(r.get('hands') or {}).get('left')
        if r.get('event')!='tick' or r.get('state')!='ARMED' or c.get('mode')!='teleop' or not h or not h['grip']:continue
        count+=1
        age.append(1000*(r['t']-h['t']))
        result_age.append(1000*(r['t']-c['issued']))
        f=r.get('feedback',{}).get('left',{})
        if f.get('goal') and f.get('q'):following.append(np.degrees(max(abs(a-b) for a,b in zip(f['goal'][:6],f['q'][:6]))))
    def stats(v):return dict(zip(('median','p95','maximum'),map(float,np.percentile(v,[50,95,100])))) if v else None
    return dict(active_tick_samples=count,controller_freshness_upper_bound_ms=stats(age),
        planner_start_to_supervisor_tick_ms=stats(result_age),feedback_goal_error_deg=stats(following),
        limitations='Tick-weighted, cached samples may repeat. Controller timestamp is server challenge issuance, not XR capture. Old logs cannot separate solve time, one-way network time or serial execution latency. Feedback goal error compares reported goal and encoder position, not optical EEF ground truth.')


if __name__=='__main__':
    print(json.dumps({p:summarize(p) for p in sys.argv[1:]},indent=2))
