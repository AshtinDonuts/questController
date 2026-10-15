"""Render a dual-arm tabletop and lateral-plane avoidance demonstration."""
import argparse
from dataclasses import asdict
from pathlib import Path
import json
import mujoco
import numpy as np
from .control import Config, Teleop
from .planar_limits import PlanarBounds
from .recording import Recording
from .run import ROOT, arm_state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(ROOT/'scenes/dual_viperx_scene.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
    cfg=Config(planar_bounds=PlanarBounds(y_min=-.18,y_max=.18))
    arms={side:Teleop(model,data,cfg,side=side) for side in ('right','left')}
    origins={side:a.actual_pose() for side,a in arms.items()}
    recorder=Recording(args.output,model,30,dict(config=asdict(cfg),synthetic=True,
        description='Direct IK targets beyond the tabletop and lateral planes; no goal clipping'),
        planar_limits=[a.ik.planes for a in arms.values()])
    status='failed';rows=[]
    try:
        for frame in range(780):
            t=frame/30
            # Smoothly command into a prohibited region, hold, then return.
            if t<13:
                phase='table'
                amount=np.clip((t-1)/4,0,1) if t<7 else np.clip((12-t)/4,0,1)
            else:
                phase='side_wall'
                amount=np.clip((t-14)/4,0,1) if t<20 else np.clip((25-t)/4,0,1)
            amount=amount*amount*(3-2*amount)
            states={}
            for side,arm in arms.items():
                p,r=origins[side];target=p.copy()
                if phase=='table':
                    target[2]-=.50*amount
                else:
                    target[0]+=(1 if side=='right' else -1)*.40*amount
                assert arm.solve(target,r,1/30)
                arm.target_p,arm.target_r=target,r
                states[side]=arm_state(arm,None)
                assert states[side]['plane_clearance_m']>=-1e-8
            data.time=t
            row=dict(elapsed_s=t,phase=phase,arms=states,qpos=data.qpos.tolist())
            recorder.write(data,row);rows.append(row)
        status='completed'
    finally:
        recorder.close(status)
    summary={s:dict(min_plane_clearance_m=min(r['arms'][s]['plane_clearance_m'] for r in rows),
        max_joint_speed_deg_s=float(np.rad2deg(max(r['arms'][s]['ik_joint_speed_rad_s'] for r in rows))),
        limited_frames=sum(r['arms'][s]['plane_limited'] for r in rows)) for s in arms}
    (args.output/'metrics.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2));print(args.output.resolve()/'session.mp4')


if __name__=='__main__':
    main()
