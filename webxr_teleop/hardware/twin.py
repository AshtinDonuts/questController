"""Passive measured-feedback viewer; never opens a robot bus or sends targets."""
import os
import time
import threading
from .ipc import receive


def run(sock, profile, sides):
    # A dedicated process allows an interactive GLFW window even when the video
    # recorder uses EGL. Closing this window does not stop or command the robot.
    os.environ['MUJOCO_GL']='glfw'
    import mujoco
    import mujoco.viewer
    import numpy as np
    from .planner import make_model, put_measured
    model=make_model(profile,sides);data=mujoco.MjData(model)
    viewer=None
    viewer_threads=[]
    try:
        previous_threads=set(threading.enumerate())
        viewer=mujoco.viewer.launch_passive(model,data)
        viewer_threads=[t for t in threading.enumerate() if t not in previous_threads]
        viewer.cam.lookat[:]=[0,.25,.3]
        viewer.cam.distance=1.3;viewer.cam.azimuth=0;viewer.cam.elevation=-20
        row=None
        while viewer.is_running():
            messages=receive(sock)
            if any(m.get('op')=='shutdown' for m in messages):break
            if messages:row=messages[-1]
            if row:
                with viewer.lock():
                    put_measured(model,data,profile,row['measured'])
                    stale=time.monotonic()-row['t']>.1 or not row.get('feedback_fresh',False)
                    model.light_diffuse[:]=[.7,.05,.05] if stale else [.7,.7,.7]
                    viewer.user_scn.ngeom=0
                    diagnostics=(row.get('command') or {}).get('diagnostics',{})
                    for side in sides:
                        for key,color in [('desired_position',[1,.45,0,1]),('filtered_position',[0,1,1,1])]:
                            pos=diagnostics.get(side,{}).get(key)
                            if pos is None or stale:continue
                            i=viewer.user_scn.ngeom
                            mujoco.mjv_initGeom(viewer.user_scn.geoms[i],mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([.012]*3),np.array(pos),np.eye(3).ravel(),np.array(color,dtype=np.float32))
                            viewer.user_scn.ngeom+=1
                viewer.sync()
            time.sleep(1/30)
    except Exception as exc:
        print(f'Digital twin unavailable: {exc}',flush=True)
    finally:
        if viewer:viewer.close()
        # MuJoCo close() requests exit asynchronously. Let GLFW finish before
        # Python tears down this subprocess (otherwise it can segfault on exit).
        for thread in viewer_threads:thread.join(timeout=2)
        sock.close()
