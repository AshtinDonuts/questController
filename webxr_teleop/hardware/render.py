"""Best-effort asynchronous measured-state video. Never runs in a motor worker."""
import time
import tempfile
import textwrap
from pathlib import Path
import numpy as np
import mujoco
from .core import SIDES
from .planner import make_model, put_measured
from .ipc import receive
from ..recording import Recording


def run(sock, profile, output, mode, sides=SIDES):
    model=make_model(profile,sides); data=mujoco.MjData(model)
    overlay=tempfile.NamedTemporaryFile(prefix='quest-overlay-',suffix='.txt',delete=False)
    overlay.close(); Path(overlay.name).write_text('Starting measured-state recording')
    recording=Recording(output,model,30,dict(hardware=True,mode=mode,mock_motors=mode=='mock',profile=profile,
        note='Measured feedback, not commanded poses. Stale feedback frames have a red tint.'),
        video_filter=f'drawtext=textfile={overlay.name}:reload=1:fontcolor=white:fontsize=17:box=1:boxcolor=black@0.8:x=8:y=8')
    recording.sites=[site for site in recording.sites if model.site(site).name.split('/')[0] in sides]
    row=None; deadline=time.monotonic()
    try:
        while True:
            messages=receive(sock)
            if any(m.get('op')=='shutdown' for m in messages):
                break
            if messages:
                row=messages[-1]
            now=time.monotonic()
            if row and now>=deadline:
                put_measured(model,data,profile,row['measured'])
                if now-row['t']>.1 or not row.get('feedback_fresh',False):
                    # Visible warning tint even if the supervisor or feedback disappears.
                    model.light_diffuse[:]=[.7,.05,.05]
                    row=dict(row,video_warning='STALE FEEDBACK - HOLD UNCONFIRMED')
                else:
                    model.light_diffuse[:]=[.7,.7,.7]
                label=f"{mode.upper()}  {row['elapsed']:.1f}s  {row['state']}\n"+textwrap.fill(row.get('video_warning',row['reason']),65)
                Path(overlay.name).write_text(label)
                recording.write(data,row)
                deadline=now+1/30
            time.sleep(.002)
    finally:
        recording.close()
        Path(overlay.name).unlink(missing_ok=True)
        sock.close()
