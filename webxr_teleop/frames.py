"""Render the IK end-effector site's actual local RGB axes in world coordinates."""
import mujoco
import numpy as np


def draw_eef_frame(scene, data, site, length=0.10):
    p = data.site_xpos[site]
    r = data.site_xmat[site].reshape(3, 3)
    for axis in range(3):
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        color = np.zeros(4, dtype=np.float32)
        color[axis] = color[3] = 1
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW,
                           np.zeros(3), np.zeros(3), np.eye(3).ravel(), color)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, 0.004,
                             p, p + length * r[:, axis])
        scene.ngeom += 1
