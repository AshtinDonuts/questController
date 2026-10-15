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


def draw_planar_bounds(scene, planes):
    """Wireframe of the base-fixed permitted volume; table supplies the bottom."""
    b = planes.bounds
    c = b.clearance
    corners = []
    normal = planes.normals[-1]
    for x in (b.x_min+c,b.x_max-c):
        for y in (b.y_min+c,b.y_max-c):
            # Intersection with the actual tabletop plane at this base-frame XY.
            anchor = planes.origin+planes.rotation@np.array([x,y,0.])
            z_low = (planes.offsets[-1]+c-normal@anchor)/(normal@planes.rotation[:,2])
            for z in (z_low,b.z_max-c):
                corners.append(planes.origin+planes.rotation@np.array([x,y,z]))
    for i in range(8):
        for bit in (1,2,4):
            j=i^bit
            if j<=i or scene.ngeom>=scene.maxgeom:
                continue
            geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),
                               np.eye(3).ravel(),np.array([.1,.8,.9,.6],np.float32))
            mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,2.,corners[i],corners[j])
            scene.ngeom+=1
