"""Base-fixed half-spaces and an infinite tabletop plane for moving arm geometry.

Each collision geom is conservatively enclosed by its compiled local bounding box.
No environment-object, self-collision or arm-to-arm pairs are constructed.
"""
from dataclasses import dataclass
from itertools import product

import mink
import mujoco
import numpy as np
from mink.limits import Constraint


@dataclass
class PlanarBounds:
    x_min: float = -0.40
    x_max: float = 0.70
    y_min: float = -0.50
    y_max: float = 0.50
    z_max: float = 0.75
    clearance: float = 0.005

    def __post_init__(self):
        if not all(np.isfinite(x) for x in vars(self).values()):
            raise ValueError('Planar bounds must be finite')
        if self.clearance < 0 or self.x_max-self.x_min <= 2*self.clearance or self.y_max-self.y_min <= 2*self.clearance or self.z_max <= self.clearance:
            raise ValueError('Planar bounds must have positive interior and nonnegative clearance')


class PlanarLimit(mink.Limit):
    def __init__(self, model, data, base_body, bounds=None):
        if not np.all(np.isin(model.jnt_type,[mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE])):
            raise ValueError('Planar guarding requires a fixed-base hinge/slide model')
        self.model = model
        self.bounds = bounds or PlanarBounds()
        base = model.body(base_body).id
        # These transforms are deliberately captured once, never from the EEF.
        self.origin = data.xpos[base].copy()
        self.rotation = data.xmat[base].reshape(3, 3).copy()
        b = self.bounds
        normals_local = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,-1]],float)
        self.normals = normals_local @ self.rotation.T
        self.offsets = np.array([b.x_min,-b.x_max,b.y_min,-b.y_max,-b.z_max]) + self.normals@self.origin
        table = model.geom('table').id
        if model.geom_type[table] not in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_PLANE):
            raise ValueError('The table must be a box or plane')
        normal = data.geom_xmat[table].reshape(3, 3)[:, 2]
        top = data.geom_xpos[table].copy()
        if model.geom_type[table] == mujoco.mjtGeom.mjGEOM_BOX:
            top += normal*model.geom_size[table, 2]
        self.normals = np.vstack([self.normals, normal])
        self.offsets = np.r_[self.offsets, normal@top]
        self.names = ('base_x_min','base_x_max','base_y_min','base_y_max','base_z_max','table')
        descendants = set()
        for body in range(model.nbody):
            ancestor = body
            while ancestor and ancestor != base:
                ancestor = model.body_parentid[ancestor]
            if ancestor == base and body != base:
                descendants.add(body)
        # The fixed mounting base is exempt. Only robot collision geometry, never
        # task objects, enters the constraint. Visual duplicate meshes are excluded.
        self.geom_ids = np.array([g for g in range(model.ngeom)
            if model.geom_bodyid[g] in descendants and (model.geom_contype[g] or model.geom_conaffinity[g])])
        if not len(self.geom_ids):
            raise ValueError('No moving collision geometry found for planar constraints')
        signs = np.array(list(product((-1.,1.),repeat=3)))
        aabb = model.geom_aabb[self.geom_ids]
        self.corners = aabb[:,None,:3] + signs[None,:,:]*aabb[:,None,3:]
        self.body_ids = np.repeat(model.geom_bodyid[self.geom_ids], 8)
        self.scratch = mujoco.MjData(model)

    def points(self, data):
        rotations = data.geom_xmat[self.geom_ids].reshape(-1,3,3)
        return (np.einsum('gij,gkj->gki',rotations,self.corners)
                + data.geom_xpos[self.geom_ids,None,:]).reshape(-1,3)

    def distances(self, data):
        """Signed clearance after the configured margin, for each corner/plane."""
        return self.points(data)@self.normals.T - self.offsets - self.bounds.clearance

    def compute_qp_inequalities(self, configuration, dt):
        data = configuration.data
        points = self.points(data)
        distances = points@self.normals.T-self.offsets-self.bounds.clearance
        # Only nearby corners need Jacobians; the finite-step guard below checks all.
        point_ids, plane_ids = np.nonzero(distances < 0.05)
        if not len(point_ids):
            return Constraint()
        G = np.empty((len(point_ids), self.model.nv))
        jp = np.empty((3,self.model.nv)); jr = np.empty_like(jp)
        for row,(point_id,plane_id) in enumerate(zip(point_ids,plane_ids)):
            mujoco.mj_jac(self.model,data,jp,jr,points[point_id],int(self.body_ids[point_id]))
            G[row] = -self.normals[plane_id]@jp
        return Constraint(G=G,h=0.8*np.maximum(distances[point_ids,plane_ids],0.))

    def safe_fraction(self, start, candidate):
        """Backtrack a finite step until its sampled path respects every plane.

        QP constraints are linearized. Rechecking FK prevents nonlinear endpoint
        penetration; intermediate samples are <=0.25 deg / 0.25 mm apart for this
        hinge/slide model. This is not general continuous collision detection.
        An already-invalid state is held and reported, never teleported to safety.
        """
        def valid(q):
            self.scratch.qpos[:] = q
            mujoco.mj_kinematics(self.model,self.scratch)
            return bool(np.min(self.distances(self.scratch)) >= -1e-8)
        if not valid(start):
            return 0.
        delta = candidate-start
        thresholds = np.where(self.model.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE, .00025, np.deg2rad(.25))
        for fraction in (1., .5, .25, .125, .0625, .03125, .015625):
            steps = max(1,int(np.ceil(np.max(np.abs(delta)*fraction/thresholds))))
            if all(valid(start+delta*fraction*i/steps) for i in range(1,steps+1)):
                return fraction
        return 0.
