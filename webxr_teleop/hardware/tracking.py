"""Controller pose deadband with continuous motion outside the quiet region."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def tracking_deadbands(profile):
    values=[]
    for key,default in (('tracking_position_deadband_m',.002),('tracking_rotation_deadband_deg',1.)):
        value=profile.get(key,default)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
            raise ValueError(f'{key} must be a finite nonnegative number')
        values.append(float(value))
    return values[0],math.radians(values[1])


class PoseDeadband:
    def __init__(self, position, rotation, position_threshold, rotation_threshold):
        self.position=np.array(position,dtype=float,copy=True)
        self.rotation=np.array(rotation,dtype=float,copy=True)
        self.position_threshold=position_threshold
        self.rotation_threshold=rotation_threshold

    def update(self, position, rotation):
        delta=np.asarray(position)-self.position
        distance=np.linalg.norm(delta)
        if distance>self.position_threshold:
            self.position+=delta*(1-self.position_threshold/distance)
        delta_r=Rotation.from_matrix(rotation@self.rotation.T).as_rotvec()
        angle=np.linalg.norm(delta_r)
        if angle>self.rotation_threshold:
            self.rotation=Rotation.from_rotvec(delta_r*(1-self.rotation_threshold/angle)).as_matrix()@self.rotation
        return self.position.copy(),self.rotation.copy()


class PoseMotionGate:
    """Require accumulated controller motion to keep IK pursuing its target.

    A short grace interval bridges repeated XR samples. Compare against an
    activity anchor, not the previous sample, so slow deliberate motion counts.
    """
    def __init__(self, position, rotation, now):
        self.position=np.array(position,copy=True)
        self.rotation=np.array(rotation,copy=True)
        # Clutch engagement alone must not activate posture-seeking motion.
        self.last_motion=-math.inf

    def moving(self, position, rotation, now):
        distance=np.linalg.norm(position-self.position)
        angle=np.linalg.norm(Rotation.from_matrix(rotation@self.rotation.T).as_rotvec())
        if distance>=.0005 or angle>=math.radians(.2):
            self.position=np.array(position,copy=True)
            self.rotation=np.array(rotation,copy=True)
            self.last_motion=now
        return now-self.last_motion<.10
