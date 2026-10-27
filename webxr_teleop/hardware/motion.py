"""Dependency-free motion settings and latest-command selection."""
import math

VELOCITY_UNIT_DEG_S = .229 * 6
ACCELERATION_UNIT_DEG_S2 = 214.577 / 10


def motion_settings(profile):
    values=[]
    # Initial limits are below the existing opening envelope (30 deg/s, 90 deg/s²).
    for key,default,maximum in [('teleop_joint_speed_deg_s',18.,60.),
                                 ('teleop_joint_acceleration_deg_s2',60.,180.)]:
        value=profile.get(key,default)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=maximum:
            raise ValueError(f'{key} must be finite and in (0, {maximum}]')
        values.append(float(value))
    return tuple(values)


def motor_profile_counts(speed, acceleration):
    return (math.ceil(1.5*speed/VELOCITY_UNIT_DEG_S),
            math.ceil(2*acceleration/ACCELERATION_UNIT_DEG_S2))


def latest_motion_messages(messages):
    """Keep control operations, but never replay intermediate position targets."""
    commands=[m for m in messages if m.get('op')=='command']
    controls=[m for m in messages if m.get('op')!='command']
    if commands:
        controls.append(max(commands,key=lambda m:(m.get('epoch',-1),m.get('seq',-1))))
    return controls


def controller_source_fresh(message, now, ttl):
    """Check the original challenge timestamp, never the planner/receipt time."""
    if message.get('mode')!='teleop' or (message.get('freeze') and not message.get('gripper_active')):
        return True
    source=message.get('controller_t')
    return (isinstance(source,(int,float)) and not isinstance(source,bool)
            and math.isfinite(source) and 0<=now-source<=ttl)


def expired_planner_sources(result, sides, now, ttl):
    """Report stale active sources before admitting a multi-arm planner result."""
    expired={}
    for side in sides:
        source=result.get('controller_t',{}).get(side)
        if not controller_source_fresh(dict(mode=result['mode'],controller_t=source,
                freeze=result['freeze'][side],gripper_active=result.get('gripper_active',False)),now,ttl):
            expired[side]=(1000*(now-source) if isinstance(source,(int,float))
                and not isinstance(source,bool) and math.isfinite(source) else None)
    return expired


def matching_grip_sides(result, sides, grip_generation):
    """One hand's clutch transition cannot suppress the other arm's result.

    Source freshness is still checked for the full result before forwarding.
    A mismatched side gets its direct clutch hold and waits for a new plan.
    """
    if result['mode']!='teleop':return tuple(sides)
    planned=result.get('grip_generation',{})
    return tuple(s for s in sides if planned.get(s)==grip_generation[s])


class ClutchBarrier:
    """Reject delayed commands from a motion that ended at grip release."""
    def __init__(self):
        self.generation=0
        self.released_at=-math.inf

    def release(self,message):
        self.generation=max(self.generation,message.get('grip_generation',self.generation))
        self.released_at=max(self.released_at,message['issued'])

    def accepts(self,message):
        if message.get('mode')!='teleop':return True
        generation=message.get('grip_generation',0)
        if generation<self.generation or message.get('input_t',message['issued'])<self.released_at:return False
        self.generation=generation
        return True
