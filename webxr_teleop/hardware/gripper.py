"""Absolute aperture control; electrical current is a separate read-only limit."""
import math

CURRENT_UNIT_MA = 2.69  # XM430-W350 Goal_Current and Current_Limit.
GRIPPER_SPEED = .18  # normalized aperture / second
GRIPPER_ACCELERATION = .6  # normalized aperture / second squared


def gripper_speed(profile):
    value=profile.get('gripper_speed',GRIPPER_SPEED)
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise ValueError('gripper_speed must be a finite positive aperture/second value')
    return float(value)


def gripper_acceleration(profile):
    # Default rise time stays 0.3 s as the requested speed changes.
    value=profile.get('gripper_acceleration',gripper_speed(profile)/.3)
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise ValueError('gripper_acceleration must be a finite positive aperture/second squared value')
    return float(value)


def aperture_from_trigger(trigger):
    if isinstance(trigger,bool) or not isinstance(trigger,(int,float)) or not math.isfinite(trigger):
        raise ValueError('Gripper requires a finite tracked index-trigger value')
    return 1.-min(1.,max(0.,trigger))


def validate_calibration(calibration):
    try:
        i=calibration['motor_names'].index('gripper')
        start,end=calibration['start_pos'][i],calibration['end_pos'][i]
        mode=calibration['calib_mode'][i]
    except (KeyError,ValueError,IndexError,TypeError,AttributeError) as exc:
        raise ValueError('Gripper requires calibrated LINEAR aperture endpoints') from exc
    if mode!='LINEAR' or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (start,end)) or start==end:
        raise ValueError('Gripper requires distinct finite LINEAR aperture endpoints')
    return dict(closed_raw=start,open_raw=end)


def current_cap(registers, maximum=None, baseline=None):
    """Validate actual readbacks, never manufacture or write an electrical cap."""
    try:mode,goal,limit=(registers[k][-1] for k in ('Operating_Mode','Goal_Current','Current_Limit'))
    except (KeyError,IndexError,TypeError) as exc:
        raise ValueError('Gripper current cap unavailable: missing register readbacks') from exc
    if (mode!=5 or any(isinstance(v,bool) or not isinstance(v,int) for v in (goal,limit))
            or not 0<goal<=limit<=1193):
        raise ValueError(f'Gripper current cap invalid: require mode 5 and 0 < Goal_Current <= Current_Limit <= 1193; got {mode}, {goal}, {limit}')
    if maximum is not None and (isinstance(maximum,bool) or not isinstance(maximum,int) or not 0<maximum<=1193 or max(goal,limit)>maximum):
        raise ValueError(f'Gripper current cap exceeds configured verified ceiling {maximum} counts: Goal_Current={goal}, Current_Limit={limit}')
    if baseline is not None and (goal!=baseline['goal_current_counts'] or limit!=baseline['current_limit_counts']):
        raise ValueError('Gripper current cap readback changed during session; restart after resolving the change')
    return dict(operating_mode=mode,goal_current_counts=goal,current_limit_counts=limit,
                goal_current_ma=goal*CURRENT_UNIT_MA,current_limit_ma=limit*CURRENT_UNIT_MA,
                readback_verified=True)


def shape_aperture(position,velocity,target,dt,*,speed_limit=GRIPPER_SPEED,acceleration_limit=GRIPPER_ACCELERATION):
    """Bound aperture speed/acceleration, with conservative discrete braking."""
    delta=target-position
    speed=min(speed_limit,max(0.,math.sqrt((acceleration_limit*dt)**2+2*acceleration_limit*abs(delta))-acceleration_limit*dt))
    wanted=math.copysign(speed,delta) if delta else 0.
    step=acceleration_limit*dt
    velocity+=max(-step,min(step,wanted-velocity))
    result=position+velocity*dt
    if not -1e-10<=result<=1+1e-10:
        raise ValueError('Gripper aperture shaper exceeded calibrated bounds')
    return min(1.,max(0.,result)),velocity


def feedback_aperture(raw,calibration):
    """Calibration endpoints bound commands, not exact encoder hard stops.

    Two encoder counts are tolerated as endpoint quantization/settling. Larger
    finite excursions up to 10% of travel are observable but require latched hold;
    they are never accepted as valid teleop input or expanded command limits.
    """
    closed,opened=calibration['closed_raw'],calibration['open_raw']
    span=abs(opened-closed)
    normalized=(float(raw)-closed)/(opened-closed)
    outside=max(0.,-normalized,normalized-1.)*span
    info=dict(raw_count=int(raw),closed_raw=closed,open_raw=opened,
              unclamped_aperture=normalized,outside_counts=outside,
              endpoint_tolerance_counts=2)
    if not math.isfinite(normalized) or outside>max(2.,.10*span)+1e-8:
        raise ValueError(f'Gripper feedback implausibly outside calibration: raw={raw}, closed={closed}, open={opened}, aperture={normalized:.6f}; inspect calibration/encoder')
    info['state']='out_of_range' if outside>2.+1e-8 else 'endpoint_tolerance' if outside>1e-8 else 'in_range'
    return min(1.,max(0.,normalized)),info
