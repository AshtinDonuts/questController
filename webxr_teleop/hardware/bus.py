"""LeRobot low-level adapter. Torque changes are explicit startup or completed sleep only; no mode, EEPROM or reboot writes."""
import fcntl
import json
import math
import os
import time
from .core import JOINTS, MOTORS
from .gripper import current_cap, validate_calibration, gripper_speed, gripper_acceleration, feedback_aperture
from .motion import motor_profile_counts, VELOCITY_UNIT_DEG_S, ACCELERATION_UNIT_DEG_S2


class MockBus:
    def __init__(self, profile, side, *, startup=False):
        a = profile['arms'][side]
        self.q = a['initial_rad'][:]+[a['initial_gripper']]
        self.goal = self.q[:]
        self.writes = []
        self.torque = not startup
        self.enable_calls = 0
        self.startup_allowed = startup
        self.startup_attempted = False
        self.startup_report = None

    def read(self, *, torque_off=False):
        if self.torque == torque_off:
            raise RuntimeError('Startup verification required')
        self.q = self.goal[:]
        return self.q[:], dict(torque=self.torque, errors=0, mock=True)

    def disable_torque(self):
        self.torque_release_attempted = True
        self.torque = False

    def write(self, q):
        if getattr(self,'torque_release_attempted',False):raise RuntimeError('Sleep torque is disabled')
        self.goal = q[:]
        self.writes.append(q[:])

    def startup_hold(self, cancelled=lambda: False, *, ignore_model_limits=False):
        if not self.startup_allowed or self.startup_attempted:
            raise RuntimeError('Startup must be explicitly requested once per connection')
        self.startup_attempted = True
        if cancelled():
            raise RuntimeError('Startup cancelled before torque enable')
        self.goal = self.q[:]
        self.configure_motion_caps()
        if not self.torque:
            self.enable_calls += 1
            self.torque = True
        self.startup_report = dict(verified=True, torque_enabled_once=self.enable_calls == 1, mock=True)
        return self.read()

    def configure_motion_caps(self):
        pass

    def opening_speed(self, enabled, counts=24, acceleration_counts=None):
        pass

    def teleop_speed(self, enabled, speed=18., acceleration=60.):
        return dict(mock=True,counts=motor_profile_counts(speed,acceleration))

    def watchdog(self):
        pass

    def close(self):
        pass


class LeRobotBus:
    def __init__(self, profile, side, *, startup=False):
        import numpy as np
        import dynamixel_sdk as dxl
        from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
        self.np, self.dxl = np, dxl
        self.cfg = profile['arms'][side]
        self.gripper_speed=gripper_speed(profile)
        self.gripper_acceleration=gripper_acceleration(profile)
        self.startup_allowed = startup
        self.startup_attempted = False
        self.startup_report = None
        self.command_names = list(JOINTS)+['gripper']
        motors = {n: (i+1, 'xm540-w270' if i < 7 else 'xm430-w350') for i,n in enumerate(MOTORS)}
        self.bus = DynamixelMotorsBus(port=self.cfg['port'], motors=motors)
        self.lock = None
        self.raw = None
        self.last_q = None
        try:
            # Resolve aliases so competing processes cannot lock the same port under different names.
            import hashlib
            key = hashlib.sha256(os.path.realpath(self.cfg['port']).encode()).hexdigest()[:16]
            self.lock = open(f'/tmp/quest-aloha-{os.getuid()}-{key}.lock', 'a')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.bus.connect()
            # Kernel exclusive serial access also excludes other driver implementations.
            import termios
            fcntl.ioctl(self.bus.port_handler.ser.fileno(), termios.TIOCEXCL)
            if not self.bus.port_handler.setBaudRate(1_000_000):
                raise RuntimeError('Cannot set serial baud rate')
            ph = self.bus.port_handler
            ph.ser.write_timeout = .020
            ph.setPacketTimeout = lambda length: ph.setPacketTimeoutMillis(20)
            calibration = json.loads(open(self.cfg['calibration']).read())
            if calibration['motor_names'] != list(MOTORS):
                raise ValueError('Calibration motor order mismatch')
            self.gripper_calibration=validate_calibration(calibration)
            self.bus.set_calibration(calibration)
            self.preflight = self.inspect()
        except BaseException:
            self.close()
            raise

    def register(self, name, names=None):
        names = list(names or MOTORS)
        addr, size = self.bus.model_ctrl_table['xm540-w270'][name]
        reader = self.dxl.GroupSyncRead(self.bus.port_handler, self.bus.packet_handler, addr, size)
        for n in names:
            reader.addParam(self.bus.motors[n][0])
        if reader.txRxPacket() != self.dxl.COMM_SUCCESS:
            raise ConnectionError(f'Read failed: {name}')
        result = []
        for n in names:
            idx = self.bus.motors[n][0]
            if not reader.isAvailable(idx, addr, size):
                raise ConnectionError(f'Missing feedback: {n}/{name}')
            result.append(reader.getData(idx, addr, size))
        return result

    def inspect(self):
        r = {n:self.register(n) for n in ('Model_Number','Firmware_Version','Torque_Enable','Operating_Mode',
             'Drive_Mode','Secondary_ID','Hardware_Error_Status','Bus_Watchdog','Goal_Current','Current_Limit','Present_Velocity','Profile_Velocity','Profile_Acceleration','Homing_Offset')}
        if r['Model_Number'] != [1120]*7+[1020]*2:
            raise ValueError('Unexpected motor identities; expected XM540-W270 and XM430-W350')
        if min(r['Firmware_Version']) < 38:
            raise ValueError('Firmware lacks required bus watchdog')
        allowed = ([0]*9, [1]*9) if self.startup_allowed else ([1]*9,)
        if r['Torque_Enable'] not in allowed or any(r['Hardware_Error_Status']):
            raise ValueError('Expected uniform torque state and no motor errors; torque-off requires explicit startup')
        if any(v & 8 for v in r['Drive_Mode']):
            raise ValueError('Automatic torque-on on goal update must be disabled')
        if any(v & 4 for v in r['Drive_Mode']):
            raise ValueError('Velocity-based profiles required; time-based drive mode is unsupported')
        if r['Operating_Mode'] != [4]*8+[5]:
            raise ValueError('Expected extended position arm joints and current-based position gripper')
        if r['Secondary_ID'][2] != 2 or r['Secondary_ID'][4] != 4:
            raise ValueError('Shoulder/elbow shadow IDs are not configured')
        if any(r['Secondary_ID'][i] != 255 for i in (0,1,3,5,6,7,8)):
            raise ValueError('Unexpected secondary IDs')
        reuse_stationary = (getattr(self,'reuse_stationary_watchdog',False)
                            and r['Torque_Enable']==[0]*9 and r['Bus_Watchdog']==[15]*9)
        if any(v != 0 for v in r['Bus_Watchdog']) and not reuse_stationary:
            raise ValueError('Existing watchdog configuration/fault: explicit supported-arm recovery required')
        cap=current_cap(r,self.cfg.get('gripper_current_cap_counts'),getattr(self,'_gripper_current_cap',None))
        velocity=self.np.asarray(r['Present_Velocity'],dtype=self.np.uint32).view(self.np.int32)
        if any(abs(v) > 2 for v in velocity):
            raise ValueError('Attach only to stationary arms')
        self._gripper_current_cap=cap
        r['gripper_current_cap']=cap
        # No writes here. Observe mode remains genuinely read-only.
        return r

    def decode(self, raw):
        # Explicitly avoid LeRobot's runtime calibration-autocorrection path.
        values = self.bus.apply_calibration(raw.copy(), list(MOTORS))
        if abs(values[1]-values[2]) > 2 or abs(values[3]-values[4]) > 2:
            raise ValueError('Coupled motor feedback differs by more than 2 degrees')
        q = []
        for i,n in enumerate(JOINTS):
            v = values[list(MOTORS).index(n)]
            q.append(math.radians((float(v)-self.cfg['offset_deg'][i])/self.cfg['sign'][i]))
        calibration=getattr(self,'gripper_calibration',None) or validate_calibration(self.bus.calibration)
        aperture,self.gripper_feedback=feedback_aperture(raw[-1],calibration)
        q.append(aperture)
        return q, values

    def read_raw(self):
        return self.np.asarray(self.register('Present_Position'), dtype=self.np.uint32).view(self.np.int32)

    def read(self, *, torque_off=False):
        raw = self.read_raw()
        q, values = self.decode(raw)
        torque = self.register('Torque_Enable')
        errors = self.register('Hardware_Error_Status')
        watchdog = self.register('Bus_Watchdog')
        if torque != ([0]*9 if torque_off else [1]*9) or any(errors) or any(x == 255 for x in watchdog):
            raise RuntimeError('Motor torque/error/watchdog fault; no automatic recovery')
        cap=self.verify_gripper_current()
        self.raw, self.last_q = raw, q[:]
        return q, dict(torque=not torque_off, errors=0, watchdog=watchdog,gripper_current_cap=cap,gripper_feedback=getattr(self,'gripper_feedback',None),gripper_motion_profile=getattr(self,'gripper_motion_profile',None),raw_position=raw.tolist(),calibrated_position=values.tolist())

    def verify_gripper_current(self):
        baseline=getattr(self,'_gripper_current_cap',None)
        if baseline is None:
            raise ValueError('Live gripper control rejected: no verified electrical current cap')
        registers={key:self.register(key,['gripper']) for key in ('Operating_Mode','Goal_Current','Current_Limit')}
        return current_cap(registers,self.cfg.get('gripper_current_cap_counts'),baseline)

    def disable_torque(self):
        # Include primary, shadow and gripper motors; verify every register.
        self.torque_release_attempted = True
        self.bus.write('Torque_Enable', 0)
        if self.register('Torque_Enable') != [0]*len(MOTORS):
            raise ConnectionError('Sleep torque-off readback mismatch')

    def bounded_gripper_raw(self,raw,*,startup=False):
        calibration=getattr(self,'gripper_calibration',None)
        if calibration is None and getattr(self.bus,'calibration',None):calibration=validate_calibration(self.bus.calibration)
        if calibration is None:return int(raw)  # Offline test adapters may omit calibration.
        _,info=feedback_aperture(raw,calibration)
        if startup and info['state']=='out_of_range':
            raise ValueError('Startup raw hold rejected: gripper outside endpoint tolerance')
        lo,hi=sorted((calibration['closed_raw'],calibration['open_raw']))
        return int(min(hi,max(lo,raw)))

    def write_raw_hold(self, raw):
        """Preload current counts with shared primary/shadow goals (<=2 ticks apart)."""
        raw = raw.copy()
        raw[-1]=self.bounded_gripper_raw(raw[-1],startup=True)
        for primary, shadow in ((1,2),(3,4)):
            if abs(int(raw[primary])-int(raw[shadow]))>2:
                raise ValueError('Raw primary/shadow positions differ by more than 2 ticks')
            raw[shadow]=raw[primary]
        writer = self.dxl.GroupSyncWrite(self.bus.port_handler,self.bus.packet_handler,116,4)
        for name,value in zip(MOTORS,raw):
            writer.addParam(self.bus.motors[name][0],list((int(value)&0xffffffff).to_bytes(4,'little')))
        if writer.txPacket()!=self.dxl.COMM_SUCCESS:
            raise ConnectionError('Startup raw hold write failed')
        actual=self.np.asarray(self.register('Goal_Position'),dtype=self.np.uint32).view(self.np.int32)
        if self.np.max(self.np.abs(actual.astype('int64')-raw.astype('int64')))>2:
            raise ConnectionError('Startup raw hold readback differs by more than 2 ticks')

    def startup_hold(self, cancelled=lambda: False, *, ignore_model_limits=False):
        """Enable one follower once, at its current pose. Never retries torque-on.

        Modes 4/5 are required: mode 3 resets its encoder on enable and is refused.
        Any unexpected post-enable discontinuity is held in raw coordinates and
        latched as an error; calibration is never silently shifted to absorb it.
        """
        if not self.startup_allowed or self.startup_attempted:
            raise RuntimeError('Startup must be explicitly requested once per connection')
        self.startup_attempted=True
        enabled=False
        safe_to_hold=False
        try:
            self.preflight=self.inspect()
            before=self.read_raw()
            q,_=self.decode(before)
            if (getattr(self,'gripper_feedback',None) or {}).get('state')=='out_of_range':
                raise ValueError('Startup rejected: gripper encoder outside calibrated range; resolve before enabling control')
            if not ignore_model_limits and any(not lo<=v<=hi for v,lo,hi in zip(q[:6],self.cfg['lower_rad'],self.cfg['upper_rad'])):
                raise ValueError('Startup pose outside configured joint limits')
            safe_to_hold=True
            self.write_raw_hold(before)
            self.configure_motion_caps()
            # Watchdog is armed before torque-on, covering a process/USB failure in that window.
            self.watchdog()
            current=self.read_raw()
            if self.np.max(self.np.abs(current.astype('int64')-before))>2:
                raise RuntimeError('Arm moved during startup; support it stationary and retry explicitly')
            self.write_raw_hold(current)
            if cancelled():
                raise RuntimeError('Startup cancelled before torque enable')
            if self.preflight['Torque_Enable']==[0]*9:
                # Torque-on is only permitted here. No torque-off cleanup.
                enabled=True  # A communication error may still have enabled some motors.
                self.bus.write('Torque_Enable',1)
            after=self.read_raw()
            self.write_raw_hold(after)
            delta=after.astype('int64')-current
            if self.np.max(self.np.abs(delta))>6:
                raise RuntimeError('Encoder reset or unexpected motion on torque enable; raw hold latched')
            if (self.register('Operating_Mode')!=self.preflight['Operating_Mode']
                    or self.register('Homing_Offset')!=self.preflight['Homing_Offset']):
                raise RuntimeError('Motor coordinate settings changed during startup')
            fixed=after.copy()
            deadline=time.monotonic()+.3
            while True:
                q,health=self.read()
                if self.np.max(self.np.abs(self.raw.astype('int64')-fixed))>6:
                    raise RuntimeError('Startup hold did not settle; motion remains disabled')
                if cancelled():
                    raise RuntimeError('Startup cancelled; leaving torque enabled in hold')
                if time.monotonic()>=deadline:
                    break
                time.sleep(.02)
            self.startup_report=dict(verified=True,torque_enabled_once=enabled,
                raw_before=current.tolist(),raw_after=after.tolist(),encoder_delta=delta.tolist())
            return q,health
        except BaseException:
            # Post-enable errors may occur before calibrated feedback is usable.
            # Freeze actual raw counts once; never retry enable or disable torque.
            try:
                if safe_to_hold:
                    self.write_raw_hold(self.read_raw())
            except Exception:
                pass
            self.startup_report=dict(verified=False,torque_enable_attempted=enabled,hold_confirmed=False)
            raise

    def raw_target(self, q):
        # Holding the last read pose uses exact encoder ticks, avoiding float roundoff.
        if q == self.last_q:
            raw=[int(self.raw[list(MOTORS).index(n)]) for n in self.command_names]
            raw[-1]=self.bounded_gripper_raw(raw[-1])
            return raw
        values = self.np.array([math.degrees(x)*s+o for x,s,o in zip(q[:6],self.cfg['sign'],self.cfg['offset_deg'])]+[q[6]*100])
        return self.bus.revert_calibration(values, self.command_names).tolist()

    def write(self, q):
        if getattr(self, 'torque_release_attempted', False):
            raise RuntimeError('Sleep torque is disabled')
        if getattr(self,'_gripper_current_cap',None) is None:
            raise ValueError('Live gripper control rejected: no verified electrical current cap')
        raw = self.raw_target(q)
        if self.np.array_equal(raw,getattr(self,'_last_written_raw',None)):
            return False  # Health reads continue; unchanged goals need no new motor profile.
        # Shadow IDs follow primary commands. Verify they would receive compatible raw goals.
        vals = self.np.array([math.degrees(q[i])*self.cfg['sign'][i]+self.cfg['offset_deg'][i] for i in (1,2)])
        shadows = self.bus.revert_calibration(vals, ['shoulder_shadow','elbow_shadow'])
        if any(abs(int(a)-b) > 2 for a,b in zip(shadows, (raw[1],raw[2]))):
            raise ValueError('Calibration conflicts with primary/shadow raw target coupling')
        writer = self.dxl.GroupSyncWrite(self.bus.port_handler, self.bus.packet_handler, 116, 4)
        for name,value in zip(self.command_names,raw):
            writer.addParam(self.bus.motors[name][0], list((int(value)&0xffffffff).to_bytes(4,'little')))
        if writer.txPacket() != self.dxl.COMM_SUCCESS:
            raise ConnectionError('Goal-position write failed')
        # Sync writes have no per-motor acknowledgement. Confirm stored goals explicitly.
        actual = self.np.asarray(self.register('Goal_Position', self.command_names),dtype=self.np.uint32).view(self.np.int32)
        if not self.np.array_equal(actual, raw):
            raise ConnectionError('Goal-position readback mismatch')
        self._last_written_raw=raw.copy()
        return True

    def opening_speed(self, enabled, counts=24, acceleration_counts=None):
        """Temporary opening profiles; restore exact prior RAM values on exit."""
        saved=getattr(self,'_opening_profiles',None)
        if enabled:
            if saved is not None:return
            saved={field:self.register(field) for field in ('Profile_Velocity','Profile_Acceleration')}
            self._opening_profiles=saved
        elif saved is None:return
        # Default ceremony needs 90 deg/s² for 3x trajectory time scaling.
        acceleration_counts=5 if acceleration_counts is None and counts==24 else acceleration_counts
        for field,address in [('Profile_Velocity',112),('Profile_Acceleration',108)]:
            desired=saved[field][:]
            value=counts if field=='Profile_Velocity' else acceleration_counts
            if enabled and value is not None:desired[:8]=[value]*8
            for name,value in zip(MOTORS,desired):
                comm,error=self.bus.packet_handler.write4ByteTxRx(
                    self.bus.port_handler,self.bus.motors[name][0],address,value)
                if comm!=self.dxl.COMM_SUCCESS or error:
                    raise ConnectionError(f'Opening {field} write failed')
            if self.register(field)!=desired:
                raise ConnectionError(f'Opening {field} readback failed')
        if not enabled:self._opening_profiles=None

    def teleop_speed(self, enabled, speed=18., acceleration=60.):
        if not enabled:
            self.opening_speed(False);return None
        drive=self.register('Drive_Mode')
        if any(v & 12 for v in drive[:8]):
            raise ValueError('Teleop requires velocity-based profiles and no automatic torque-on')
        velocity_counts,acceleration_counts=motor_profile_counts(speed,acceleration)
        self.opening_speed(True,counts=velocity_counts,acceleration_counts=acceleration_counts)
        return dict(velocity_counts=velocity_counts,acceleration_counts=acceleration_counts,
                    velocity_deg_s=velocity_counts*VELOCITY_UNIT_DEG_S,
                    acceleration_deg_s2=acceleration_counts*ACCELERATION_UNIT_DEG_S2,
                    drive_mode=drive,readback_verified=True)

    def configure_motion_caps(self):
        # RAM profiles only. Explicit arm tuning overrides prior arm caps;
        # absent arm tuning preserves the legacy arm min-cap policy.
        # Gripper profile follows the requested aperture speed with headroom.
        # XM-series velocity: .229 rpm/count; acceleration: 214.577 rpm/min/count.
        c=self.bus.calibration
        i=c['motor_names'].index('gripper')
        gripper_degrees=abs(c['end_pos'][i]-c['start_pos'][i])*360/4096
        velocities=[10.]*8+[getattr(self,'gripper_speed',gripper_speed({}))*gripper_degrees]
        caps=[int(v/1.374) for v in velocities]
        gripper_cap=math.ceil(1.5*velocities[-1]/VELOCITY_UNIT_DEG_S)
        if not 1<=gripper_cap<=32767:
            raise ValueError('Requested gripper speed exceeds representable motor profile range')
        acceleration=getattr(self,'gripper_acceleration',gripper_acceleration({'gripper_speed':getattr(self,'gripper_speed',gripper_speed({}))}))
        gripper_acceleration_cap=math.ceil(2*acceleration*gripper_degrees/ACCELERATION_UNIT_DEG_S2)
        if not 1<=gripper_acceleration_cap<=32767:
            raise ValueError('Requested gripper acceleration exceeds representable motor profile range')
        caps[-1]=gripper_cap
        if min(caps)<1:
            raise ValueError('Gripper calibration cannot support a representable safe speed cap')
        tuning=self.cfg.get('motor_tracking_profile')
        if tuning is not None:
            for key in ('velocity_counts','acceleration_counts'):
                value=tuning.get(key)
                if isinstance(value,bool) or not isinstance(value,int) or not 1<=value<=32767:
                    raise ValueError(f'Invalid motor tracking profile {key}')
            if any(value & 4 for value in self.preflight['Drive_Mode'][:8]):
                raise ValueError('Tracking profile requires velocity-based Drive Mode')
        for field,address,limits in [('Profile_Velocity',112,caps),('Profile_Acceleration',108,[1]*9)]:
            previous=self.preflight[field]
            desired=[min(old,cap) if old else cap for old,cap in zip(previous,limits)]
            desired[-1]=gripper_cap if field=='Profile_Velocity' else gripper_acceleration_cap
            if tuning is not None:
                desired[:8]=[tuning['velocity_counts' if field=='Profile_Velocity' else 'acceleration_counts']]*8
            for n,value in zip(MOTORS,desired):
                comm,error=self.bus.packet_handler.write4ByteTxRx(self.bus.port_handler,self.bus.motors[n][0],address,value)
                if comm!=self.dxl.COMM_SUCCESS or error:
                    raise ConnectionError(f'Could not apply {field}')
            if self.register(field)!=desired:
                raise ConnectionError(f'{field} readback mismatch')
        self.gripper_motion_profile=dict(speed_aperture_s=getattr(self,'gripper_speed',gripper_speed({})),
            acceleration_aperture_s2=acceleration,velocity_counts=gripper_cap,acceleration_counts=gripper_acceleration_cap,readback_verified=True)

    def watchdog(self):
        # RAM watchdog only; never clear a fault automatically.
        for n in MOTORS:
            idx = self.bus.motors[n][0]
            comm, error = self.bus.packet_handler.write1ByteTxRx(self.bus.port_handler,idx,98,15)
            if comm != self.dxl.COMM_SUCCESS or error:
                raise ConnectionError('Could not enable bus watchdog')
        if self.register('Bus_Watchdog') != [15]*9:
            raise ConnectionError('Bus watchdog readback mismatch')

    def close(self):
        if getattr(self.bus,'is_connected',False):
            self.bus.disconnect()  # Low-level closePort only; no torque writes.
        if self.lock is not None:
            self.lock.close()
            self.lock = None


class SoftwareHoldBus(LeRobotBus):
    """Explicit software-watchdog mode; motor timeout is not used or certified."""
    reuse_stationary_watchdog = True

    def watchdog(self):
        # This explicit mode replaces an existing non-faulted motor watchdog
        # only while torque is off. Never clear 255 or cycle torque.
        current=self.register('Bus_Watchdog')
        if current==[0]*9:return
        if current!=[15]*9 or self.register('Torque_Enable')!=[0]*9:
            raise RuntimeError('Software hold requires an unlatched watchdog and torque off to change its configuration')
        for n in MOTORS:
            comm,error=self.bus.packet_handler.write1ByteTxRx(self.bus.port_handler,self.bus.motors[n][0],98,0)
            if comm!=self.dxl.COMM_SUCCESS or error:raise ConnectionError('Could not select software hold mode')
        if self.register('Bus_Watchdog')!=[0]*9:raise ConnectionError('Software hold configuration readback failed')
