import numpy as np
import time
import os
import yaml
from utils_OV.guidance_utils import TrajectoryGeneratorV2

import logging



logger = logging.getLogger(__name__)


def _should_emit_every(last_emit_time: float, interval_s: float, *, now: float | None = None) -> tuple[bool, float]:
    """Return (should_emit, new_last_emit_time) for simple time-based throttling.

    Keeping this helper local avoids repeating `if last + interval < time.time()` blocks.
    """
    if now is None:
        now = time.time()
    if now - last_emit_time >= interval_s:
        return True, now
    return False, last_emit_time


class PhaseLogger:
    """Consistent, structured logging for flight phases.

    - Prefixes every message with `[PHASE]`.
    - Optionally throttles high-rate messages per (phase, key).

    Usage:
        self._phase_logger.log("CLIMB", logging.INFO, "msg %s", x)
        self._phase_logger.log_every("CLIMB", "progress", 2.0, logging.INFO, "alt=%.2f", alt)
    """

    def __init__(self, logger_obj: logging.Logger):
        self._logger = logger_obj
        self._last_by_key: dict[tuple[str, str], float] = {}

    def log(self, phase: str, level: int, msg: str, *args, **kwargs) -> None:
        self._logger.log(level, f"[{phase}] {msg}", *args, **kwargs)

    def log_every(
        self,
        phase: str,
        key: str,
        interval_s: float,
        level: int,
        msg: str,
        *args,
        **kwargs,
    ) -> None:
        now = time.time()
        last = self._last_by_key.get((phase, key), now - interval_s - 1e-6)
        should_emit, new_last = _should_emit_every(last, interval_s, now=now)
        if not should_emit:
            return
        self._last_by_key[(phase, key)] = new_last
        self.log(phase, level, msg, *args, **kwargs)

    # Phase-specific convenience wrappers (keeps call-sites consistent)
    def takeoff_started(self, *, pf: bool = False) -> None:
        subj = "PF/VIO" if pf else "VIO"
        self.log("TAKEOFF", logging.INFO, "applying initial thrust until %s initializes", subj)

    def takeoff_complete(self, target_alt: float, *, pf: bool = False) -> None:
        subj = "PF/VIO" if pf else "VIO"
        self.log(
            "TAKEOFF",
            logging.INFO,
            "Takeoff complete; %s is initialized. Transitioning to CLIMB (target_alt=%.2f)",
            subj,
            target_alt,
        )

    def climb_progress(self, interval_s: float, target_alt: float, current_alt: float, error: float, *, low_alt_boost: bool = False) -> None:
        suffix = " (low-altitude boost)" if low_alt_boost else ""
        self.log_every(
            "CLIMB",
            "progress",
            interval_s,
            logging.INFO,
            f"target_alt=%.2f current_alt=%.2f error=%.2f{suffix}",
            target_alt,
            current_alt,
            error,
        )

    def climb_complete(self, current_alt: float, target_alt: float) -> None:
        self.log(
            "CLIMB",
            logging.INFO,
            "complete: reached target altitude (alt=%.2f, target=%.2f)",
            current_alt,
            target_alt,
        )

    def descend_progress(self, interval_s: float, target_alt: float, current_alt: float, error: float) -> None:
        self.log_every(
            "DESCEND",
            "progress",
            interval_s,
            logging.INFO,
            "target_alt=%.2f current_alt=%.2f error=%.2f",
            target_alt,
            current_alt,
            error,
        )

    def descend_complete(self, current_alt: float, target_alt: float) -> None:
        self.log(
            "DESCEND",
            logging.INFO,
            "complete: reached target altitude (alt=%.2f, target=%.2f)",
            current_alt,
            target_alt,
        )

    def landing_progress(self, interval_s: float, current_alt: float, error: float) -> None:
        self.log_every(
            "LANDING",
            "progress",
            interval_s,
            logging.INFO,
            "current_alt=%.2f error=%.2f",
            current_alt,
            error,
        )

    def landing_complete(self, current_alt: float) -> None:
        self.log(
            "LANDING",
            logging.INFO,
            "complete: touchdown detected (alt=%.2f)",
            current_alt,
        )


def from_pos_vel_to_angle_ref(a_n, a_e, a_d, chaser_yaw, yaw_in_degrees=False, max_accel = 9.81) -> list:

    if yaw_in_degrees:
        chaser_yaw = chaser_yaw * np.radians(1)
    
    accel_in_body =  np.array([a_n * np.cos(chaser_yaw) + a_e * np.sin(chaser_yaw),
                              -a_n * np.sin(chaser_yaw) + a_e * np.cos(chaser_yaw)])
    # accel_in_body = np.clip(accel_in_body, -max_accel, max_accel)
    # print("accel in body:", accel_in_body)
    pitch_target = np.arctan(-accel_in_body[0]/9.81) 
    roll_target = np.arctan(accel_in_body[1]*np.cos(pitch_target)/9.81)
    return [np.degrees(pitch_target), np.degrees(roll_target)]


class PIDController:
    def __init__(self, kp,  ki, kd, alpha_deriv_filt, dt, max_acc=15.0, log=True, log_file_name="PID.txt") -> None:
        # PID controller
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha_deriv_filt = alpha_deriv_filt
        self.dt = dt
        self.max_acc = max_acc
        self.log = log
        self.log_file_name = log_file_name
        if self.log:
            logdata = np.array([0.0, 0, 0, 0]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, logdata,  delimiter=',')
        self.reset()
        
    def update(self, pos_ref, vel_ref, pos, vel):
        error = pos_ref - pos
        error_dot = (vel_ref - vel)
        self.integral_term += self.ki * (error + self.last_error) * self.dt / 2
        u = self.kp * error + self.integral_term + self.kd * error_dot
        
        if u > self.max_acc:
            u = self.max_acc
            self.integral_term -= self.ki * (error + self.last_error) * self.dt / 2
        if u < -self.max_acc:
            u = -self.max_acc
            self.integral_term -= self.ki * (error + self.last_error) * self.dt / 2
        self.last_error = np.copy(error)
        
        if self.log:
            logdata = np.array([error, error_dot, self.integral_term, u]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "ab") as f:
                np.savetxt(f, logdata,  delimiter=',')
        return u
        
    def reset(self):
        self.integral_term = 0
        self.last_error = 0

class CascadedPIPDController:
    def __init__(self, kp_out, ki_out, kp_in, kd_in, alpha_deriv_filt, dt, max_acc=15.0, log=True, log_file_name="PIPD_controller_log.txt") -> None:
        self.kp_out = kp_out
        self.ki_out = ki_out
        self.kp_in = kp_in
        self.kd_in = kd_in
        self.alpha_deriv_filt = alpha_deriv_filt
        self.dt = dt
        self.max_acc = max_acc
        self.log = log
        self.log_file_name = log_file_name
        if self.log:
            logdata = np.array([0.0, 0, 0, 0, 0, 0]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, logdata,  delimiter=',')
        self.reset()
        
    def update(self, pos_ref, vel_ref, pos, vel):
        error_out = pos_ref - pos
        self.integral_term_out += self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        u_out = self.kp_out * error_out + self.integral_term_out
        error_in =  u_out 
        error_in_dot = self.kp_out * (vel_ref - vel) + self.ki_out * (error_out + self.last_error_out) / 2 # + (vel_ref - vel)
        u = self.kp_in * error_in + self.kd_in * error_in_dot
        if u > self.max_acc:
            u = self.max_acc
            self.integral_term_out -= self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        if u < -self.max_acc:
            u = -self.max_acc
            self.integral_term_out -= self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        self.last_error_out = np.copy(error_out) 
        self.last_integral_term_out = np.copy(self.integral_term_out)
        if self.log:
            logdata = np.array([error_out, self.integral_term_out, u_out, error_in, error_in_dot, u]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "ab") as f:
                np.savetxt(f, logdata,  delimiter=',')
        return u
    
    def reset(self):
        self.integral_term_out = 0
        self.last_integral_term_out = 0
        self.last_error_out = 0


class CascadedPIPDControllerv2:
    def __init__(self, kp_out, ki_out, kp_in, kd_in, alpha_deriv_filt, dt, max_acc=15.0, log=True, log_file_name="PIPD_controller_log.txt") -> None:
        self.kp_out = kp_out
        self.ki_out = ki_out
        self.kp_in = kp_in
        self.kd_in = kd_in
        self.alpha_deriv_filt = alpha_deriv_filt
        self.dt = dt
        self.max_acc = max_acc
        self.integral_max = max_acc
        self.integral_min = -max_acc
        self.log = log
        self.log_file_name = log_file_name
        if self.log:
            logdata = np.array([0.0, 0, 0, 0, 0, 0]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, logdata,  delimiter=',')
        self.reset()
        
    def update(self, pos_ref, vel_ref, pos, vel):
        error_out = pos_ref - pos
        self.integral_term_out += self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        if self.integral_term_out >= self.integral_max:
            self.integral_term_out = self.integral_max
        if self.integral_term_out <= self.integral_min:
            self.integral_term_out = self.integral_min
        u_out = self.kp_out * error_out + self.integral_term_out
        error_in =  u_out 
        error_in_dot = self.kp_out * (vel_ref - vel) #+ self.ki_out * (error_out + self.last_error_out) / 2 # + (vel_ref - vel)
        u = self.kp_in * error_in + self.kd_in * error_in_dot
        # if u > self.max_acc:
        #     u = self.max_acc
        #     self.integral_term_out -= self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        # if u < -self.max_acc:
        #     u = -self.max_acc
        #     self.integral_term_out -= self.ki_out * (error_out + self.last_error_out) * self.dt / 2
        self.last_error_out = np.copy(error_out) 
        self.last_integral_term_out = np.copy(self.integral_term_out)
        if self.log:
            logdata = np.array([error_out, self.integral_term_out, u_out, error_in, error_in_dot, u]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "ab") as f:
                np.savetxt(f, logdata,  delimiter=',')
        return u
    
    def reset(self):
        self.integral_term_out = 0
        self.last_integral_term_out = 0
        self.last_error_out = 0


class PositionController:
    def __init__(self, kp_pos, kp_vel, kd_vel, ki_vel, alpha_deriv_filt, dt, max_acc=15.0, max_vel=10.0, log_file_name="P_PID.txt") -> None:
        # P+PID controller
        self.kp_pos = kp_pos
        self.kp_vel = kp_vel
        self.kd_vel = kd_vel
        self.ki_vel = ki_vel
        self.i_max = 10.0 / (ki_vel + 1e-5) 
        self.alpha_deriv_filt = alpha_deriv_filt
        self.dt = dt
        self.max_acc = max_acc
        self.log_file_name = log_file_name
        if True:
            logdata = np.array([0.0, 0, 0, 0]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, logdata,  delimiter=',')
        self.reset()
    def update(self, pos_ref, vel_ref, pos, vel):
        # pos_error = np.sign(pos_ref - pos) * np.sqrt(abs(pos_ref - pos))
        pos_error = pos_ref - pos
        vel_in = self.kp_pos * pos_error
        vel_in = np.clip(vel_in, -10.0, 10.0)
        vel_error =  vel_in  - vel # 
        # vel_error = self.alpha_deriv_filt * self.last_vel_error + (1 - self.alpha_deriv_filt) * vel_error
        vel_error_dot = (vel_error - self.last_vel_error) / self.dt
        # vel_error_dot = self.alpha_deriv_filt * self.last_vel_error_dot + (1 -  self.alpha_deriv_filt) * vel_error_dot
        
        self.vel_error_sum += (vel_error + self.last_vel_error) * self.dt / 2
        # self.vel_error_sum += ( - vel + self.kp_pos * pos_error) * (self.dt )
        # print("integrator:", self.vel_error_sum)
        
        u_unsaturated = self.kp_vel * vel_error + self.ki_vel * self.vel_error_sum + self.kd_vel * vel_error_dot
        
        ### Clamping the integral term
        if u_unsaturated >= self.max_acc :
            u = self.max_acc
            if self.vel_error_sum > 0:
                self.vel_error_sum -= (vel_error + self.last_vel_error) * self.dt / 2
            
        elif u_unsaturated <= -self.max_acc:
            u = -self.max_acc
            if self.vel_error_sum < 0:
                self.vel_error_sum -= (vel_error + self.last_vel_error) * self.dt / 2
        else:
            u = u_unsaturated
        
        ### Back calculation of the integral term
        # if u_unsaturated >= self.max_acc:
        #     u= self.max_acc
        #     self.vel_error_sum += u - u_unsaturated
        #     # self.vel_error_sum -= (vel_ref - vel + self.kp_pos * pos_error) * (self.dt )
        # elif u_unsaturated <= -self.max_acc:
        #     u = -self.max_acc
        #     self.vel_error_sum += u - u_unsaturated
        #     # self.vel_error_sum -= (vel_ref - vel + self.kp_pos * pos_error) * (self.dt )
        # else:
        #     u = u_unsaturated
        self.last_vel_error = np.copy(vel_error)
        self.last_vel_error_dot = np.copy(vel_error_dot)
        if True:
            logdata = np.array([pos_error, vel_error, self.vel_error_sum, u]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "ab") as f:
                np.savetxt(f, logdata,  delimiter=',')
        return u  
    def reset(self):
        self.vel_error_sum = 0
        self.last_vel_error = 0
        self.last_vel_error_dot = 0
        
class PositionControllerBumpless:
    def __init__(self, kp_pos, kp_vel, kd_vel, ki_vel, alpha_deriv_filt, dt, max_acc=15.0, max_vel=10.0, log_file_name="P_PID.txt") -> None:
        # P+PID controller
        self.kp_pos = kp_pos
        self.kp_vel = kp_vel
        self.kd_vel = kd_vel
        self.ki_vel = ki_vel
        self.i_max = 10.0 / (ki_vel + 1e-5) 
        self.alpha_deriv_filt = alpha_deriv_filt
        self.dt = dt
        self.max_acc = max_acc
        self.max_vel = max_vel
        self.log_file_name = log_file_name
        if True:
            logdata = np.array([0.0, 0, 0, 0]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, logdata,  delimiter=',')
        
        self.reset()
    def update(self, pos_ref, vel_ref, pos, vel):
        # pos_error = np.sign(pos_ref - pos) * np.sqrt(abs(pos_ref - pos))
        pos_error = pos_ref - pos
        vel_in = self.kp_pos * pos_error
        vel_in = np.clip(vel_in + vel_ref, -self.max_vel, self.max_vel)
        vel_error =  vel_in  - vel # 
        
        if not self.initialized:
            self.vel_error_sum = vel_error
            self.last_vel_error = vel_error
            self.initialized = True
        
        # vel_error = self.alpha_deriv_filt * self.last_vel_error + (1 - self.alpha_deriv_filt) * vel_error
        vel_error_dot = (vel_error - self.last_vel_error) / self.dt
        # vel_error_dot = self.alpha_deriv_filt * self.last_vel_error_dot + (1 -  self.alpha_deriv_filt) * vel_error_dot
        
        self.vel_error_sum += (vel_error + self.last_vel_error) * self.dt / 2
        # self.vel_error_sum += ( - vel + self.kp_pos * pos_error) * (self.dt )
        # print("integrator:", self.vel_error_sum)
        
        u_unsaturated = self.kp_vel * vel_error + self.ki_vel * self.vel_error_sum + self.kd_vel * vel_error_dot
        
        
        ### Clamping the integral term
        if u_unsaturated >= self.max_acc :
            u = self.max_acc
            if (self.vel_error_sum > 0 and vel_error > 0) or (self.vel_error_sum < 0 and vel_error < 0):
                self.vel_error_sum -= (vel_error + self.last_vel_error) * self.dt / 2
            
        elif u_unsaturated <= -self.max_acc:
            u = -self.max_acc
            if (self.vel_error_sum < 0 and vel_error < 0) or (self.vel_error_sum > 0 and vel_error > 0):
                self.vel_error_sum -= (vel_error + self.last_vel_error) * self.dt / 2
        else:
            u = u_unsaturated
            
            
        
        ### Back calculation of the integral term
        # if u_unsaturated >= self.max_acc:
        #     u= self.max_acc
        #     self.vel_error_sum += u - u_unsaturated
        #     # self.vel_error_sum -= (vel_ref - vel + self.kp_pos * pos_error) * (self.dt )
        # elif u_unsaturated <= -self.max_acc:
        #     u = -self.max_acc
        #     self.vel_error_sum += u - u_unsaturated
        #     # self.vel_error_sum -= (vel_ref - vel + self.kp_pos * pos_error) * (self.dt )
        # else:
        #     u = u_unsaturated
        self.last_vel_error = np.copy(vel_error)
        self.last_vel_error_dot = np.copy(vel_error_dot)
        if True:
            logdata = np.array([pos_error, vel_error, self.vel_error_sum, u]).reshape(1,-1)  
            logdata = np.insert(logdata, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "ab") as f:
                np.savetxt(f, logdata,  delimiter=',')
        return u  
    def reset(self):
        self.initialized = True
        self.vel_error_sum = 0
        self.last_vel_error = 0
        self.last_vel_error_dot = 0
    def switch(self):
        self.initialized = False
        
        
class ControllerManager:
    def __init__(self, wp_list, alt_target_climb, LOG = True, print = True):
        
        # Guidance and control settings
        with open('/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/OV/config/guidance_and_control_parameters.yaml') as f:
            gc_params = yaml.safe_load(f)

                
        self.max_acc = 5
        self.max_vel = 10
        self.max_acc_climbdescend = 2

        log_file_x = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/OV/logs/x_ref.txt"
        log_file_y = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/OV/logs/y_ref.txt"
        self.pos_controller_x              = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], self.max_acc, self.max_vel, log_file_name =log_file_x)
        self.pos_controller_y              = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], self.max_acc, self.max_vel, log_file_name =log_file_y)

        self.pos_controller_x_climbdescend = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], max_vel=2, max_acc=self.max_acc_climbdescend, log_file_name =log_file_x)
        self.pos_controller_y_climbdescend = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], max_vel=2, max_acc=self.max_acc_climbdescend, log_file_name =log_file_y)

        self.controller_dt = gc_params['gc_dt']
        
        
        # Trajectory generator settings
        self.traj         = TrajectoryGeneratorV2(sampling_freq=1/self.controller_dt, max_vel=[self.max_vel, self.max_vel, self.max_vel], max_acc=[self.max_acc, self.max_acc, self.max_acc])
        
        
        # Waypoint list for tracking
        self.wp_list = self.traj.resample_equal_spacing(wp_list)
        # print(type(self.wp_list))

        # Altitude targets and thresholds
        self.alt_target_climb       = alt_target_climb
        self.alt_thresh_climb_low   = 10.0
        self.alt_target_descend     = 5.0
        self.alt_thresh_descend_low = 10.0
        self.alt_thresh_landing_low = 2.0
        
        # Thrust settings
        self.DEFAULT_TAKEOFF_THRUST = 1
        self.DEFAULT_LANDING_THRUST = 0
        
        
        # Waypoint Navigation Parameters
        self.traj_id = 0
        self.ref_pos = np.array([0.0, 0.0, -0.0])
        self.ref_vel = np.array([0.0, 0.0, 0.0])
        
        
        # State machine flags
        self.TAKEOFF         = True
        self.CLIMB           = False
        self.TRACK           = False
        self.DESCEND         = False
        self.LANDING         = False
        self.DONE            = False
        self.VIO_DIVERGENCE  = False
        self.TAKEOFF_STARTED = False
        self.PROBLEM         = []

        self.EMERGENCY_LAND_TIMEOUT = 60  # seconds for force land if VIO is diverged
        self.home_alt = np.array([0.0])  # home altitude for landing
        # Store VIO position
        self.PF_pos_list   = []
        self.VIO_pos_list = []
        self.GT_pos_list    = []
        self.generated_traj = None
        self.VIO_descend_pos = np.array([0,0,0])
        self.print = print
        self.last_print_time = time.time()
        self._phase_logger = PhaseLogger(logger)
        # How often to emit progress logs for climb/descend/land (seconds)
        self.progress_log_interval_s = 2.0
        self.LOG   = LOG
        

        # Log flight
        if self.LOG:
            self.date_var = os.environ.get("FEATUREMATCH_RUN_TS") or time.strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.join("logs", str(self.date_var))
            os.makedirs(log_dir, exist_ok=True)
            self.log_dir = log_dir
            self.log_file_name = os.path.join(log_dir, "controller_log.txt")

            ref_pos = np.array([0.0,0.0, -0.0])  # Initial reference position
            ref_vel = np.array([0.0, 0.0, 0.0])  # Initial reference velocity
            VIO_vel = np.array([0.0, 0.0, 0.0])  # Initial VIO velocity
            # Set first VIO position and velocity as

            ref_angles = np.array([0.0, 0.0, 0.0])
            acc_cmd_xy = np.array([0.0, 0.0, 0.0])  # Initial acceleration command
            ref_posvel = np.array([ref_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
            ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
            with open(self.log_file_name, "w") as f:
                np.savetxt(f, ref_posvel,  delimiter=',')

    
    def control_UAV(self, node_OdomVIO, node_PixhawkCMD):

        # self.PF_pos_first = node_OdomVIO.VIOned_dict['position'].copy()
        
        t_prev = time.time()
        while True:
            if time.time() - t_prev > self.controller_dt:  # apply control at the specified controller_dt
                t_prev = time.time()


                # Problem detected, abort mission
                if len(self.PROBLEM) > 0:
                    # Handle each problem
                    logger.error("Mission abort: one or more problems detected")
                    for problem in self.PROBLEM:
                        logger.error("Problem: %s", problem)

                    return 
            
                # TAKEOFF phase
                if self.TAKEOFF:
                    self._take_off(node_OdomVIO, node_PixhawkCMD)
                    
                    if node_OdomVIO.initialization_status and self.TAKEOFF_STARTED and node_OdomVIO.first_pf_pose_msg:

                        self.TAKEOFF = False
                        self.CLIMB   = True
                        self._phase_logger.takeoff_complete(self.alt_target_climb, pf=True)
                        
                        # Get initial position from VIO once yaw ref is initialized
                        # while not node_OdomVIO.ned_conversion_initialized:
                        while_timeout = time.time() + 5  # 5 seconds timeout
                        while True:
                            
                            try:
                                # self.PF_pos_first = node_OdomVIO.VIOned_dict['position'].copy()
                                self.PF_pos_first = node_OdomVIO.pf_pos_dict['position'].copy()
                                break
                            except Exception as e:
                                logger.exception("Failed to read initial PF position")
                                time.sleep(0.1)
                                
                            if time.time() > while_timeout:
                                logger.error("Timeout while reading initial PF position; continuing without initialization")
                                break


                # CLIMB phase
                elif self.CLIMB:
                    self._climb(node_OdomVIO, node_PixhawkCMD)
                    
                # TRAJECTORY TRACKING phase
                elif self.TRACK:
                    self._track_trajectory(node_OdomVIO, node_PixhawkCMD) 
                    
                # DESCEND phase
                elif self.DESCEND:
                    self._descend(node_OdomVIO, node_PixhawkCMD)    

                # LANDING phase
                elif self.LANDING:
                    self._land(node_OdomVIO, node_PixhawkCMD)

                # # Terminate
                # else:
                #     print("Mission completed.")
                #     self.DONE = True
                
                    
                # Check mode for interrupting flight or done flag
                if (not (node_OdomVIO.state_dict['mode'] == "GUIDED" or node_OdomVIO.state_dict['mode'] == "GUIDED_NOGPS") or self.DONE) and (self.generated_traj is not None):

                    # log_dir = os.path.join("logs", str(self.date_var), "traj")
                    # os.makedirs(log_dir, exist_ok=True)

                    # np.save(os.path.join(log_dir, 'PF_pos_list.npy'), np.array(self.PF_pos_list))
                    # np.save(os.path.join(log_dir, 'VIO_pos_list.npy'), np.array(self.VIO_pos_list))
                    # np.save(os.path.join(log_dir, 'generated_traj.npy'), np.array(self.generated_traj['pos']))
                    # np.save(os.path.join(log_dir, 'GT_pos_list.npy'), np.array(self.GT_pos_list))

                    log_dir = os.path.join("logs", str(self.date_var))
                    node_OdomVIO.odom_data_logger.save(log_dir)

                    # RESET position controllers
                    self.pos_controller_x.reset()
                    self.pos_controller_y.reset()

                    # Reset traj id
                    self.traj_id = 0

                    # # Reset lists
                    # self.PF_pos_list = []
                    # self.VIO_pos_list = []
                    # self.GT_pos_list  = []

                    # Reset yaw
                    node_OdomVIO._update_yaw_difference()

                    if self.DONE:
                        logger.info("Mission completed successfully")
                        return
                        
                    else:
                        logger.warning(
                            "Mission interrupted: vehicle mode is not GUIDED/GUIDED_NOGPS (mode=%s)",
                            node_OdomVIO.state_dict['mode'],
                        )
                        return
                                        

                # Try to recover from VIO divergence
                if node_OdomVIO.try_recover_maneuver and not node_OdomVIO.vio_divergence_detected:
                    self._divergence_maneuver(node_PixhawkCMD)
                                    
                # Check VIO divergence to terminate mission
                if node_OdomVIO.vio_divergence_detected:
                
                    if not self.VIO_DIVERGENCE:
                    
                        logger.error("VIO divergence detected; entering recovery/emergency-landing logic")
                        self.VIO_DIVERGENCE_START_TIME = time.time() # 60 seconds to land
                        # self.DONE = True
                        self.VIO_DIVERGENCE = True

                    else:
                        self._divergence_maneuver(node_PixhawkCMD)

    def _take_off(self,node_OdomVIO, node_PixhawkCMD):
        
        # send arm message
        while_timeout = time.time() + 5  # 10 seconds to arm
        while not node_OdomVIO.state_dict['armed']:
            # node_PixhawkCMD.arm(True)
            time.sleep(1)

            # if time.time() > while_timeout:
            #     logger.error("Arming timeout; unable to arm the vehicle")
            #     self.PROBLEM.append("Arming timeout, unable to arm the UAV")
            #     return 
            
            # get home altitude from GT odometry which mean barometric altitude at takeoff
            # self.home_alt = - np.array(node_OdomVIO.gt_odom_dict.copy()['position'][2])  # convert to DOWN
            self.home_alt = 0.0
            
        # give high thrust to takeoff and start VIO
        if not self.TAKEOFF_STARTED:
            self._phase_logger.takeoff_started(pf=False)
        self.TAKEOFF_STARTED = True
        yaw_target = 0.0 # or 180 for south
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, 0, 0]), thrust=self.DEFAULT_TAKEOFF_THRUST*0.57)
                
                
    def _climb(self, node_OdomVIO, node_PixhawkCMD):
        
        
         # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        PF_pos  = node_OdomVIO.pf_pos_dict['position'].copy()
        VIO_vel  = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.PF_pos_first, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], PF_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], PF_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)
        pitch_target = np.clip(pitch_target, -10.0, 10.0)
        roll_target  = np.clip(roll_target , -10.0, 10.0)

        # Vertical position control
        alt = abs(PF_pos[2]) 
        alt_diff = self.alt_target_climb - alt

        if alt < 5.0:    # Low altitude boost for safety climb on takeoff
            thrust_target = 0.65*self.DEFAULT_TAKEOFF_THRUST
            self._phase_logger.climb_progress(self.progress_log_interval_s,self.alt_target_climb,alt,alt_diff,low_alt_boost=True)
        elif alt_diff > 2.0:

            self._phase_logger.climb_progress(self.progress_log_interval_s,self.alt_target_climb,alt,alt_diff,low_alt_boost=False)
            if alt_diff > self.alt_thresh_climb_low:
                thrust_target = 0.85*self.DEFAULT_TAKEOFF_THRUST

            else:
                thrust_target = max(min(0.6 + (0.2/self.alt_thresh_climb_low) * alt_diff, 0.85*self.DEFAULT_TAKEOFF_THRUST), 0.5)

        else:
            thrust_target = 0.5
            self.CLIMB = False
            self.TRACK = True
            # self.DESCEND = True
            self._phase_logger.climb_complete(alt, self.alt_target_climb)

            # Generating trajectory from waypoints using climb position as starting point
            shaped_wp_list  = self.wp_list.copy() + PF_pos
            self.traj.generate_traj_from_wplist_interp(shaped_wp_list, coordinate_type="ned")
            self.generated_traj = self.traj.get_pos_vel_acc_in_ned()
            node_OdomVIO._update_yaw_difference()


        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(PF_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
    


    def _track_trajectory(self, node_OdomVIO, node_PixhawkCMD):

        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        PF_pos = node_OdomVIO.pf_pos_dict['position'].copy()
        VIO_vel = np.array(VIO_dict['velocity'])

        # Get reference position and velocity from trajectory generation
        self.traj_id +=1
        if self.traj_id >= len(self.generated_traj["pos"])-1:
            self.traj_id = len(self.generated_traj["pos"])-1
            
            self.TRACK   = False
            self.DESCEND = True
            self.VIO_descend_pos = PF_pos.copy()

        ref_pos = self.generated_traj["pos"][self.traj_id,:].copy()
        ref_vel = self.generated_traj["vel"][self.traj_id,:].copy()

        # Get acc commands from position controllers
        acc_cmd_x = self.pos_controller_x.update(ref_pos[0], ref_vel[0], PF_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y.update(ref_pos[1], ref_vel[1], PF_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

        # Limit acceleration command
        if np.linalg.norm(acc_cmd_xy) > self.max_acc:
            acc_cmd_xy = acc_cmd_xy / np.linalg.norm(acc_cmd_xy) * self.max_acc

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        # Get yaw target from velocity reference
        yaw_target = 0.0

        # Get thrust reference (assume hover thrust for now)
        thrust_ref = 0.5 

        # Get attitude reference from acceleration commands and yaw target
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc)
        
        # Set attitude and thrust to Pixhawk from attitude reference and thrust reference
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_ref)

        # # Store UAV position and GT position for visualization
        # self.PF_pos_list.append(PF_pos.copy())
        # self.VIO_pos_list.append(VIO_dict['position'].copy())
        # if node_OdomVIO.GTned_dict['ts'] is not None:
        #     self.GT_pos_list.append(node_OdomVIO.GTned_dict['position'].copy())

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(PF_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
        
        
    def _descend(self, node_OdomVIO, node_PixhawkCMD):

        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        PF_pos = node_OdomVIO.pf_pos_dict['position'].copy()
        VIO_vel = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.VIO_descend_pos, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], PF_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], PF_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0])

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)

        # Vertical position control
        # alt_diff = (-PF_pos[2]) - self.alt_target_descend
        # alt_baro = - (node_OdomVIO.GTned_dict.copy()['position'][2] - self.home_alt)  # use barometric altitude for descend
        alt_baro = node_OdomVIO.baroAlt
        alt_diff = alt_baro - self.alt_target_descend  

        if alt_diff > 1:
            # print("Descending to target altitude: ", self.alt_target_descend, "Current altitude: ", -PF_pos[2], "diff: ", alt_diff),
            self._phase_logger.descend_progress(self.progress_log_interval_s,self.alt_target_descend,alt_baro,alt_diff)

            if alt_diff > self.alt_thresh_descend_low:
                thrust_target = self.DEFAULT_LANDING_THRUST + 0.3
            else:
                thrust_target = min(max(0.4 - (0.1/self.alt_thresh_descend_low) * alt_diff, self.DEFAULT_LANDING_THRUST), 0.5)
        else:
            thrust_target = 0.485
            self._phase_logger.descend_complete(alt_baro, self.alt_target_descend)
            self.DESCEND = False
            self.LANDING = True
            
            self.VIO_landing_pos = PF_pos.copy()
            
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(PF_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)

    def _land(self, node_OdomVIO, node_PixhawkCMD): 
        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        PF_pos = node_OdomVIO.pf_pos_dict['position'].copy()
        VIO_vel = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.VIO_landing_pos, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], PF_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], PF_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0])

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)
        # clip pitch and roll to small angles for landing safety
        pitch_target = np.clip(pitch_target, -5.0, 5.0)
        roll_target  = np.clip(roll_target, -5.0, 5.0)

        # Vertical position control
        # alt_diff = (-PF_pos[2]) - 0.0
        # alt_baro = -( node_OdomVIO.GTned_dict.copy()['position'][2] - self.home_alt)  # use barometric altitude for descend
        alt_baro = node_OdomVIO.baroAlt
        alt_diff = alt_baro - 0.0  

        if alt_diff > 0.1:
            self._phase_logger.landing_progress(self.progress_log_interval_s,alt_baro,alt_diff)
            if alt_diff > self.alt_thresh_landing_low:
                thrust_target = self.DEFAULT_LANDING_THRUST + 0.41
            else:
                thrust_target = min(max(0.485 - (0.09/self.alt_thresh_landing_low) * alt_diff, self.DEFAULT_LANDING_THRUST + 0.4), 0.5)
        else:
            thrust_target = 0.485
            self._phase_logger.landing_complete(alt_baro)
            # self.LANDING = False  # NOTE : keep landing flag true for forcing low thrust for altitude measurement errors
                        
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(PF_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
        
    def _divergence_maneuver(self, node_PixhawkCMD):

        if self.VIO_DIVERGENCE:
            if time.time() - self.VIO_DIVERGENCE_START_TIME > self.EMERGENCY_LAND_TIMEOUT:
                logger.error("Emergency landing: VIO divergence recovery timed out")
                node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 0.0, 0.0]), thrust=0.3)

            else:
                node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 0.0, 0.0]), thrust=0.5)

        else:
            logger.warning("Attempting VIO divergence recovery maneuver")
            node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 10.0, 0.0]), thrust=0.5)
                

    def _log(self, yaw_target, pitch_target, roll_target, PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, log_file_name):

        if self.LOG:
            ref_angles = np.array([yaw_target, pitch_target, roll_target])
            ref_posvel = np.array([PF_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
            ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
            with open(log_file_name, "ab") as f:
                np.savetxt(f, ref_posvel,  delimiter=',')
            
            
    def _print_status(self, PF_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target):
        if self.print:
            should_print, self.last_print_time = _should_emit_every(self.last_print_time, 1.0)
            if should_print:
                self._phase_logger.log(
                    "STATUS",
                    logging.INFO,
                    "\ndiff_pos: %s | diff_vel: %s\nVIO pos : %s    VIO vel : %s\nREF pos : %s    REF vel : %s\nACC XY : [%0.4f, %0.4f]    RPY (yaw, pitch, roll) : [%0.4f, %0.4f, %0.4f]",
                    np.round(ref_pos - PF_pos, 4),
                    np.round(ref_vel - VIO_vel, 4),
                    np.round(PF_pos, 4),
                    np.round(VIO_vel, 4),
                    np.round(ref_pos, 4),
                    np.round(ref_vel, 4),
                    a_n,
                    a_e,
                    yaw_target,
                    pitch_target,
                    roll_target,
                )
