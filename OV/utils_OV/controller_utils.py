import numpy as np
import time
import yaml
from utils_OV.guidance_utils import TrajectoryGeneratorV2


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
        self.VIO_pos_list   = []
        self.GT_pos_list    = []
        self.generated_traj = None
        self.VIO_descend_pos = np.array([0,0,0])
        self.print = print
        self.last_print_time = time.time()
        self.LOG   = LOG
        

        # Log flight
        if self.LOG:
            self.date_var = time.strftime("%Y%m%d-%H%M%S")
            self.log_file_name = "logs/pos_controller_test_with_odom_{}.txt".format(self.date_var)

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

        # self.VIO_pos_first = node_OdomVIO.VIOned_dict['position'].copy()
        
        t_prev = time.time()
        while True:
            if time.time() - t_prev > self.controller_dt:  # apply control at the specified controller_dt
                t_prev = time.time()


                # Problem detected, abort mission
                if len(self.PROBLEM) > 0:
                    # Handle each problem
                    print("Aborting mission due to detected problems:")
                    for problem in self.PROBLEM:
                        print(f"Problem detected: {problem}")

                    return 
            
                # TAKEOFF phase
                if self.TAKEOFF:
                    self._take_off(node_OdomVIO, node_PixhawkCMD)
                    
                    if node_OdomVIO.initialization_status and self.TAKEOFF_STARTED:

                        self.TAKEOFF = False
                        self.CLIMB   = True
                        print(f"Takeoff completed, VIO started. Start climbing to target altitude: {self.alt_target_climb}")
                        
                        # Get initial position from VIO once yaw ref is initialized
                        # while not node_OdomVIO.ned_conversion_initialized:
                        while_timeout = time.time() + 5  # 5 seconds timeout
                        while True:
                            
                            try:
                                self.VIO_pos_first = node_OdomVIO.VIOned_dict['position'].copy()
                                break
                            except Exception as e:
                                print(f"Error getting VIO ned position: {e}")
                                time.sleep(0.1)
                                
                            if time.time() > while_timeout:
                                print("Timeout while getting initial VIO position...exiting")
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

                    # visualize2DgenTraj(generated_traj['pos'][:,0:2], np.array(UAV_pos_list))
                    np.save('logs/VIO_pos_list_{}.npy'.format(self.date_var),   np.array(self.VIO_pos_list))
                    np.save('logs/generated_traj_{}.npy'.format(self.date_var), self.generated_traj['pos'][:,0:2])
                    np.save('logs/GT_pos_list_{}.npy'.format(self.date_var),    np.array(self.GT_pos_list))

                    # RESET position controllers
                    self.pos_controller_x.reset()
                    self.pos_controller_y.reset()

                    # Reset traj id
                    self.traj_id = 0

                    # Reset lists
                    self.VIO_pos_list = []
                    self.GT_pos_list  = []

                    # Reset yaw
                    node_OdomVIO._update_yaw_difference()

                    if self.DONE:
                        print("Mission completed successfully.")
                        return
                        
                    else:
                        print(f"Mode is not GUIDED or GUIDED_NOGPS, interrupting the mission... Current mode: {node_OdomVIO.state_dict['mode']}")
                        return
                                        

                # Try to recover from VIO divergence
                if node_OdomVIO.try_recover_maneuver and not node_OdomVIO.vio_divergence_detected:
                    self._divergence_maneuver(node_PixhawkCMD)
                                    
                # Check VIO divergence to terminate mission
                if node_OdomVIO.vio_divergence_detected:
                
                    if not self.VIO_DIVERGENCE:
                    
                        print("VIO divergence detected, stopping the mission...")
                        self.VIO_DIVERGENCE_START_TIME = time.time() # 60 seconds to land
                        # self.DONE = True
                        self.VIO_DIVERGENCE = True

                    else:
                        self._divergence_maneuver(node_PixhawkCMD)

                



    def _take_off(self,node_OdomVIO, node_PixhawkCMD):
        
        # send arm message
        while_timeout = time.time() + 5  # 10 seconds to arm
        while not node_OdomVIO.state_dict['armed']:
            node_PixhawkCMD.arm(True)
            time.sleep(1)

            if time.time() > while_timeout:
                print("Arming timeout...exiting")
                self.PROBLEM.append("Arming timeout, unable to arm the UAV")
                return 
            
            # get home altitude from GT odometry which mean barometric altitude at takeoff
            # self.home_alt = - np.array(node_OdomVIO.gt_odom_dict.copy()['position'][2])  # convert to DOWN
            self.home_alt = 0.0
  
        # give high thrust to takeoff and start VIO
        self.TAKEOFF_STARTED = True
        yaw_target = 0.0 # or 180 for south
        print("Start to give high thrust to takeoff until VIO initialized"),
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, 0, 0]), thrust=self.DEFAULT_TAKEOFF_THRUST*0.57)
                
                
    def _climb(self, node_OdomVIO, node_PixhawkCMD):
        
        
         # GEt odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        VIO_pos  = np.array(VIO_dict['position'])
        VIO_vel  = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.VIO_pos_first, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)
        pitch_target = np.clip(pitch_target, -10.0, 10.0)
        roll_target  = np.clip(roll_target , -10.0, 10.0)

        # Vertical position control
        alt = abs(VIO_pos[2])  # DOWN is positive
        alt_diff = self.alt_target_climb - alt

        if alt < 5.0:    # Low altitude boost for safety climb on takeoff
            thrust_target = 0.65*self.DEFAULT_TAKEOFF_THRUST
            print(f"Climbing to target altitude: {self.alt_target_climb} Current altitude: {alt} diff: {alt_diff} (Low altitude boost)")

        elif alt_diff > 2.0:

            print(f"Climbing to target altitude: {self.alt_target_climb} Current altitude: {alt} diff: {alt_diff}")
            if alt_diff > self.alt_thresh_climb_low:
                thrust_target = 0.85*self.DEFAULT_TAKEOFF_THRUST

            else:
                thrust_target = max(min(0.6 + (0.2/self.alt_thresh_climb_low) * alt_diff, 0.85*self.DEFAULT_TAKEOFF_THRUST), 0.5)

        else:
            thrust_target = 0.5
            self.CLIMB = False
            self.TRACK = True
            # self.DESCEND = True
            print('Climb altitude reached...')

            # Generating trajectory from waypoints using climb position as starting point
            shaped_wp_list  = self.wp_list.copy() + VIO_pos
            self.traj.generate_traj_from_wplist_interp(shaped_wp_list, coordinate_type="ned")
            self.generated_traj = self.traj.get_pos_vel_acc_in_ned()
            node_OdomVIO._update_yaw_difference()


        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(VIO_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
    


    def _track_trajectory(self, node_OdomVIO, node_PixhawkCMD):

        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        VIO_pos = np.array(VIO_dict['position'])
        VIO_vel = np.array(VIO_dict['velocity'])

        # Get reference position and velocity from trajectory generation
        self.traj_id +=1
        if self.traj_id >= len(self.generated_traj["pos"])-1:
            self.traj_id = len(self.generated_traj["pos"])-1
            
            self.TRACK   = False
            self.DESCEND = True
            self.VIO_descend_pos = VIO_pos.copy()

        ref_pos = self.generated_traj["pos"][self.traj_id,:].copy()
        ref_vel = self.generated_traj["vel"][self.traj_id,:].copy()

        # Get acc commands from position controllers
        acc_cmd_x = self.pos_controller_x.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
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

        # Store UAV position and GT position for visualization
        self.VIO_pos_list.append(VIO_pos[0:2].copy())  
        if node_OdomVIO.GTned_dict['ts'] is not None:
            self.GT_pos_list.append(node_OdomVIO.GTned_dict['position'].copy())
            
        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(VIO_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
        
        
    def _descend(self, node_OdomVIO, node_PixhawkCMD):

        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        VIO_pos = np.array(VIO_dict['position'])
        VIO_vel = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.VIO_descend_pos, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0])

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)

        # Vertical position control
        # alt_diff = (-VIO_pos[2]) - self.alt_target_descend
        # alt_baro = - (node_OdomVIO.GTned_dict.copy()['position'][2] - self.home_alt)  # use barometric altitude for descend
        alt_baro = node_OdomVIO.baroAlt
        alt_diff = alt_baro - self.alt_target_descend  

        if alt_diff > 1:
            # print("Descending to target altitude: ", self.alt_target_descend, "Current altitude: ", -VIO_pos[2], "diff: ", alt_diff),
            print(f"Descending to target altitude: {self.alt_target_descend} Current altitude: {alt_baro} diff: {alt_diff}")

            if alt_diff > self.alt_thresh_descend_low:
                thrust_target = self.DEFAULT_LANDING_THRUST + 0.3
            else:
                thrust_target = min(max(0.4 - (0.1/self.alt_thresh_descend_low) * alt_diff, self.DEFAULT_LANDING_THRUST), 0.5)
        else:
            thrust_target = 0.485
            print('Descend altitude reached...')
            self.DESCEND = False
            self.LANDING = True
            
            self.VIO_landing_pos = VIO_pos.copy()
            
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(VIO_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)

    def _land(self, node_OdomVIO, node_PixhawkCMD): 
        # Get odometry data from VIO
        VIO_dict = node_OdomVIO.VIOned_dict.copy()
        VIO_pos = np.array(VIO_dict['position'])
        VIO_vel = np.array(VIO_dict['velocity'])

        # Lateral position control
        ref_pos, ref_vel = self.VIO_landing_pos, self.ref_vel

        acc_cmd_x = self.pos_controller_x_climbdescend.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
        acc_cmd_y = self.pos_controller_y_climbdescend.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0])

        a_n = acc_cmd_xy[0]
        a_e = acc_cmd_xy[1]

        yaw_target = 0.0
        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=self.max_acc_climbdescend)
        # clip pitch and roll to small angles for landing safety
        pitch_target = np.clip(pitch_target, -5.0, 5.0)
        roll_target  = np.clip(roll_target, -5.0, 5.0)

        # Vertical position control
        # alt_diff = (-VIO_pos[2]) - 0.0
        # alt_baro = -( node_OdomVIO.GTned_dict.copy()['position'][2] - self.home_alt)  # use barometric altitude for descend
        alt_baro = node_OdomVIO.baroAlt
        alt_diff = alt_baro - 0.0  

        if alt_diff > 0.1:
            print(f"Landing to ground: {0.0} Current altitude: {alt_baro} diff: {alt_diff}")
            if alt_diff > self.alt_thresh_landing_low:
                thrust_target = self.DEFAULT_LANDING_THRUST + 0.41
            else:
                thrust_target = min(max(0.485 - (0.09/self.alt_thresh_landing_low) * alt_diff, self.DEFAULT_LANDING_THRUST + 0.4), 0.5)
        else:
            thrust_target = 0.485
            print('Landed...')
            # self.LANDING = False  # NOTE : keep landing flag true for forcing low thrust for altitude measurement errors
                        
        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
        self._print_status(VIO_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target)

        # Log data
        self._log(yaw_target, pitch_target, roll_target, VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, self.log_file_name)
        
    def _divergence_maneuver(self, node_PixhawkCMD):

        if self.VIO_DIVERGENCE:
            if time.time() - self.VIO_DIVERGENCE_START_TIME > self.EMERGENCY_LAND_TIMEOUT:
                print("Executing emergency landing due to VIO divergence...")
                node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 0.0, 0.0]), thrust=0.3)

            else:
                node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 0.0, 0.0]), thrust=0.5)

        else:
            print("Trying to recover from VIO divergence...")
            node_PixhawkCMD.set_attitude(np.deg2rad([0.0, 10.0, 0.0]), thrust=0.5)

                    
    def _log(self, yaw_target, pitch_target, roll_target, VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, log_file_name):

        if self.LOG:
            ref_angles = np.array([yaw_target, pitch_target, roll_target])
            ref_posvel = np.array([VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
            ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
            with open(log_file_name, "ab") as f:
                np.savetxt(f, ref_posvel,  delimiter=',')
            
            
    def _print_status(self, VIO_pos, VIO_vel, ref_pos, ref_vel, a_n, a_e, yaw_target, pitch_target, roll_target):
        if self.print:
            if self.last_print_time + 1 < time.time():
                print(f"diff_pos: {np.round(ref_pos - VIO_pos, 4)} | diff_vel: {np.round(ref_vel - VIO_vel, 4)}\n"
                      f"VIO pos : {np.round(VIO_pos, 4)}    VIO vel : {np.round(VIO_vel, 4)}\n"
                      f"REF pos : {np.round(ref_pos, 4)}    REF vel : {np.round(ref_vel, 4)}\n"
                      f"ACC XY : [{a_n:.4f}, {a_e:.4f}]    RPY (yaw, pitch, roll) : [{yaw_target:.4f}, {pitch_target:.4f}, {roll_target:.4f}]")
                self.last_print_time = time.time()
                
                self.last_print_time = time.time()
