import numpy as np
import time


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