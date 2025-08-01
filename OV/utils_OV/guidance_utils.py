
import navpy
from geomdl import BSpline, knotvector
import numpy as np


class TrajectoryGeneratorV2:
    def __init__(self, sampling_freq=40, degree=3 , max_vel=[14.0,14,10], max_acc=[19.62, 19.62, 9.81], num_of_points=100):
        """
        Initialization of B-Spline Trajectory Generator Class
        param sampling_freq: sampling frequency of the trajectory
        type sampling_freq: int
        param degree: degree of generated B-Spline curve
        type  degree: int 
        param max_vel: maximum velocity for each axis
        type max_vel: list
        param max_acc: maximum acceleration for each axis
        type max_acc: list
        param num_of_points: number of points in the generated curve (will be used for limiting operations)
        type  num_of_points: int
        """
        self.num_of_points = num_of_points
        
        self.sampling_freq = sampling_freq
        
        self.curve = BSpline.Curve(normalize_kv=False)
        self.curve.delta = 1/self.num_of_points
        self.curve.degree = degree

        self.max_vel = np.array(max_vel)
        self.max_accel = np.array(max_acc)
        
        
    def generate_traj_from_wplist_interp(self, wp_list, coordinate_type="ned", zero_vel_at_start=True):
        
        # wp_list = np.vstack((wp_list[0], wp_list, wp_list[-1]))
        no_of_wpts = len(wp_list)
        if no_of_wpts <3:
            wp_list = np.insert(wp_list, 1, (wp_list[0] + wp_list[1])/2, axis=0)
            no_of_wpts = len(wp_list)
        self.L = no_of_wpts - 1
        
        self.A = np.eye(self.L+1)
        for i in range(self.L-1):
            self.A[i+1, i:i+3] = np.array([1, 4, 1])
        self.A[1, 0:3] = np.array([3/2, 7/2, 1])
        self.A[self.L-1, (self.L-2):(self.L+1)] = np.array([1, 7/2, 3/2])
        
        if zero_vel_at_start:
            self.curve.ctrlpts = np.zeros((self.L+5, 3)).tolist() # used for generating knotvector, will be overwritten when trajectory is generated
        else:
            self.curve.ctrlpts = np.zeros((self.L+3, 3)).tolist()
        knot_vector_magnitude = 1
        knot_vector = knot_vector_magnitude * np.array(knotvector.generate(self.curve.degree, len(self.curve.ctrlpts)))
        self.curve.knotvector = knot_vector.tolist()
        
        self.r = np.zeros((self.L+1, 3))
        for i in range(self.L):
            self.r[i, :] = 6*wp_list[i, :]
        self.r[0, :] = wp_list[0,:] + (wp_list[1,:] - wp_list[0,:])/2
        self.r[-1, :] = wp_list[-2,:] + (wp_list[-1,:] - wp_list[-2,:])/2
        d_interior = np.linalg.solve(self.A, self.r)
        if zero_vel_at_start:
            self.control_points = np.vstack((wp_list[0,:],wp_list[0,:], d_interior, wp_list[-1,:], wp_list[-1,:]))
        else:
            self.control_points = np.vstack((wp_list[0,:], d_interior, wp_list[-1,:]))
        
        knot_vector_mag = self.find_knot_vector_magnitude()
        self.generate_traj(self.control_points.tolist(), knot_vector_magnitude=knot_vector_mag)
    
    def generate_traj(self, control_points, knot_vector_magnitude=1, unit_knot_vector=None):
        """
        B-Spline Curve with uniform knot vector generation for given control points 
        and knot vector in cartesian coordinates
        param control_points: control points of the B-Spline curve
        type control_points: list
        param knot_vector_magnitude: Knot vector magnitude
        type knot_vector_magnitude: float
        param unit_knot_vector: unit knot vector. if None uniform knot vector will be generated
        type unit_knot_vector: list
        """

        self.curve.ctrlpts = control_points
        self.knot_vector_magnitude = knot_vector_magnitude

        if unit_knot_vector is None:
            knot_vector = self.knot_vector_magnitude * np.array(knotvector.generate(self.curve.degree, len(self.curve.ctrlpts)))
        else:
            knot_vector = self.knot_vector_magnitude * np.array(unit_knot_vector)
        
        self.curve.knotvector = knot_vector.tolist()


    def get_pos_vel_acc_in_ned(self):
        """
        Returns position, velocity, acceleration and relative time data 
        dictionary for generated trajectory in NED coordinates
        """
        self.num_of_samples = int(self.sampling_freq * self.knot_vector_magnitude)
        generated_traj = {
            "pos": np.zeros((self.num_of_samples,3)), 
            "vel": np.zeros((self.num_of_samples,3)), 
            "acc": np.zeros((self.num_of_samples,3)),
            "time": np.linspace(0, self.knot_vector_magnitude, self.num_of_samples) 
            }
        
        for i in range(self.num_of_samples):
            pos, vel, acc = np.array(
                self.curve.derivatives(u=self.knot_vector_magnitude*(i)/self.num_of_samples, order=2))
            generated_traj["pos"][i] = pos
            generated_traj["vel"][i] = vel
            generated_traj["acc"][i] = acc
            if np.linalg.norm(pos) < 0.01:
                print("u:", self.knot_vector_magnitude*(i)/self.num_of_samples)
                print("pos", pos)
        return generated_traj
    
    
    def get_pos_vel_acc_in_lla(self):
        """
        Returns position, velocity, acceleration and relative time data 
        dictionary for generated trajectory
        positions in lat lon alt format
        velocity and accel data in NED frame
        """
        self.num_of_samples = int(self.sampling_freq * self.knot_vector_magnitude)
        generated_traj = {
            "pos": np.zeros((self.num_of_samples,3)), 
            "vel": np.zeros((self.num_of_samples,3)), 
            "acc": np.zeros((self.num_of_samples,3)),
            "time": np.linspace(0, self.knot_vector_magnitude, self.num_of_samples) 
            }
        
        for i in range(self.num_of_samples):
            pos, vel, acc = np.array(
                self.curve.derivatives(u=self.knot_vector_magnitude*(i)/self.num_of_samples, order=2))
            generated_traj["pos"][i] = pos
            generated_traj["vel"][i] = vel
            generated_traj["acc"][i] = acc
            
        generated_traj["pos"] = navpy.ned2lla(generated_traj["pos"], self.lat_ref, self.lon_ref, self.alt_ref)
        generated_traj["pos"] = np.vstack(generated_traj["pos"]).T
        return generated_traj
    
    
    def generate_traj_for_elliptic_dive(self, start_pos_in_lla, target_pos_in_lla):
        """
        Generates trajectory for elliptic dive
        param start_pos_in_lla: start position of the dive
        type start_pos_in_lla: list
        param target_pos_in_lla: target position of the dive
        type target_pos_in_lla: list
        """
        self.lat_ref, self.lon_ref = target_pos_in_lla[0:2]  # ref coordinates for lla to ned transformation
        self.alt_ref = 0
        
        start_pos_in_ned = navpy.lla2ned(start_pos_in_lla[0], start_pos_in_lla[1], start_pos_in_lla[2], 
                                         self.lat_ref, self.lon_ref, self.alt_ref)
        
        target_pos_in_ned = navpy.lla2ned(target_pos_in_lla[0], target_pos_in_lla[1], target_pos_in_lla[2], 
                                       self.lat_ref, self.lon_ref, self.alt_ref)
        self.control_points = np.vstack((start_pos_in_ned,
                                          [0.75,0.75,1.2] * start_pos_in_ned,
                                          [0.5,0.5,1] * start_pos_in_ned,
                                          [0.25,0.25,0.8] * start_pos_in_ned,
                                          target_pos_in_ned))

        knot_vector_mag = self.find_knot_vector_magnitude()
        self.generate_traj(self.control_points.tolist(), knot_vector_magnitude=knot_vector_mag)
        
        
    def generate_traj_for_wp_list(self, wp_list, coordinate_type='lla'):
        """
        B-Spline trajectory generator for given waypoint list
        param lla_wp_list: waypoints in lat lon alt format
        type lla_wp_list: list
        """
        if coordinate_type == 'lla':
            self.lla_wp_list = np.array(wp_list)
            
            self.lat_ref, self.lon_ref = self.lla_wp_list[0][0:2]  # ref coordinates for lla to ned transformation
            self.alt_ref = 0
            self.wp_list = navpy.lla2ned(self.lla_wp_list[:,0], self.lla_wp_list[:,1], self.lla_wp_list [:,2], 
                                        self.lat_ref, self.lon_ref, self.alt_ref)
        elif coordinate_type == 'ned':
            self.wp_list = np.array(wp_list)
        else:
            raise ValueError("coordinate_type should be either 'lla' or 'ned'")
        number_of_wp = len(self.wp_list)
        
        if number_of_wp == 2:
            # straight line will be generated
            first_wp = self.wp_list[0]
            second_wp = self.wp_list[1]
            v1 = second_wp - first_wp # direction vector from first wp to second wp
            self.control_points = np.vstack((first_wp, first_wp,
                                             first_wp + 0.25*v1,
                                             first_wp + 0.50*v1,
                                             first_wp + 0.75*v1,
                                             second_wp, second_wp))
        elif number_of_wp >= 3:
            for i, (first_wp, second_wp, third_wp) in enumerate(zip(self.wp_list, self.wp_list[1:], self.wp_list[2:])):
                v1 = second_wp - first_wp # direction vector from first wp to second wp
                v2 = third_wp - second_wp # direction vector from second wp to third wp
                u1 = v1 / np.linalg.norm(v1) # unit direction vector
                u2 = v2 / np.linalg.norm(v2) # unit direction vector
                
                if i==0:
                    self.control_points = np.vstack((first_wp, first_wp, first_wp, 
                                                     first_wp + v1*0.1, 
                                                     (second_wp + first_wp)/2 -2.5*u2,
                                                     second_wp - 2*u1 - 2.2*u2,
                                                     second_wp,
                                                     second_wp + 2.2*u1 + 2*u2))
                    
                if (i == number_of_wp - 3):
                    self.control_points = np.vstack((self.control_points,
                                                     (first_wp+second_wp)/2 - 2.5*u2,
                                                     second_wp - 2*u1 - 2.5*u2, 
                                                     second_wp,  
                                                     second_wp + 2*u1 + 2*u2, 
                                                     (second_wp+third_wp)/2, 
                                                     third_wp -v2*0.1, 
                                                     third_wp, third_wp, third_wp))
                    
                elif i > 0:
                    self.control_points = np.vstack((self.control_points,
                                                     (first_wp+second_wp)/2 - 2.5*u2,
                                                     second_wp - 2*u1 - 2.5*u2, 
                                                     second_wp,  
                                                     second_wp + 2*u1 + 2*u2))
                
        knot_vector_mag = self.find_knot_vector_magnitude()
        self.generate_traj(self.control_points.tolist(), knot_vector_magnitude=knot_vector_mag)
        
    def find_knot_vector_magnitude(self):
        """
        Finds knot vector magnitude by limiting max velocity and max acceleration
        (Velocity and acceleration limits along x and y axis assumed to be equal)
        """
        self.generate_traj(self.control_points.tolist())
        # limit velocity and acceleration values
        generated_traj_in_ned = self.get_pos_vel_acc_in_ned()

        max_xy_vel_of_generated_traj = max(np.linalg.norm(generated_traj_in_ned["vel"][:,0:2],axis=1))
        max_z_vel_of_generated_traj = max(abs(generated_traj_in_ned["vel"][:,2]))
        max_xy_acc_of_generated_traj = max(np.linalg.norm(generated_traj_in_ned["acc"][:,0:2],axis=1))
        max_z_acc_of_generated_traj = max(abs(generated_traj_in_ned["acc"][:,2]))
        
        vel_scale_gain = max_xy_vel_of_generated_traj / self.max_vel[0]
        vel_scale_gain = max(vel_scale_gain, max_z_vel_of_generated_traj/self.max_vel[2])
        
        accel_scale_gain = np.sqrt(max_xy_acc_of_generated_traj / self.max_accel[0])
        accel_scale_gain = max(accel_scale_gain, np.sqrt(max_z_acc_of_generated_traj / self.max_accel[2]))
        
        knot_vector_magnitude = max(vel_scale_gain, accel_scale_gain)
        return knot_vector_magnitude
    
    def generate_wp_list_for_square_trajectory(self,edge_length, relative_wp_alt, edge_inter_cp = 1, order = 'NW'):

        #edge_inter_cp = 1  # number of intermediate control points between two corner of square
        n_step = edge_inter_cp + 1
        print(n_step)
        cp_dist = edge_length / (n_step)
        wp_list = [[0, 0, -relative_wp_alt]]


        # North - West order
        if order == 'NW':

            for corner in range(4):
                for i in range(n_step):
                    
                    if corner == 0:
                        wp_list.append([i*cp_dist, 0, -relative_wp_alt])
                    elif corner == 1:
                        wp_list.append([edge_length, -i*cp_dist, -relative_wp_alt])

                    elif corner == 2:
                        wp_list.append([edge_length - i*cp_dist, -edge_length, -relative_wp_alt])
                    
                    elif corner == 3:
                        wp_list.append([0, -edge_length + i*cp_dist, -relative_wp_alt])

        # West - North order
        elif order == 'WN':

            for corner in range(4):
                for i in range(n_step):

                    if corner == 0:
                        wp_list.append([0, -i*cp_dist, -relative_wp_alt])
                    elif corner == 1:
                        wp_list.append([i*cp_dist, -edge_length, -relative_wp_alt])

                    elif corner == 2:
                        wp_list.append([edge_length, -edge_length + i*cp_dist, -relative_wp_alt])
                    
                    elif corner == 3:
                        wp_list.append([edge_length - i*cp_dist, 0, -relative_wp_alt])

        # Add the last point to close the square
        if edge_inter_cp > 0:
            wp_list.append([0, 0, -relative_wp_alt])

        return wp_list


