import numpy as np
# from scipy.spatial.transform import Rotation as R
from utils import wrap2_pi, quat2eul, quat2rotm
from scipy.stats import norm
from math import ceil
import time
from Timer import Timer

# Define the dynamics function for RK4
def dynamics(particles, u_input, noise_vec):
    """Compute derivatives for the particle dynamics"""
    derivs = np.zeros_like(particles)
    derivs[0, :] = u_input[0] + noise_vec[0, :]  # dx/dt
    derivs[1, :] = u_input[1] + noise_vec[1, :]  # dy/dt
    derivs[2, :] = 0                             # dz/dt
    # derivs[3, :] = u_input[1] + noise_vec[3, :]  # dyaw/dt
    
    derivs[3, :] = 0*u_input[1] + noise_vec[3, :]  # dyaw/dt

    return derivs


class StateEstimatorMPF:
    """
    This is the Python translation of the MATLAB StateEstimatorMPF class.

    Main property of MPF: states of filter is divided into two parts: nonlinear and linear.
    Nonlinear states represent the Monte Carlo filtering procedure. Particles are updated
    via a measurement function that depends on position/orientation (highly nonlinear).
    
    Linear states (velocity, orientation, accelerometer bias, gyroscope bias) are updated 
    via Kalman Filtering. We combine Monte Carlo for nonlinear states and KF for linear states.
    """
    def __init__(
        self, N, mu_part, std_part, mu_kalman, cov_kalman, circular_var, dt, dt_mpf_meas_update, v,
        gimballedCamera = False, KLDsamplingFlag = False, 
        KLDparams = {'epsilon': 0.15, 'delta': 0.01, 'binSize': 5.00, 'nMax': 500,  'nMin': 10}):
        """
        Constructor for the MPF object.

        Parameters
        ----------
        N : int
            Number of particles
        mu_part : ndarray of shape (n,) or (n, 2)
            Particle initialization parameter. Two modes:
            - Gaussian mode: 1D array of shape (n,) representing mean of position error states
            - Uniform mode: 2D array of shape (n, 2) with [min, max] boundaries for each dimension
        std_part : ndarray of shape (n,) or None
            Standard deviation of position error states (nonlinear states).
            - Gaussian mode: 1D array of shape (n,)
            - Uniform mode: None (boundaries specified in mu_part)
        mu_kalman : ndarray of shape (12,)
            Mean of linear states (velocity, orientation error, bias, etc.)
        cov_kalman : ndarray of shape (12, 12)
            Covariance matrix of linear states
        circular_var : boolean array of shape (n_total,)
            Flag for circular variable (e.g., yaw) handling
        dt : float
            Timestep
        R_meas : float or ndarray
            Measurement noise parameter used in measurement update
            
        """
        # MPF parameters
        self.N = N
        self.circular_var = circular_var  # boolean array for circular variables
        self.dt = dt
        self.dt_mpf_meas_update = dt_mpf_meas_update
        self.v = v  # likelihood exponentiel parameter
        self.n_nonlin = np.asarray(mu_part).shape[0]  # usually 3 or 4 (x, y, z, yaw error)
        self.n_lin = 0 if mu_kalman is None else len(mu_kalman)   # usually 12
        self.gimballedCamera = gimballedCamera # Flag for gimballed camera 
        self.cond_meas_upt = False
        self.last_meas_update_time = time.time()
        
        # KLD parameters
        self.KLDsamplingFlag = KLDsamplingFlag
        self.KLDparam        = KLDparams 
            
        # Count of estimates
        self.count_est = 0

        # Full error-state estimate (3 nonlinear + 12 linear = 15 total)
        n_total = self.n_nonlin + self.n_lin
        self.X = np.zeros((n_total, 1))
        self.P = np.diag(np.zeros(n_total))

        # Initialize the particle set, weights, linear KF states, etc.
        self._mpf_initialize(mu_part, std_part, mu_kalman, cov_kalman)

        # Placeholders for IMU objects; set these externally
        #   e.g., self.Accelerometer = ...
        #         self.Gyroscope = ...
        self.Accelerometer = None
        self.Gyroscope = None
        self.FramemostLikelihoodPart = None

        # Placeholder for a database scanner object
        #   e.g., self.DataBaseScanner = ...
        self.DataBaseScanner = None

        # Will store likelihood for each particle
        self.likelihood = np.ones(self.N)
        
        # Counter for SimPerclosedLoop
        self.predCount_bofore_closedLoop = 0

    def getEstimate(self, u, Xnom, UAVKp, UAVDesc , closedLoop = False, predPerclosedLoop = 1 , UAVframe = None):
        """
        Main estimation loop of MPF:
            1) Predict error states based on IMU signals
            2) Compute likelihood of each particle based on image matching
            3) Update (weights) with that likelihood (Bayes)
            4) Estimate the final states
            5) Possibly resample the particles

        Parameters
        ----------
        u : ndarray of shape (6,)
            IMU signals: [body_accel_x, body_accel_y, body_accel_z,
                          body_gyro_x,  body_gyro_y,  body_gyro_z]
        X_nom : ndarray of shape (16,)
            Nominal INS states (position, velocity, quaternion, biases, etc.)
            - In the MATLAB code, X_nom has shape (16,) but used up to index 16
              (1:3 pos, 4:6 vel, 7:10 quat, 11:13 accel bias, 14:16 gyro bias).
        UAVImage : any
            The UAV camera image (features extracted, etc.)

        Returns
        -------
        param_estimate : dict
            Dictionary containing:
                "State":      (15,)  the combined MPF error-state estimate
                "Covariance": (15,15) the combined MPF error-state covariance
        """
        self.count_est += 1

        # 1) Predict
        self._predict(u,Xnom)

        # Only perform measurement update if the time since last update meas_update exceeds dt_mpf_meas_update
        if self.cond_meas_upt: # or (self.count_est == 1):

            with Timer("MPF measurement update"):
                # Record the time of this measurement update
                self.last_meas_update_time = time.time()
                self.cond_meas_upt         = False

                # print(f'yaw particle:', np.rad2deg(self.particles[3,1]))

                # 2) Likelihood from DB (image matching)
                self._find_likelihood_particles(Xnom, UAVKp, UAVDesc)

                # 3) Measurement update (weights)
                self._update_weights()
            

        # 4) Evaluate the final state and covariance
        self._estimate(closedLoop, predPerclosedLoop)

        # 5) Resample if needed
        self._resample()

        # Prepare a dictionary for output
        param_estimate = {
            "State":      self.X.copy(),  # (15,1)
            "Covariance": self.P.copy()   # (15,15)
        }
        return param_estimate

    # ---------------------------------------------------------------------
    #  "Private" methods (in Python, we'll just underscore them)
    # ---------------------------------------------------------------------
    def _mpf_initialize(self, mu_part, std_part, mu_kalman, cov_kalman):
        """
        Initializes the particle set, weights, and per-particle linear KF states.
        
        Supports two initialization modes:
        1. Gaussian distribution: mu_part is 1D array (n,), std_part is 1D array (n,)
        2. Uniform distribution: mu_part is 2D array (n, 2) with [min, max] boundaries, std_part is None
        """
        
        # Store the initialization parameters for reset_dist function
        if self.count_est == 0:
            self.mu_part    = mu_part
            self.std_part   = std_part
            self.mu_kalman  = mu_kalman
            self.cov_kalman = cov_kalman
        
        # Detect initialization mode based on mu_part shape
        mu_part_array = np.asarray(mu_part)
        
        if mu_part_array.ndim == 2 and mu_part_array.shape[1] == 2:
            # Uniform distribution mode: mu_part has shape (n, 2) with [min, max]
            # Extract boundaries
            lower_bounds = mu_part_array[:, 0]  # shape (n,)
            upper_bounds = mu_part_array[:, 1]  # shape (n,)
            
            # Generate uniformly distributed particles
            # shape: (n_nonlin, N)
            self.particles = np.random.uniform(
                low=lower_bounds.reshape(-1, 1),
                high=upper_bounds.reshape(-1, 1),
                size=(self.n_nonlin, self.N)
            )
            
        else:
            # Gaussian distribution mode: mu_part and std_part are 1D arrays
            # Create gaussian distributed particles around aircraft
            # shape: (3, N)
            mu_part_2d  = np.tile(mu_part_array.reshape(-1, 1), (1, self.N))
            std_part_2d = np.tile(std_part.reshape(-1, 1), (1, self.N))
            self.particles = mu_part_2d + std_part_2d * np.random.randn(self.n_nonlin, self.N)
        
        # Apply circular variable wrapping for both modes
        for (var_idx, var_flag) in enumerate(self.circular_var):
            if var_flag:
                # If circular variable, wrap the particles to [0, 2*pi]
                self.particles[var_idx, :] = wrap2_pi(self.particles.copy()[var_idx, :])  # Assuming the 3rd state is circular (e.g., yaw)

        # Initialize weights to 1/N
        self.weights = np.ones((self.N,)) / self.N

        # Create linear Kalman Filter states for each particle
        if mu_kalman is not None:
            mu_kalman_2d = np.tile(mu_kalman.reshape(-1, 1), (1, self.N))  # shape (12, N)
            cov_kalman_3d = np.tile(cov_kalman[:, :, np.newaxis], (1, 1, self.N))  # shape (12,12,N)
            self.KalmanFiltersState = mu_kalman_2d
            self.KalmanFiltersCovariance = cov_kalman_3d

    def _predict(self, u, Xnom):
        """
        Propagate error states' kinematic equations based on IMU vector u (body accel & gyro).
        """
        # if self.Accelerometer is None or self.Gyroscope is None:
        #     raise ValueError(
        #         "Accelerometer or Gyroscope objects have not been set. "
        #         "Please set them before calling `getEstimate`."
        #     )

        n = self.n_nonlin
        l = self.n_lin
        
        noise_std = 2*np.array([2, 2, 0, np.deg2rad(1)]) # Process noise for nonlinear states
        
        # eul_vio = quat2eul(Xnom[3:7])
        # self.particles[3, :] = eul_vio[0]  # Set yaw of all particles to nominal yaw from VIO
        
        # Add noise to all particles at once
        noise = noise_std.reshape(-1, 1) * np.random.randn(n, self.N)  # shape (4, N)
        
        # Runge-Kutta 4th order integration
        k1 = dynamics(self.particles.copy(), u, noise)
        k2 = dynamics(self.particles.copy() + 0.5 * self.dt * k1, u, noise)
        k3 = dynamics(self.particles.copy() + 0.5 * self.dt * k2, u, noise)
        k4 = dynamics(self.particles.copy() + self.dt * k3, u, noise)

        # Update particles using RK4 formula
        self.particles = self.particles.copy() + (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        # self.particles = self.particles + self.dt * k1 # Using Euler method for simplicity

        # Wrap the particles to [0, 2*pi] for circular variables
        for (var_idx, var_flag) in enumerate(self.circular_var):
            if var_flag:
                self.particles[var_idx, :] = wrap2_pi(self.particles.copy()[var_idx, :])


    def _find_likelihood_particles(self, Xnom, UAVKp=None, UAVDesc=None, UAVFrame=None, MaskOrthography=None, PartCorrection=None):
        """
        Find likelihood of each particle via image matching:
          - Build hypothetical positions by adding (INS nominal + position error)
          - Orientation error is assumed small or known => we use nominal yaw
          - Then call `self.DataBaseScanner.find_likelihood` for each particle.
        """
        if self.DataBaseScanner is None:
            raise ValueError(
                "No DataBaseScanner object attached (self.DataBaseScanner). "
                "Cannot perform image-based likelihood."
            )
            
        # Get nominal states
        eulNom = quat2eul(Xnom[3:7])  # shape (3,)  ZYX euler angles (yaw, pitch, roll)
        rotNom = quat2rotm(Xnom[3:7])  # shape (3,)  rotation matrix
        posNom = Xnom[0:3]          # shape (3,)
        
        #Calculating the particles position translation due to rotation of the UAV
        if not self.gimballedCamera and (PartCorrection is None):   # Using correction from roll pitch altitude compensation
            rotM_hypo               = rotNom #quat2rotm(qcor)                           # shape (3, 3)
            range_finder_body       = np.array([0, 0, 1])                       # Assuming the range is in the Z direction
            range_finder_world      = rotM_hypo @ range_finder_body             # shape (3,)
            scale                   = np.abs(posNom[2]) / range_finder_world[2]  # shape (1,)

            delta_pos               = range_finder_world * scale  # shape (3,)
            # print('altitude', abs(posNom[2]))
            # print('euler angles', np.rad2deg(eulNom))
            # print("delta_pos", delta_pos)
            print(f"Not Gimballed camera correction: altitude: {abs(posNom[2])} | euler angles (deg): {np.rad2deg(eulNom)} | delta_pos: {delta_pos}")

            delta_pos[2] = 0 # Set deltaZ to zero

        elif not self.gimballedCamera and (PartCorrection is not None):   # Using correction from orthoprojection using intersection of ground plane with camera center ray
            delta_pos = PartCorrection.reshape(3,)
            print(f"Not Gimballed camera correction with PartCorrection input: delta_pos: {delta_pos}")
            
        else:
            delta_pos = np.zeros((3))  # No correction for gimballed camera
        
        # Hypothetical position of each particle => X_nom(1:3) + error
        PartXYZ = self.particles[0:3,:].T + delta_pos # shape (N,3)
        yaw = self.particles[3,:].T      # shape (N,)

        # Convert particles yaw to error yaw if orhoprojection is used
        if MaskOrthography is not None:
            yaw = yaw - eulNom[0]  # NOTE: check sign convention here

        # Now call the DB scanner to find likelihood for each particle
        # Expecting an array of length N back
        self.FramemostLikelihoodPart, ScoreParticles = self.DataBaseScanner.find_likelihood(np.atleast_2d(PartXYZ),yaw, UAVKp = UAVKp, UAVDesc = UAVDesc, UAVFrame = UAVFrame, MaskOrthography = MaskOrthography)
        
        # Likelihood calculation based on ScoreParticles with using logistic function
        self.likelihood = self._likelihood_func(ScoreParticles)
        
    def _update_weights(self):
        """
        Particle Filter measurement update on weights:
          weights = weights * likelihood
          then normalize
        """
        # Multiply prior weights by newly computed likelihood
        w_new = self.weights * self.likelihood

        # Avoid zero-division or 0 weights
        w_new += 1e-300
        w_new /= np.sum(w_new)

        self.weights = w_new

    def _estimate(self, closedLoop, predPerclosedLoop):
        """
        Compute the weighted mean and covariance of the full MPF error-state 
        (3D nonlinear + 12D linear).
        """
        # Nonlinear states: shape (3, N)
        xn = self.particles
        w  = self.weights
        w_sum = np.sum(w)

        # Weighted mean of nonlinear states (3,)
        xn_est = np.sum(xn * w[np.newaxis, :], axis=1) / w_sum

        # Weighted "diagonal" covariance for the 3D position error
        # The original code does an elementwise approach for the diagonal only
        # Pn_est => (3,)
        diff_n = xn - xn_est.reshape(-1, 1)
        Pn_est = np.sum((diff_n**2) * w[np.newaxis, :], axis=1) / w_sum
        Pn_est_mat = np.diag(Pn_est)

        # Combine into full 15D error-state
        self.X = xn_est
        self.P = Pn_est_mat
        
        # Closed-loop reset of states with mean removal if needed
        if closedLoop and (self.predCount_bofore_closedLoop == predPerclosedLoop): #and self.meas_updated:
            self.particles          = self.particles.copy() - xn_est.reshape(-1, 1)
            self.predCount_bofore_closedLoop = 0

        self.predCount_bofore_closedLoop += 1

    def _resample(self):
        """
        Systematic resampling if effective N < N/2
        """
        n_eff = self._neff()
        if n_eff < self.N / 2:
        # if False:
            indices = self._resample_systematic()
            
            # Trigger adaptive KLD resampling
            if self.KLDsamplingFlag:
                np.random.shuffle(indices)
                indices = self.KLDsampling(indices)

            # Resample from indices
            self.particles               = self.particles.copy()[:, indices]
            if self.n_lin > 0:
                self.KalmanFiltersState      = self.KalmanFiltersState[:, indices]
                self.KalmanFiltersCovariance = self.KalmanFiltersCovariance[:, :, indices]

            # Reset weights
            self.weights = np.ones(self.N) / self.N

            # If you wanted to do closed-loop reset of linear states:
            # self.KalmanFiltersState = np.zeros_like(self.KalmanFiltersState)
            print("---------------------------------------")
            print("--------------Resampled----------------")
            print("---------------------------------------")

    def _resample_systematic(self):
        """
        Systematic resampling of particles according to weights.
        Returns
        -------
        indices : ndarray of shape (N,)
            The new indices of particles to keep.
        """
        # Cumulative sum of weights
        cum_sum = np.cumsum(self.weights)
        # The random start
        step = 1.0 / self.N
        T = np.linspace(0, 1 - step, self.N) + np.random.rand() * step
        # Make sure the last element is exactly 1
        T = np.append(T, 1.0)

        indices = np.zeros(self.N, dtype=int)
        i, j = 0, 0
        while i < self.N and j < self.N:
            if T[i] < cum_sum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1
        return indices

    def _neff(self):
        """
        Effective number of particles. If n_eff < N/2, triggers resampling.
        """
        return 1.0 / np.sum(self.weights**2)
    
    def _likelihood_func(self,ScoreParticles):
        # Calculate likelihood value of each particles with given number of match
        # v :  hyper parameter of likelihood function, bigger value eliminite more particles even they relatively high number of matched feature
        # ScoreParticles : number of matched feature of each particles
        
        v = self.v 

        
        # No likelihood update if ScoreParticles is less than 50
        # if np.max(ScoreParticles) <= 30:
        if False:
            self._resetDistribution()
            return  np.ones(self.N)
        else:
            
            if not self.DataBaseScanner.FeatureDM.TemplateMatchingFlag:

                ScoreParticles = np.array(ScoreParticles) + 1e-5 # Force to be numpy array and add small number for preventing devision zero 
                
                mean = np.mean(ScoreParticles)
                min = np.min(ScoreParticles)
                max = np.max(ScoreParticles)

                # no measurement update if max number of matched feature is less than 50
                thresh = 50
                if max < 50:
                    print(f"No measurement update due to low max number of matched feature : thresh: {thresh}")
                    return np.ones(self.N)
                
                # max_nMatch = np.max(ScoreParticles)
                max_nMatch = 100
                ScoreParticles = ScoreParticles / max_nMatch # Normalize number of matched point to [0,1]
                

                likelihood = (1 / (1 + np.exp(-10 * (ScoreParticles - 0.5))) ** (1 / v)) / (1 / (1 + np.exp(-5)) ** (1 / v))
                
                mean_l = np.mean(likelihood)
                min_l = np.min(likelihood)
                
                print(f"mean_l: {mean_l}  mean: {mean}")
                print(f"min_l: {min_l},  min: {min}")
                
                return likelihood
        
                # ScoreParticles = np.array(ScoreParticles) + 1e-5 # Force to be numpy array and add small number for preventing devision zero 
                # ScoreParticles = ScoreParticles / max(ScoreParticles) # Normalize number of matched point to [0,1]
                # return (1 / (1 + np.exp(-10 * (ScoreParticles - 0.5))) ** (1 / v)) / (1 / (1 + np.exp(-5)) ** (1 / v))

            else:
                ScoreParticles = np.array(ScoreParticles)
                likelihood = (1 / (1 + np.exp(-5*ScoreParticles))**(1 / v)) / (1 / (1 + np.exp(-5))**(1 / v))

                mean = np.mean(ScoreParticles)
                min = np.min(ScoreParticles)

                mean_l = np.mean(likelihood)
                min_l = np.min(likelihood)

                print(f"mean_l: {mean_l}  mean: {mean}")
                print(f"min_l: {min_l},  min: {min}")

                return likelihood


    def KLDsampling(self, indices):
        """
        Adaptive KLD sampling of *resampled* indices.
        Args:
            indices (array-like of int): initial list of particle-indices to draw from,
                                        typically sorted by weight descending.
        Returns:
            new_indices (List[int]): adaptively chosen subset (length M) of indices.
        """
        new_indices = []
        bins = set()           # set of occupied bins (as tuples)
        part_count = 0         # how many we've drawn so far
        N_upt = 1              # how many we *should* draw (updates as bins fill)
        k = 0                  # number of occupied bins

        # get KLD parameters
        N       = self.N
        nMin    = self.KLDparam['nMin']
        nMax    = self.KLDparam['nMax']
        binSize = self.KLDparam['binSize']
        epsilon = self.KLDparam['epsilon']
        delta   = self.KLDparam['delta']

        while (((part_count < N_upt) and (part_count < nMax)) or (part_count < nMin)):

            # 1) pick an index (if we run out, pick uniformly at random)
            if part_count <= N-1:
                p_idx = indices[part_count]
            else:
                p_idx = np.random.choice(indices)

            # 2) fetch the particle's first two dims and bin it
            p = self.particles[0:2, p_idx]              # shape (2,)
            b = tuple(np.floor(p / binSize).astype(int))

            # 3) if this bin is new, update k and recompute N_upt
            if b not in bins:
                bins.add(b)
                k = len(bins)
                # KLDcomputeRequiredParticleCount implements your
                # chi-squared / Wilson–Hilferty formula
                n_req = self.KLDcomputeRequiredParticleCount(k, epsilon, delta)
                N_upt = min(n_req, nMax)

            # 4) accept this sample
            new_indices.append(p_idx)
            part_count += 1


        self.N  = len(new_indices)
        print(f'Adaptive resampled {self.N} particles (k = {k} bins occupied)\n')

        return new_indices
    
    def KLDcomputeRequiredParticleCount(self, k, epsilon, delta):
        """
        Compute the required number of particles for KLD-sampling using
        the Wilson–Hilferty–transformed chi-squared approximation:

            n = ceil( ((k-1)/(2*ε)) * [1 - 2/(9(k-1)) + sqrt(2/(9(k-1))) * z]^3 )

        where z = norm.ppf(1 - delta).
        """
        if k <= 1:
            return 2

        # (1-δ)-quantile of standard normal
        z = norm.ppf(1 - delta)

        a = 1 - 2.0 / (9 * (k - 1))
        b = np.sqrt(2.0 / (9 * (k - 1))) * z
        factor = (a + b) ** 3

        n = ceil(((k - 1) / (2 * epsilon)) * factor)
        return n
    
    def _resetDistribution(self):
        self._mpf_initialize(self.mu_part, self.std_part, None, self.cov_kalman)