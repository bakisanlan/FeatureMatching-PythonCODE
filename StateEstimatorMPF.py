import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import *


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
        self, N, mu_part, std_part, mu_kalman, cov_kalman, dt, dt_mpf_meas_update, v
    ):
        """
        Constructor for the MPF object.

        Parameters
        ----------
        N : int
            Number of particles
        mu_part : ndarray of shape (3,)
            Mean of position error states (nonlinear states)
        std_part : ndarray of shape (3,)
            Std dev of position error states (nonlinear states)
        mu_kalman : ndarray of shape (12,)
            Mean of linear states (velocity, orientation error, bias, etc.)
        cov_kalman : ndarray of shape (12, 12)
            Covariance matrix of linear states
        dt : float
            Timestep
        R_meas : float or ndarray
            Measurement noise parameter used in measurement update
        """
        # MPF parameters
        self.N = N
        self.dt = dt
        self.dt_mpf_meas_update = dt_mpf_meas_update
        self.v = v  # likelihood exponentiel parameter
        self.n_nonlin = len(mu_part)  # usually 3 (x, y, z error)
        self.n_lin = len(mu_kalman)   # usually 12
        
        self.count_est = 0

        # Full error-state estimate (3 nonlinear + 12 linear = 15 total)
        self.X = np.zeros((15, 1))
        self.P = np.diag(np.zeros(15))

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

    def getEstimate(self, u, X_nom, UAVKp, UAVDesc , closedLoop = False, predPerclosedLoop = 1 , UAVframe = None):
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
        self._predict(u, X_nom)


        if self.count_est % (round(self.dt_mpf_meas_update / self.dt)) == 0:

            # 2) Likelihood from DB (image matching)
            self._find_likelihood_particles(X_nom , UAVKp, UAVDesc, UAVframe)

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
        """
        # Create gaussian distributed particles around aircraft
        # shape: (3, N)
        mu_part_2d = np.tile(mu_part.reshape(-1, 1), (1, self.N))
        std_part_2d = np.tile(std_part.reshape(-1, 1), (1, self.N))
        self.particles = mu_part_2d + std_part_2d * np.random.randn(self.n_nonlin, self.N)

        # Initialize weights to 1/N
        self.weights = np.ones((self.N,)) / self.N

        # Create linear Kalman Filter states for each particle
        mu_kalman_2d = np.tile(mu_kalman.reshape(-1, 1), (1, self.N))  # shape (12, N)
        cov_kalman_3d = np.tile(cov_kalman[:, :, np.newaxis], (1, 1, self.N))  # shape (12,12,N)
        self.KalmanFiltersState = mu_kalman_2d
        self.KalmanFiltersCovariance = cov_kalman_3d
        
    def _predict(self, u, X_nom):
        """
        Propagate error states' kinematic equations based on IMU vector u (body accel & gyro).
        """
        if self.Accelerometer is None or self.Gyroscope is None:
            raise ValueError(
                "Accelerometer or Gyroscope objects have not been set. "
                "Please set them before calling `getEstimate`."
            )

        n = self.n_nonlin
        l = self.n_lin

        # IMU measurements from the array u
        # In the MATLAB code, the accelerometer reading is negated
        acc_m =  u[0]
        gyro_m = u[1]

        # Nominal IMU outputs = measured - biases (for gyro),
        # but note the sign for the accel bias in MATLAB code:
        # NOTE: ADD ERROR STATE BIAS TO THE NOMINAL VALUES
        acc_nom = acc_m  - X_nom[10:13]  - self.X[9:12].T.squeeze()
        # acc_nom[2] = 0  # if ignoring Z acceleration

        gyro_nom = gyro_m - X_nom[13:16] - self.X[12:15].T.squeeze()

        # Rotation from nominal quaternion
        rotM_nom = quat2rotm(X_nom[6:10])  # X_nom(7:10) in MATLAB

        # Create skew-symmetric of acc_nom (3x3)
        acc_nom_skew = np.array([
            [0.0,         -acc_nom[2],  acc_nom[1]],
            [acc_nom[2],   0.0,        -acc_nom[0]],
            [-acc_nom[1],  acc_nom[0],  0.0]
        ])

        # Nonlinear part: f_nonlin, A_nonlin
        f_nonlin = np.eye(n)
        # A_nonlin is shape (3, 12) in MATLAB usage
        # Because only the first (3) columns of linear states matter for coupling
        A_nonlin = np.hstack([np.eye(n) * self.dt, np.zeros((n, l - n))])

        # A_lin has 4 blocks of shape (3,3), making up (12,12).
        # The original code:
        # A_lin = [
        #   eye(3),         -rotM_nom*acc_nom_skew*self.dt,  -rotM_nom*self.dt,       zeros(3,3);
        #   zeros(3,3),     exp_rot(gyro_nom * self.dt)',                      zeros(3,3),             -eye(3)*self.dt;
        #   zeros(3,3),     zeros(3,3),                      eye(3),                 zeros(3,3);
        #   zeros(3,3),     zeros(3,3),                      zeros(3,3),             eye(3)
        # ];

        block_1 = np.hstack([np.eye(3)        , -rotM_nom @ acc_nom_skew * self.dt  , -rotM_nom * self.dt  ,  np.zeros((3, 3))])
        block_2 = np.hstack([np.zeros((3, 3)) ,  exp_rot(gyro_nom * self.dt).T      ,  np.zeros((3, 3))    , -np.eye(3) * self.dt])
        block_3 = np.hstack([np.zeros((3, 3)) ,  np.zeros((3, 3))                   ,  np.eye(3)           ,  np.zeros((3, 3))])
        block_4 = np.hstack([np.zeros((3, 3)) ,  np.zeros((3, 3))                   ,  np.zeros((3, 3))    ,  np.eye(3)])
 
        A_lin = np.vstack([block_1, block_2, block_3, block_4])
        f_lin = np.zeros((l,n))

        # The code sets 'C = zeros(1, l)' for measurement function but does not use it.
        # C = np.zeros((1, l))

        # Process noise terms for the nonlinear (Qn), linear (Ql), and cross-cov (Qnl)
        # Qn is a 3x3 for the particle position exploration
        noise_rate = 100 
        V = np.linalg.norm(X_nom[3:6])
        Qn = np.diag([V/noise_rate,V/noise_rate, 0.0001])  # originally commented: # Qn = diag([0 0 0]) => set as needed

        Qnl = np.zeros((n, l))
        # Ql: IMU noise cov terms for linear states
        # # For demonstration, we assume each is a float. Adapt as needed:
        # [accel_noise^2*dt(3x),
        #  gyro_noise^2*dt(3x),
        #  2*accel_bias_instab^2/3600*dt(3x),
        #  2*gyro_bias_instab^2/3600*dt(3x)]
        # Ql = np.diag(np.hstack([
        #                        (self.Accelerometer.NoiseDensity ** 2)                 * self.dt,      # (3,)
        #                        (self.Gyroscope.NoiseDensity  ** 2)                    * self.dt,      # (3,)
        #                        (2 * (self.Accelerometer.BiasInstability ** 2) / 3600) * self.dt,      # (3,)
        #                        (2 * (self.Gyroscope.BiasInstability  ** 2) / 3600)    * self.dt       # (3,)
        #                       ]))
        
        Ql = np.diag(np.hstack([
                               (self.Accelerometer.BiasInstability ** 2)                 * self.dt ** 2,      # (3,)
                               (self.Gyroscope.BiasInstability  ** 2)                    * self.dt ** 2,      # (3,)
                               (self.Accelerometer.NoiseDensity ** 2)                    * self.dt     ,      # (3,)
                               (self.Gyroscope.NoiseDensity  ** 2)                       * self.dt            # (3,)
                               ]))

        # Full augmented covariance Q = [[Qn, Qnl],[Qnl', Ql]] but we only use pieces
        # for the separate updates:
        # Q = np.block([
        #    [Qn, Qnl],
        #    [Qnl.T, Ql]
        # ])

        # Perform the per-particle propagation (MPF loop)
        c = 3  # used for the power 1/c in the code
        for i in range(self.N):
            # 1) Nonlinear state update
            particle_current = self.particles[:, i].copy()
            #   The code multiplies random noise by 0 => effectively no process noise
            #   If you want real process noise, remove the `* 0`:
            #   np.linalg.cholesky(...) or ( ... )**(1/c) might be intended.

            noise_part = (A_nonlin @ self.KalmanFiltersCovariance[:, :, i] @ A_nonlin.T + Qn)
            self.particles[:, i] = (
                                    f_nonlin @ particle_current              +
                                    A_nonlin @ self.KalmanFiltersState[:, i] +
                                    np.sign(noise_part)*(np.abs(noise_part))**(1 / c) @ np.random.randn(3)  # NOTE the * 0 DO NOT FORGET!!!
                                   )
            # If altitude is known => force 0 to the z-error:
            self.particles[2, i] = 0

            # 2) Linear state update
            #   The code's commented lines are left intact.  If you need them,
            #   remove the comments.
            A_lin_t = A_lin  # - Qnl'/Qn*A_nonlin
            Ql_t = Ql        # - Qnl'/Qn*Qnl

            # Nonlinear state covariance
            N_ = A_nonlin @ self.KalmanFiltersCovariance[:, :, i] @ A_nonlin.T + Qn
            L_ = A_lin_t  @ self.KalmanFiltersCovariance[:, :, i] @ A_nonlin.T @ np.linalg.inv(N_)
            Z_ = self.particles[:, i] - f_nonlin @ particle_current

            # Update each particle's linear KF state
            self.KalmanFiltersState[:, i] = (
                                            A_lin_t @ self.KalmanFiltersState[:, i]              + 
                                            L_ @ (Z_ - A_nonlin @ self.KalmanFiltersState[:, i]) +
                                            f_lin @ particle_current
                                            )

            # Zero out rotation error states (4:6 in MATLAB are indexes 3:6 in 1-based)
            self.KalmanFiltersState[3:6, i] = 0.0

            # Update each particle's linear covariance
            self.KalmanFiltersCovariance[:, :, i] = (
                                                    A_lin_t @ self.KalmanFiltersCovariance[:, :, i] @ A_lin_t.T +
                                                    Ql_t                                                        -
                                                    L_ @ N_ @ L_.T
                                                    )

    def _find_likelihood_particles(self, X_nom, UAVKp, UAVDesc, UAVFrame):
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
        
        # Hypothetical position of each particle => X_nom(1:3) + error
        pos_hypo =  X_nom[0:3].reshape(-1, 1) + self.particles  # shape (3, N)

        # Hypothetical orientation (assuming we only have yaw variation).
        # Our linear error states for rotation are KalmanFiltersState(4:6), 
        # so exponentiate them into a correction quaternion and multiply.
        # For now, the code sets them to zero, so effectively no orientation error.
        # But let's keep the logic to illustrate how you'd do it:
        q_dt = exp_quat(self.KalmanFiltersState[3:6, :])  # shape (4, N)
        qcor = quatmultiply(X_nom[6:10].T, q_dt.T)        # (N,4)
        rot_hypo = quat2eul(qcor).reshape(-1,3)           #(N,3)
        
        # Build the X_hypo array: shape (N, 2) for XY
        X_hypo = pos_hypo.T  # shape (N, 3)
        # We only pass X and Y plus the computed yaw to the DB scanner
        PartXYZ = X_hypo[:, 0:3]
        yaw = rot_hypo[:,0]

        # Now call the DB scanner to find likelihood for each particle
        # Expecting an array of length N back
        self.FramemostLikelihoodPart, numMatchedFeaturePart = self.DataBaseScanner.find_likelihood(UAVKp, UAVDesc, np.atleast_2d(PartXYZ), yaw, UAVFrame)
        
        # Likelihood calculation based on numMatchedFeaturePart with using logistic function
        self.likelihood = self._likelihood_func(numMatchedFeaturePart)
        
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

        # Linear states: shape (12, N)
        xl = self.KalmanFiltersState
        xl3d = xl.reshape(self.n_lin, 1, self.N)  # Shape: (12, 1, N)
        xl_est = np.sum(xl * self.weights, axis=1, keepdims=True) / np.sum(self.weights)  # Shape: (12, 1)
        # Cov for linear part:
        # "aux_cov = (xl - xl_est) .* (xl - xl_est)" then weighted + KF's own covariance
        diff_l = xl3d - xl_est[:, :, np.newaxis] # Shape: (12, 1, N)
        Xl_expected = diff_l * diff_l.transpose(1, 0, 2)
        
        weights_reshaped = self.weights.reshape(1, 1, self.N)  # Shape: (1, 1, N)
        Pl_est = np.sum((self.KalmanFiltersCovariance + Xl_expected) * weights_reshaped, axis=2) / np.sum(self.weights)  # Shape: (12, 12)

        # Combine into full 15D error-state
        self.X = np.hstack([xn_est, xl_est.reshape(-1)])

        # Combine the blocks into a 15x15
        # [ Pn_est_mat,    0
        #        0,     Pl_est ]
        self.P = np.block([
            [Pn_est_mat,                             np.zeros((self.n_nonlin, self.n_lin))],
            [np.zeros((self.n_lin, self.n_nonlin)),  Pl_est]
        ])
        
        if closedLoop and (self.predCount_bofore_closedLoop == predPerclosedLoop):
            # Closed-loop reset of states with mean removal
            self.particles          = self.particles - xn_est.reshape(-1, 1)
            self.KalmanFiltersState = self.KalmanFiltersState - xl_est
            
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

            # Resample from indexes
            self.particles               = self.particles[:, indices]
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
    
    def _likelihood_func(self,numMatchedFeaturePart):
        # Calculate likelihood value of each particles with given number of match
        # v :  hyper parameter of likelihood function, bigger value eliminite more particles even they relatively high number of matched feature
        # numMatchedFeaturePart : number of matched feature of each particles
        
        v = self.v 

        if not self.DataBaseScanner.useColorSimilarity:
        
            # # No likelihood update if numMatchedFeaturePart is less than 50
            # if np.max(numMatchedFeaturePart) <= 20:
            #     return np.ones(self.N)
            # else:
            
            w = 0
            numMatchedFeaturePart = np.array(numMatchedFeaturePart) + 1e-5 # Force to be numpy array and add small number for preventing devision zero 
            numMatchedFeaturePart = numMatchedFeaturePart / max(numMatchedFeaturePart) # Normalize number of matched point to [0,1]
            
            similarity = 0 + 1e-5
            similarity = similarity / (similarity) # Normalize number of matched point to [0,1]

            a = (1 / (1 + np.exp(-10 * (numMatchedFeaturePart - 0.5))) ** (1 / v)) / (1 / (1 + np.exp(-5)) ** (1 / v))
            b = (1 / (1 + np.exp(-10 * (similarity - 0.5))) ** (1 / v)) / (1 / (1 + np.exp(-5)) ** (1 / v))
            
            return a*(1-w) + b*w
            # return numMatchedFeaturePart
        
        else:
            numMatchedFeaturePart = np.array(numMatchedFeaturePart) + 1e-5 # Force to be numpy array and add small number for preventing devision zero 
            numMatchedFeaturePart = numMatchedFeaturePart / max(numMatchedFeaturePart) # Normalize number of matched point to [0,1]
            return (1 / (1 + np.exp(-10 * (numMatchedFeaturePart - 0.5))) ** (1 / v)) / (1 / (1 + np.exp(-5)) ** (1 / v))
