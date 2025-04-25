import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import *


# ---------------------------------------------------------
# Helper Classes for IMU Specs (similar to accelparams, gyroparams)
# ---------------------------------------------------------
class AccelParams:
    """
    Stores accelerometer noise/bias parameters.
    """
    def __init__(self,
                 ConstantBias=0.0,
                 BiasInstability=0.0,
                 NoiseDensity=0.0,
                 RandomWalk = 0.0):
        """
        Parameters:
         - ConstantBias: in m/s^2
         - BiasInstability: random walk or bias stability in m/s^2
         - NoiseDensity: white noise density in m/s^2 / sqrt(Hz)
        """
        self.ConstantBias = ConstantBias
        self.BiasInstability = BiasInstability
        self.NoiseDensity = NoiseDensity
        self.RandomWalk = RandomWalk

class GyroParams:
    """
    Stores gyroscope noise/bias parameters.
    """
    def __init__(self,
                 ConstantBias=0.0,
                 BiasInstability=0.0,
                 NoiseDensity=0.0,
                 RandomWalk = 0.0):
        """
        Parameters:
         - ConstantBias: in rad/s
         - BiasInstability: random walk or bias stability in rad/s
         - NoiseDensity: white noise density in rad/s / sqrt(Hz)
        """
        self.ConstantBias = ConstantBias
        self.BiasInstability = BiasInstability
        self.NoiseDensity = NoiseDensity
        self.RandomWalk   = RandomWalk


# ---------------------------------------------------------
# IMUSensor: Python approximation of MATLAB imuSensor
# ---------------------------------------------------------
class IMUSensor:
    """
    Pythonic approximation of MATLAB's imuSensor('accel-gyro') object.
    This class is callable: IMUSensor(acc, gyro, rot_matrix).
    It outputs biased + noisy IMU readings in the body frame,
    given true accelerations and angular velocities in the inertial frame.
    """
    def __init__(self, ReferenceFrame='ENU', SampleRate=100.0):
        self.ReferenceFrame = ReferenceFrame
        self.SampleRate = SampleRate
        
        # Gravity sign convention
        if ReferenceFrame == 'NED':
            self.g = np.array([0.0, 0.0, 9.8022])  #[41.1010349, 29.0254601, 160]
        elif ReferenceFrame == 'ENU':
            self.g = np.array([0.0, 0.0, -9.8022])
        else:
            raise ValueError("ReferenceFrame must be 'NED' or 'ENU'.")
        
        # Default perfect IMU params
        self.Accelerometer = AccelParams()
        self.Gyroscope     = GyroParams()

        # These could be states if you want to model random-walk over time
        self.accumulatedAccelBias = 0.0
        self.Accb1 = 0 #Additive Noise by BiasInstability
        self.Accb2 = 0 #Additive Noise by NoiseDensity
        self.Accb3 = 0 #Additive Noise by RandomWalk
        
        self.accumulatedGyroBias  = 0.0
        self.Gyrob1 = 0
        self.Gyrob2 = 0
        self.Gyrob3 = 0
        
        # For reproducible random or controlled seeds:
        # self.rng = np.random.default_rng()

    def __call__(self, trueAcceleration, trueAngularVelocity, inertiaROTbody):
        """
        Generate IMU readings based on the true linear acceleration
        and true angular velocity in the inertial frame.

        Input:
          - trueAcceleration: shape (3,) in [m/s^2]
          - trueAngularVelocity: shape (3,) in [rad/s]
          - inertiaROTbody: 3x3 rotation matrix from inertial frame to body frame
                            (or body to inertial, depending on your sign conventions).
        
        Returns:
          (accReading_body, gyroReading_body)
        """
        # Convert from inertial to body if needed:
        # In your code, you do [accReading, gyroReading] = obj.IMU(acc, gyro, rotMatrix).
        # Here we assume the 'rotMatrix' is from inertial to body, so
        #   body_acc = R_ib @ (trueAcceleration - g)
        #   body_gyro= R_ib @ trueAngularVelocity
        # But your code sometimes negates acceleration, so check sign conventions carefully.

        # For now, do a direct multiply for the body-frame representation:
        body_acc = inertiaROTbody @ (trueAcceleration - self.g)
        body_gyro = inertiaROTbody @ trueAngularVelocity

        # Add constant bias
        body_acc  += self.Accelerometer.ConstantBias
        body_gyro += self.Gyroscope.ConstantBias

        # Add Bias Noise(BiasInstability,NoiseDensity,RandomWalk)
        self.Accb1 = self.Accb1/2 + np.random.normal(loc=0,scale=1,size = (3,)) * self.Accelerometer.BiasInstability
        self.Accb2 = np.random.normal(loc=0,scale=1,size = (3,)) * np.sqrt(self.SampleRate/2) * self.Accelerometer.NoiseDensity
        self.Accb3 = self.Accb3 + np.random.normal(loc=0,scale=1,size = (3,)) * (self.Accelerometer.RandomWalk/np.sqrt(self.SampleRate/2))
        
        self.Gyrob1 = self.Gyrob1/2 + np.random.normal(loc=0,scale=1,size = (3,)) * self.Gyroscope.BiasInstability
        self.Gyrob2 = np.random.normal(loc=0,scale=1,size = (3,)) * np.sqrt(self.SampleRate/2) * self.Gyroscope.NoiseDensity
        self.Gyrob3 = self.Gyrob3 + np.random.normal(loc=0,scale=1,size = (3,)) * (self.Gyroscope.RandomWalk/np.sqrt(self.SampleRate/2))
        
        body_acc  += self.Accb1 + self.Accb2 + self.Accb3
        body_gyro += self.Gyrob1 + self.Gyrob2 + self.Gyrob3

        # Return as row vectors to match MATLAB style
        return (body_acc, body_gyro)


# ---------------------------------------------------------
# INSbot Class
# ---------------------------------------------------------
class INSbot:
    """
    Python equivalent of the MATLAB INSbot class. 
    """

    def __init__(self, initialState, dt, ReferenceFrame='ENU', IMUtype=3):
        """
        Constructor for the INSbot class.

        Inputs:
         - initialState: array-like (16 x 1) or at least (10 x 1),
                         [position(3), velocity(3), quaternion(4), accelerometer bias(3), gyro bias(3)]
                         or your own format as needed.
         - dt: float, time step
         - ReferenceFrame: 'NED' or 'ENU'
         - IMUtype: 1 => Tactical grade, 2 => Commercial grade, 3 => Perfect IMU
        """
        # Gravity sign convention
        if ReferenceFrame == 'NED':
            self.g = np.array([0.0, 0.0, 9.8022])
        elif ReferenceFrame == 'ENU':
            self.g = np.array([0.0, 0.0, -9.8022])
        else:
            raise ValueError("ReferenceFrame must be 'NED' or 'ENU'.")

        self.ReferenceFrame = ReferenceFrame
        self.dt = dt
        self.imuFs = 1.0 / dt

        # Initialize nominal state
        initialState = np.array(initialState, dtype=float).flatten()
        self.NomState = initialState.copy()

        # True kinematic state: [pN, pE, pD, heading, pitch, roll]
        kinquat0 = quat2eul(initialState[6:10])
        self.TrueKinState = np.concatenate([initialState[0:3],kinquat0])  #pos,rot
        # True quaternion from initialState(7:10)
        self.TrueKinQuat  = initialState[6:10].copy()                     #quat

        # If you also store "TrueState" with 16 elements (like nominal),
        # initialize it similarly:
        self.TrueState = initialState.copy()

        # For IMU
        self.IMU      = IMUSensor(ReferenceFrame, SampleRate=self.imuFs)
        self.IMUTruth = IMUSensor(ReferenceFrame, SampleRate=self.imuFs)
        
        # Set up IMU parameters depending on IMUtype
        if IMUtype == 1:  # Tactical Grade
            # from your code:
            DataSheetAccel = AccelParams(
                ConstantBias=1.7*9.81e-4,
                BiasInstability=9.81e-6,
                NoiseDensity=1e-12
            )
            DataSheetGyro = GyroParams(
                ConstantBias=7*np.pi/180/3600,
                BiasInstability=1.45*1e-8,
                NoiseDensity=5.81*1e-7
            )
        elif IMUtype == 2:  # Commercial Grade
            DataSheetAccel = AccelParams(
                BiasInstability= np.array([60*abs(self.g[2])*1e-3, 60*abs(self.g[2])*1e-3, 80*abs(self.g[2])*1e-3 ]),
                NoiseDensity   = 300*abs(self.g[2])*1e-6 * np.ones(3)
            )
            DataSheetGyro = GyroParams(
                BiasInstability= 5*(np.pi/180) * np.ones(3),
                NoiseDensity   = 0.01*(np.pi/180) * np.ones(3)
            )
        else:
            # Perfect IMU (no noise, no bias)
            DataSheetAccel = AccelParams()
            DataSheetGyro  = GyroParams()

        # Assign to IMU
        self.IMU.Accelerometer = DataSheetAccel
        self.IMU.Gyroscope     = DataSheetGyro
        self.NomState[10:13] = self.IMU.Accelerometer.ConstantBias * np.ones(3)
        self.NomState[13:16] = self.IMU.Gyroscope.ConstantBias     * np.ones(3)

        # IMUTruth is perfect
        self.IMUTruth.Accelerometer = AccelParams()
        self.IMUTruth.Gyroscope     = GyroParams()

        # Initialize placeholders
        self.IMUreading      = [np.zeros(3), np.zeros(3)]
        self.IMUTruthreading = [np.zeros(3), np.zeros(3)]
        self.inertiaROTbody  = np.eye(3)
        self.prevTrueVelocity= np.concatenate([np.array(initialState[3:6]),np.zeros(3)])  # store [vel(3), angVel(3)] if needed
        
        # Counter for SimPerclosedLoop
        self.predCount_bofore_closedLoop = 0

        # Error state initilize states covariance matrix       
        self.P = np.eye(15) * 1e-3  # small initial uncertainty
        
        self.VO_R_meas = np.eye(3) * 0.05  # e.g., 5cm std 
        
        # print(f'INS initialized with initial state(pos,vel,quat,acc_bias,gyro_bias): {initialState}, ReferenceFrame={self.ReferenceFrame}, IMUtype={IMUtype}')
        print(f'INS initialized with initial states... \n [pos] : {initialState[0:3]},\n [vel] : {initialState[3:6]},\n [quat] : {initialState[6:10]},\n [acc_bias] : {initialState[10:13]},\n [gyro_bias] : {initialState[13:16]}')
        print(f'ReferenceFrame={self.ReferenceFrame}, IMUtype={IMUtype}')


    def move(self, u):
        """
        MOVE the UAV kinematic model based on the provided input.
        Inputs are:
          u[0] = velocity
          u[1] = bank angle  (roll)
        States are:
          [ pN, pE, pD, heading, flight path angle, bank angle ]
           idx 0    1    2     3           4               5
        """

        g_val = np.linalg.norm(self.g)  # 9.81
        velocity   = u[0]
        bank_angle = u[1]

        # For your custom approach: flight path angle is TrueKinState(5),
        # heading is TrueKinState(3)
        heading     = self.TrueKinState[3]
        flight_path = self.TrueKinState[4]

        ng = 1.0/np.cos(bank_angle)  # see your code

        # Kinematic model for UAV motion [pos,rot]
        dx = np.zeros(6)
        dx[0] =  velocity * np.cos(heading) * np.cos(flight_path)
        dx[1] =  velocity * np.sin(heading) * np.cos(flight_path)
        dx[2] = -velocity * np.sin(flight_path)
        dx[3] =  (g_val * ng / velocity) * (np.sin(bank_angle)/np.cos(flight_path))
        dx[4] =  (g_val / velocity) * (ng*np.cos(bank_angle) - np.cos(flight_path))
        dx[5] =  0.0

        # Inertia to body rotation matrix
        self.inertiaROTbody = quat2rotm(self.TrueKinQuat).T    # for ENU we do transpose(test it)

        # Euler rate to body rate matrix        
        angular_vel_inertia = np.array([dx[5], dx[4], dx[3]])  # [roll_rate, pitch_rate, yaw_rate]
        angular_vel_body    = self.inertiaROTbody @ angular_vel_inertia

        # Update quaternion with small rotation
        dq = exp_quat(angular_vel_body * self.dt)  # [w, x, y, z]
        self.TrueKinQuat = quatmultiply(self.TrueKinQuat,dq)
        # Convert new quaternion to euler angles
        eul = quat2eul(self.TrueKinQuat)
        self.TrueKinState[3:6] = eul

        # Positional dynamics
        self.TrueKinState[0:3] += dx[0:3] * self.dt

        # Post-move velocity in local inertial coords
        trans_vel_post = np.array([
             velocity*np.cos(self.TrueKinState[3])*np.cos(self.TrueKinState[4]),
             velocity*np.sin(self.TrueKinState[3])*np.cos(self.TrueKinState[4]),
            -velocity*np.sin(self.TrueKinState[4])
        ])

        # Update sensor readings
        # Combine linear and angular velocities into a 6-vector if you prefer
        self.updateSensorReadings(np.concatenate([trans_vel_post, angular_vel_inertia]))

    def moveINS(self, GroundTruthFlag = True):
        """
        moveINS: Propagate the Nominal INS States (biased/noisy IMU)
                 and the TrueState (perfect IMU).
        """
        # 1) Dead reckoning motion using the biased/noisy IMU readings
        RotM = quat2rotm(self.NomState[6:10]) #body to inertia
        
        # In MATLAB code: acc_m = -obj.IMUreading{1};
        acc_m  =  self.IMUreading[0]
        gyro_m =  self.IMUreading[1]

        acc_nom  = acc_m  - self.NomState[10:13]  #subtract bias
        gyro_nom = gyro_m - self.NomState[13:16]

        V = self.NomState[3:6].copy()
        #V[2] = 0.0  # if no vertical velocity
        self.NomState[0:3]  += self.dt * (V + 0.5*(RotM@acc_nom + self.g)*self.dt)
        self.NomState[3:6]  += self.dt * (RotM@acc_nom + self.g)
        self.NomState[6:10]  = quatmultiply(self.NomState[6:10], exp_quat(gyro_nom*self.dt))
        self.NomState[10:13] = self.NomState[10:13] # Acc biases
        self.NomState[13:16] = self.NomState[13:16] # Gyro biases

        if GroundTruthFlag:
            # 2) Ground Truth Move
            RotM_truth = quat2rotm(self.TrueState[6:10])
            acc_m_truth  = self.IMUTruthreading[0]
            gyro_m_truth = self.IMUTruthreading[1]

            acc_true  = acc_m_truth  - 0.0
            gyro_true = gyro_m_truth - 0.0

            Vt = self.TrueState[3:6]
            self.TrueState[0:3]  += self.dt*(Vt + 0.0*0.5*(RotM_truth@acc_true + self.g)*self.dt)
            self.TrueState[3:6]  += self.dt*(RotM_truth@acc_true + self.g)
            self.TrueState[6:10]  = quatmultiply(self.TrueState[6:10], exp_quat(gyro_true*self.dt))
            self.TrueState[10:13] = self.TrueState[10:13] # Acc biases
            self.TrueState[13:16] = self.TrueState[13:16] # Gyro biases

    def correctINS(self, dx, closedLoop = False, predPerclosedLoop = 1):
        """
        Correct the shifted INS with MPF error states solution 'dx'.
        'dx' is 15x1 or 16x1 error state: [pos(3), vel(3), rot(3), bias(3), bias(3)] ...
        For quaternion: we do multiplicative correction.
        """
        dx = np.array(dx, dtype=float).flatten()

        # Multiplicative correction on quaternion from error
        dq = exp_quat(dx[6:9])  # error in orientation
        qt = quatmultiply(self.NomState[6:10], dq)

        # "dx_noRot" excludes the orientation part
        dx_noRot   = np.concatenate([dx[0:6], dx[9:15]])
        Xnom_noRot = np.concatenate([self.NomState[0:6], self.NomState[10:16]])

        CorrectedState_noRot = Xnom_noRot + dx_noRot
        # Reassemble
        CorrectedState = np.concatenate([CorrectedState_noRot[0:6], qt, CorrectedState_noRot[6:12]])
        
        if closedLoop and (self.predCount_bofore_closedLoop == predPerclosedLoop):
            self.NomState = CorrectedState.copy()
            self.predCount_bofore_closedLoop = 0
        
        self.predCount_bofore_closedLoop += 1
        return CorrectedState

    def updateSensorReadings(self, trueVelocity):
        """
        Update IMU readings given translational and angular velocity from
        the UAV kinematic model in the inertial frame.
        trueVelocity: 6x1 array => [vel(3), angVel(3)] in inertial coords.
        """
        # True translational velocity
        trueTranslationalVelocity = trueVelocity[0:3]
        trueAngularVelocity       = trueVelocity[3:6]

        # Approx finite difference for acceleration
        # using self.prevTrueVelocity
        prevVel = self.prevTrueVelocity[0:3]
        trueAcceleration = (trueTranslationalVelocity - prevVel)/self.dt
        self.prevTrueVelocity = trueVelocity.copy()

        # Generate biased/noisy body-frame readings
        accelReading_body, gyroReading_body = self.IMU(trueAcceleration, trueAngularVelocity, self.inertiaROTbody)
        self.IMUreading = [accelReading_body, gyroReading_body]

        # Perfect IMU for ground truth
        accelReading_body_truth, gyroReading_body_truth = self.IMUTruth(trueAcceleration, trueAngularVelocity, self.inertiaROTbody)
        self.IMUTruthreading = [accelReading_body_truth, gyroReading_body_truth]

        # Now propagate the INS equations in moveINS
        self.moveINS()

    def predictIMU(self, bodyAcceleration, bodyAngularVelocity, useV0 = False):
        """
        Predict the nominal state using given body-frame IMU measurements.
        
        Inputs:
        - bodyAcceleration: (3x1) array of linear acceleration in the body frame [m/s^2]
        - bodyAngularVelocity: (3x1) array of angular velocity in the body frame [rad/s]
        """
        # Set IMU readings
        self.IMUreading      = [bodyAcceleration, bodyAngularVelocity]
        
        # Propagate the INS state using the biased/noisy IMU readings
        self.moveINS(GroundTruthFlag = False)
        if useV0:
            self.predictErrorState(bodyAcceleration, bodyAngularVelocity)
        
    def predictErrorState(self, acc_m, gyro_m):

        acc_nom = acc_m   - self.NomState[10:13]  
        gyro_nom = gyro_m - self.NomState[13:16] 

        # Rotation from nominal quaternion
        rotM_nom = quat2rotm(self.NomState[6:10])  # NomState(7:10) in MATLAB

        # Create skew-symmetric of acc_nom (3x3)
        acc_nom_skew = np.array([
            [0.0,         -acc_nom[2],  acc_nom[1]],
            [acc_nom[2],   0.0,        -acc_nom[0]],
            [-acc_nom[1],  acc_nom[0],  0.0]
        ])
        
        # EKF F_x matrix
        block_pos  = np.hstack([np.eye(3)      , np.eye(3) * self.dt ,  np.zeros((3,3))                   ,  np.zeros((3,3))    ,  np.zeros((3,3))])
        block_vel  = np.hstack([np.zeros((3,3)), np.eye(3)           , -rotM_nom @ acc_nom_skew * self.dt , -rotM_nom * self.dt ,  np.zeros((3,3))])
        block_ang  = np.hstack([np.zeros((3,3)), np.zeros((3, 3))    ,  exp_rot(gyro_nom * self.dt).T     ,  np.zeros((3,3))    , -np.eye(3) * self.dt])
        block_accb = np.hstack([np.zeros((3,3)), np.zeros((3, 3))    ,  np.zeros((3,3))                   ,  np.eye(3)          ,  np.zeros((3,3))])
        block_gyrb = np.hstack([np.zeros((3,3)), np.zeros((3, 3))    ,  np.zeros((3,3))                   ,  np.zeros((3,3))    ,  np.eye(3)])
        
        Fx = np.vstack([block_pos, block_vel, block_ang, block_accb, block_gyrb])
    
        # Process Noise    
        Q = np.zeros((12, 12))
        Q[0:3,   0:3]   = (self.IMU.Accelerometer.BiasInstability ** 2) * self.dt ** 2
        Q[3:6,   3:6]   = (self.IMU.Gyroscope.BiasInstability ** 2)     * self.dt ** 2
        Q[6:9,  6:9]  = (self.IMU.Accelerometer.NoiseDensity ** 2)    * self.dt 
        Q[9:12, 9:12] = (self.IMU.Gyroscope.NoiseDensity ** 2)        * self.dt 
        
        Fi = np.vstack([np.zeros((3,12)),  np.eye(12)])
        
        # Propagate covariance
        self.P = Fx @ self.P @ Fx.T + Fi @ Q @ Fi.T
        
    def VO_update(self, VO_pos):
        """
        Update the filter with a VO position measurement.
        
        Parameters:
          vo_position : measured position (3D numpy array)
        """
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)

        # Innovation (measurement residual)
        p_est = self.NomState[0:3]
        y = VO_pos - p_est
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.VO_R_meas
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Compute the error state update (15x1 vector)
        delta_x = K @ y
        
        # Covariance update (Joseph form for numerical consistency)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.VO_R_meas @ K.T
        
        # ----- Correct the nominal state -----
        # Rotation should be correct through quaternion producs, other states can be summed
        self.NomState[0:2]   += delta_x[0:2]
        self.NomState[3:5]   += delta_x[3:5]
        # self.NomState[6:10]   = quatmultiply(self.NomState[6:10], exp_quat(delta_x[6:9])) 
        # self.NomState[10:13] += delta_x[9:12]
        # self.NomState[13:16] += delta_x[12:15]