import numpy as np
import imageio as imio
import torch
from time import time
# from concurrent.futures import ThreadPoolExecutor
from threading import Lock, RLock
import threading
from utils import quat2eul, bodyRates2eulerRates, quat2rotm, generate_orthoprojection, eul2quat
from OV.utils_OV.common_utils import ned_VIO_converter, ned_SLAM_PC_converter
import os
import pymap3d as pm
import cv2
from datetime import datetime


# --- ROS2 ---
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image,PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2


# --- /CUSTOM ---
from StateEstimatorVINS import StateEstimatorMPF
from FeatureDetectorMatcher import FeatureDetectorMatcher
from UAVCamera import UAVCamera
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from Timer import Timer
from utils import resize_image, ned2px
from plotter import combineFrame


# def _worker_initializer():
#     """Initialize CUDA context in worker threads"""
#     if torch.cuda.is_available():
#         torch.cuda.set_device(0)
#         dummy = torch.zeros(1, device='cuda:0')
#         del dummy
#         torch.cuda.synchronize()


class SharedStateManager:
    """Thread-safe manager for shared StateEstimatorMPF and VIO data"""
    
    def __init__(self):
        self.lock = RLock()
        
        # VIO state
        self.vio_pos = None
        self.vio_quat = None
        self.vio_vel = None
        self.vio_ang_vel = None
        self.vio_timestamp = None
        
        # Yaw difference between VIO ref frame and ENU (for NED conversion)
        # This should be updated from odom_subscriber which computes it from GPS/magnetometer
        self.yaw_vioref2enu = 0.0

        # Params
        self.showFeatures = False
        self.showFrame    = True
        self.logVisualize = True
        self.logDate = datetime.now().strftime("%Y%m%d_%H%M%S")

        # StateEstimatorMPF instance
        self.state_estimator = None
        
        # Components needed for state estimator
        # self.hFeatureDM = None
        # self.hAIM       = None
        # self.hUAVCamera = None
        # self.hDB        = None
        

        #### Flight parameters
        MAP = 'catalca'
        LLA_leftupper = [41.336322, 28.489583, 0]
        LLA_home      = [41.332762, 28.494641, 0]
        leftupperNED = np.array(
            pm.geodetic2ned(LLA_leftupper[0], LLA_leftupper[1], LLA_leftupper[2],
                            LLA_home[0], LLA_home[1], LLA_home[2]),
            dtype=float
        ) + 0*np.array([5, -5, 0])
    

        # Feature detector/matcher
        # detector_opt = {'type': 'XFEAT'}
        detector_opt = None   # Use ZNCC matching if no detector/matcher specified
        self.hFeatureDM = FeatureDetectorMatcher(detector_opt=detector_opt)   # Use ZNCC matching if no detector/matcher specified


        # Aerial Image DataBase
        preFeatureFlag = True
        self.hAIM = AerialImageModel(MAP, FeatureDM=self.hFeatureDM, preFeatureFlag=preFeatureFlag)
        self.hAIM.leftupperNED = leftupperNED


        # UAV Camera
        # Camera parameters
        self.FlagOrthoprojection = True

        self.fx = 635.4374739716663
        self.fy = 633.1552214084261
        self.cx = 486.7922140547102
        self.cy = 289.11649690690723

        self.K = np.array([
                          [self.fx,  0,       self.cx],
                          [ 0,       self.fy, self.cy],
                          [ 0,       0,       1]
                          ], dtype=np.float32)

        self.distCoeffs = np.array([
                                    0.007912692192064455,      # k1
                                   -0.029678702245859702,      # k2
                                    6.955175056576606e-05,     # p1
                                    0.0006319922266392118,     # p2
                                    0.0                        # k3 (not in YAML, set to 0)
                                   ], dtype=np.float32)

        # Camera to IMU rotation
        self.R_ic =    np.array([[-0.0278616592128909   , -0.9995298103661786   , -0.01280180203159867],
                                 [-0.9996117282427698   ,  0.027854921448106484 ,  0.0007043512074339127],
                                 [-0.0003474268388132173,  0.012816455846735854 , -0.9999178054990925]])

        snapDim = (300, 300)   # NOTE: This will be updated dynamically based on altitude, but it will not be used if orthoprojection is used
        useGAN = False
        liveFlag = True
        self.hUAVCamera = UAVCamera(
            FeatureDM=self.hFeatureDM,
            snapDim=snapDim,
            cropFlag=True,
            resizeFlag=True,
            useGAN=useGAN,
            liveFlag=liveFlag
        )
        
        # Database Scanner
        batch_mode = False
        self.hDB = DatabaseScanner(
            FeatureDM=self.hFeatureDM,
            AIM=self.hAIM,
            snapDim=snapDim,
            showFeatures=self.showFeatures,
            showFrame=self.showFrame,
            batch_mode=batch_mode
        )

        # Initialization flag
        self.initialized = False

        # Measurement update interval
        self.dt_mpf_meas_update = 1.0   # seconds

    def initialize_components(self, logger, initial_vio_pos=None, initial_vio_quat=None):
        """Initialize all processing components (call once with first VIO state)"""
        # with self.lock:
        if self.initialized:
            logger.warn("Components already initialized, skipping...")
            return
        
        # MPF State Estimator - use initial VIO state if provided
        KLDsamplingFlag = False
        KLDparams = {'epsilon': 0.15, 'delta': 0.01, 'binSize': 3.00, 'nMax': 300,  'nMin': 50}
        dt = 1/100
        N = 250  #number of particles
        v = 0.2
        
        # Set particle mean based on first VIO state
        if initial_vio_pos is not None and initial_vio_quat is not None:
            # Extract yaw from quaternion
            eul = quat2eul(initial_vio_quat)
            mu_part = np.array([initial_vio_pos[0], initial_vio_pos[1], initial_vio_pos[2], eul[0]])
            logger.info(f"Initializing particles with VIO state: pos={initial_vio_pos[:2]}, yaw={np.rad2deg(eul[0]):.2f} deg")
        else:
            mu_part = np.array([0, 0, 0, 0])
            logger.warn("No initial VIO state provided, using zero mean for particles")
        
        std_part = np.array([1, 1, 0, np.deg2rad(1)])
        mu_kalman = None
        cov_kalman = None
        circular_var = [0, 0, 0, 1]
        gimballedCamera = False
        
        self.state_estimator = StateEstimatorMPF(
            N, mu_part, std_part, mu_kalman, cov_kalman, circular_var,
            dt, self.dt_mpf_meas_update, v, gimballedCamera, KLDsamplingFlag, KLDparams = KLDparams
        )
        self.state_estimator.DataBaseScanner = self.hDB
        
        snap_dim = (300, 300)  # Update snapDim for StateEstimatorMPF and UAVCamera
        self.state_estimator.DataBaseScanner.snapDim = snap_dim
        self.hUAVCamera.snapDim = snap_dim
        
        self.initialized = True
        logger.info("Shared components initialized successfully")
    
    def update_vio_state(self, pos, quat, vel, ang_vel):
        """Update VIO state from VIO node"""
        # with self.lock:
        self.vio_pos = pos
        self.vio_quat = quat
        self.vio_vel = vel
        self.vio_ang_vel = ang_vel
        self.vio_timestamp = time()
    
    def get_vio_state(self):
        """Get current VIO state"""
        # with self.lock:
        return {
            'pos': self.vio_pos.copy() if self.vio_pos is not None else None,
            'quat': self.vio_quat.copy() if self.vio_quat is not None else None,
            'vel': self.vio_vel.copy() if self.vio_vel is not None else None,
            'ang_vel': self.vio_ang_vel.copy() if self.vio_ang_vel is not None else None,
            'timestamp': self.vio_timestamp
        }
    
    def get_nominal_state(self):
        """Construct nominal state vector for StateEstimatorMPF"""
        # with self.lock:
        if self.vio_pos is None or self.vio_quat is None:
            return None
        
        # Construct Xnom: [pos(3), quat(4)]
        Xnom = np.concatenate([
            self.vio_pos.copy(),
            self.vio_quat.copy()
        ])
        return Xnom
    
    def is_initialized(self):
        """Check if components are initialized"""
        with self.lock:
            return self.initialized


class VIOProcessorNode(Node):
    """ROS2 Node for processing VIO data - LIGHTWEIGHT"""
    def __init__(self, shared_state: SharedStateManager):
        super().__init__('vio_processor_node')
        
        self.shared_state = shared_state
        
        # No callback group needed - use default (MutuallyExclusiveCallbackGroup)
        
        self.last_print = time()
        self.first_vio_received = False
        
        # Track previous state for velocity computation
        self.prev_vio_pos = None
        self.prev_time = None

        # --- ROS Subscriber ---
        # VIO NED Odometry subscriber        
        self.vio_ned_sub = self.create_subscription(
            Odometry,
            'vio/odom_ned',   # NOTE: Change the topic name to vio/odom_ned
            self._vio_ned_callback,
            qos_profile_sensor_data
        )

        # VIO odometry subscriber for orthoprojection
        self.shared_state.t_vio = None    # imu/cam translation on VIO global frame
        self.shared_state.R_vio = None    # imu rotation on VIO global frame
        self.vio_sub = self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self._vio_callback,
            qos_profile_sensor_data
        )

        # --- OpenVINS Slam Features --- 
        self.shared_state.vio_SLAM_PC     = None       
        # self.vio_SLAM_PC_num = 0
        self.create_subscription(
            PointCloud2,
            '/ov_msckf/points_slam',
            self._vio_SLAM_PC_callback,
            10)
        
        # Publisher for particle filter position estimate
        self.pf_pos_pub = self.create_publisher(
            PointStamped,
            '/pf/pos_estimate',
            10
        )

        # Track publishing rate for pf_pos_pub
        self.pf_pos_pub_count = 0
        self.pf_pos_pub_last_log_time = time()
        
        # Publisher for all particle positions
        # self.pf_particles_pub = self.create_publisher(
        #     PoseArray,
        #     '/pf/particles',
        #     10
        # )
        
        self.get_logger().info('VIO processor node started')

    def _vio_ned_callback(self, msg: Odometry):
        """VIO NED callback - store data, initialize if needed, and predict"""
        
        # Extract all data
        vio_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        vio_quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        
        vio_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        vio_ang_vel = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # Initialize components on first VIO callback
        if not self.first_vio_received:
            self.get_logger().info("First VIO message received, initializing components...")
            self.shared_state.initialize_components(
                self.get_logger(),
                initial_vio_pos=vio_pos,
                initial_vio_quat=vio_quat
            )
            self.first_vio_received = True
            self.prev_vio_pos = vio_pos.copy()
            self.prev_time = time()

            return  # Skip prediction on first message
        
        # Update shared state
        self.shared_state.update_vio_state(vio_pos, vio_quat, vio_vel, vio_ang_vel)
        
        # Perform prediction step if initialized
        if self.shared_state.is_initialized() and self.prev_vio_pos is not None:
            current_time = time()
            dt = current_time - self.prev_time
            
            if dt > 0:
                # Compute velocity from position difference
                dP = vio_pos - self.prev_vio_pos
                computed_vel = dP / dt
                
                # Get Euler angles and rates
                euler_vio = quat2eul(vio_quat)
                euler_dot = bodyRates2eulerRates(vio_ang_vel, euler_vio)
                
                # Input for particle prediction: [vx, vy, yaw_rate]
                input_particle = [computed_vel[0], computed_vel[1], euler_dot[2]]
                
                # Nominal state vector
                vio_nom = np.hstack((vio_pos, vio_quat))
                
                # Update dt and predict
                self.shared_state.state_estimator.dt = dt
                self.shared_state.state_estimator._predict(input_particle, vio_nom)
                
                # Estimate state (without measurement update)
                self.shared_state.state_estimator._estimate(closedLoop=False, predPerclosedLoop=1)
                
                # Publish particle filter position estimate
                self._publish_pf_position()
                
                # Update previous values
                self.prev_vio_pos = vio_pos.copy()
                self.prev_time = current_time
        
        # if time() - self.last_print > 1.0:
        #     self.get_logger().info(f"VIO callback received - pos: {vio_pos[:2]}")
        #     self.last_print = time()
    def _vio_callback(self, msg: Odometry):
        """VIO callback for orthoprojection processsing"""
        # This callback can be used to extract data needed for orthoprojection
        

        # --- pose ---
        px, py, pz = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        self.shared_state.t_vio = np.array([px, py, pz])
        self.shared_state.R_vio = quat2rotm([qw, qx, qy, qz])

    def _vio_SLAM_PC_callback(self, msg):
        """Callback for OpenVINS SLAM PointCloud2 messages"""

        # Process the PointCloud2 message

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True,
            reshape_organized_cloud = True
        )

        # Store the point cloud and count
        self.shared_state.vio_SLAM_PC     = points
        # self.SLAM_PC_num = points.shape[0]
        
        # # Check for VIO divergence
        # self._check_vio_divergence()

    def _publish_pf_position(self):
        """Publish particle filter 2D position estimate"""
        if self.shared_state.state_estimator is None:
            return
        
        # Get particle filter state estimate (direct state, not error state)
        pf_state = self.shared_state.state_estimator.X  # Shape: (4,) -> [x, y, z, yaw]

        # Get VIO position for z
        vio_state = self.shared_state.get_vio_state()
        if vio_state['pos'] is not None:
            pf_state[2] = vio_state['pos'][2]
        
        # Create PointStamped message
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom_ned'
        
        # Publish position estimate
        msg.point.x = float(pf_state[0])
        msg.point.y = float(pf_state[1])
        msg.point.z = float(pf_state[2])
        
        self.pf_pos_pub.publish(msg)
        
        # Track publishing rate
        self.pf_pos_pub_count += 1
        current_time = time()
        time_elapsed = current_time - self.pf_pos_pub_last_log_time
        
        # if time_elapsed >= 1.0:
        #     pub_rate = self.pf_pos_pub_count / time_elapsed
        #     self.get_logger().info(f"PF position publisher rate: {pub_rate:.2f} Hz")
        #     self.pf_pos_pub_count = 0
        #     self.pf_pos_pub_last_log_time = current_time
        
        # Publish all particles
        # self._publish_particles()
    
    def _publish_particles(self):
        """Publish all particle positions as PoseArray"""
        if self.shared_state.state_estimator is None:
            return
        
        # Get all particles (shape: 4 x N)
        particles = self.shared_state.state_estimator.particles  # [x, y, z, yaw] x N
        
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'odom_ned'
        
        # Get VIO z position for all particles
        vio_state = self.shared_state.get_vio_state()
        z_val = vio_state['pos'][2] if vio_state['pos'] is not None else 0.0
        
        # Convert each particle to a Pose
        N = particles.shape[1]
        for i in range(N):
            pose = Pose()
            pose.position.x = float(particles[0, i])
            pose.position.y = float(particles[1, i])
            pose.position.z = float(z_val)
            
            # No orientation for 2D particles, just set identity quaternion
            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            
            pose_array.poses.append(pose)

        # Publish particle poses
        # self.pf_particles_pub.publish(pose_array)


class ImageProcessorNode(Node):
    """ROS2 Node for heavy GPU-based image processing"""
    
    def __init__(self, shared_state: SharedStateManager):
        super().__init__('image_processor_node')
        
        self.shared_state = shared_state
        
        # Use MutuallyExclusiveCallbackGroup to prevent concurrent execution
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # --- CV Bridge ---
        self.bridge = CvBridge()
        
        # --- GPU setup FIRST ---
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = True
        torch.set_num_threads(6)
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Warm up CUDA in main thread
        if torch.cuda.is_available():
            dummy = torch.zeros(1, device=self.device)
            del dummy
            torch.cuda.synchronize()
        
        # --- Thread pool with only 1 worker for GPU work ---
        # self.pool = ThreadPoolExecutor(
        #     max_workers=1,
        #     initializer=_worker_initializer
        # )
        
        # Track processing
        # self.processing_future = None
        self.processing_in_progress = False
        self.frame_count = 0
        self.last_fps_print = time()
        self.dt_meas_update = self.shared_state.dt_mpf_meas_update
        self.last_meas_update_time = time()
        self.velTreshPassed = False
        self.last_warn_time = time()
        
        # self.showFeatures = False
        # self.showFrame = False
        
        # Don't initialize components here - wait for first VIO callback
        
        # --- ROS Subscriber ---
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f'Image processor node started, measurement update every {self.dt_meas_update}s')
        # self.get_logger().info(f'Image processor node started, measurement update will be done ASAP when altitude > 50m')


    def _measurement_update_worker(self, received_image, Xnom):
        """Heavy measurement update - runs directly in callback thread"""
        try:
            # Ensure CUDA is set for this thread
            if torch.cuda.is_available():
                torch.cuda.set_device(0)

            # Get altitude from nominal state to update snap dimension
            altitude = np.abs(Xnom[2])  # Absolute value since NED z is down
            snap_dim_value = int(((altitude / self.shared_state.fx) * 2 * self.shared_state.cx) * (1 / self.shared_state.hAIM.mp))

            self.shared_state.state_estimator.DataBaseScanner.snapDim = (snap_dim_value, snap_dim_value)
            self.shared_state.hUAVCamera.snapDim                      = (snap_dim_value, snap_dim_value)

            # Process image
            # Generate orthoprojection if enabled
            MaskOrthography = None
            if self.shared_state.FlagOrthoprojection:
                with Timer("Orthoprojection generation"):
                    # # Convert VIO frame inputs to NED frame
                    t_vio = self.shared_state.t_vio
                    t_ned =  self.shared_state.state_estimator.X[0:3] # NOTE: or use vio/ned messsage  Xnom[0:3]
                    t_enu = np.array([t_ned[1], t_ned[0], -t_ned[2]])
                    psi_diff = np.arctan2(t_enu[0]*t_vio[1] - t_enu[1]*t_vio[0], t_enu[0]*t_vio[0] + t_enu[1]*t_vio[1])
                    R_vio_enu = np.array([                                          # Rotation from enu to vio
                                    [ np.cos(psi_diff), -np.sin(psi_diff), 0],
                                    [ np.sin(psi_diff),  np.cos(psi_diff), 0],
                                    [           0,            0, 1]
                                ])
                    R_enu_c = R_vio_enu.T @ self.shared_state.R_vio @ self.shared_state.R_ic
                    SLAM_PC_enu = self.shared_state.vio_SLAM_PC @ R_vio_enu

                    ortho_img, MaskOrthography, PartCorrection_ENU, _, _ = generate_orthoprojection(
                                                                            received_image,
                                                                            self.shared_state.K,
                                                                            self.shared_state.distCoeffs,
                                                                            R_enu_c,
                                                                            t_enu,
                                                                            SLAM_PC_enu,
                                                                            resolution=self.shared_state.hAIM.mp,
                                                                            flagENU = True) 
                UAVFrame = ortho_img
                PartCorrection_NED = np.array([PartCorrection_ENU[1], PartCorrection_ENU[0], -PartCorrection_ENU[2]])  # Convert ENU correction to NED #NOTE: USE THIS PARTCORRECTION IN LIKELIHOOD IF NEEDED

            # If TemplateMatchingFlag is False, extract features using UAVCamera
            UAVKp = None
            UAVDesc = None
            if not self.shared_state.hFeatureDM.TemplateMatchingFlag:
                # Snap UAV image and extract features
                UAVFrame, _, UAVKp, UAVDesc = self.shared_state.hUAVCamera.snapUAVImageLive(
                    UAVFrame,
                    showFeatures=self.shared_state.showFeatures,
                    showFrame=self.shared_state.showFrame,
                    preprocessFlag = not self.shared_state.FlagOrthoprojection   # If orthoprojection is used, no need to crop/resize again
                )
            
            # Synchronize before timing
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            start = time()
            
            # Heavy GPU computation using shared state estimator
            self.shared_state.state_estimator._find_likelihood_particles(
                Xnom,
                UAVKp=UAVKp,
                UAVDesc=UAVDesc,
                UAVFrame=UAVFrame,
                MaskOrthography = MaskOrthography,
                PartCorrection = PartCorrection 
            )
            
            # Update weights and resample
            self.shared_state.state_estimator._update_weights()
            self.shared_state.state_estimator._resample()
            
            # Synchronize after computation
            if torch.cuda.is_available():
                torch.cuda.synchronize()

            if self.shared_state.logVisualize:

                with Timer("Visualization and saving"):
                    # Get most likelihood part view
                    PartFrame = self.shared_state.state_estimator.FramemostLikelihoodPart

                    # Save UAVFrame and PartFrame side by side for visualization
                    UAV_Part_frame = np.hstack((UAVFrame, PartFrame))
                    UAV_Part_frame = np.stack((UAV_Part_frame,)*3, axis=-1) if len(UAV_Part_frame.shape) == 2 else UAV_Part_frame  # Convert UAV_Part_frame to rgb for visualization

                    score = self.shared_state.state_estimator.DataBaseScanner.partInfo['maxScore']

                    # cv2 = __import__('cv2')  # Import here to avoid top-level dependency
                    cv2.putText(UAV_Part_frame, f'max Score: {score:.4f}',(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

                    particlesPos = self.shared_state.state_estimator.particles[0:3, :] # shape 3,N
                    GT_pos = self.shared_state.vio_pos.copy().reshape(1,3)  # shape 3,1
                    pxGT  = ned2px(GT_pos.copy()        , self.shared_state.hAIM.leftupperNED, self.shared_state.hAIM.mp, self.shared_state.hDB.pxRned).squeeze()
                    pxPF  = ned2px(particlesPos.T.copy(), self.shared_state.hAIM.leftupperNED, self.shared_state.hAIM.mp, self.shared_state.hDB.pxRned)   
                    pxPF_with_weights = np.hstack((pxPF , self.shared_state.state_estimator.weights.reshape(-1, 1)))

                    combinedFrame = combineFrame(self.shared_state.hAIM.Igray, pxGT, None, pxPF_with_weights, resize_dim = UAVFrame.shape[::-1])
                    final_frame = np.hstack((UAV_Part_frame, combinedFrame))
                    
                    # create ImageMatch_output directory if it doesn't exist
                    timestamp = time()
                    log_dir = f'logs_{self.shared_state.logDate}'
                    if not os.path.exists('ImageMatch_output/' + log_dir):
                        os.makedirs('ImageMatch_output/' + log_dir)
                    imio.imwrite(f'ImageMatch_output/{log_dir}/measurement_update_{timestamp:.4f}.png', final_frame)

            # Visualize results by combining AIM image, GT projection, and PF projection with weights

            elapsed = time() - start
            # self.get_logger().info(f"Measurement update completed in {elapsed:.4f} seconds")
            print(f"Measurement update completed in {elapsed:.4f} seconds")
            
        except Exception as e:
            self.get_logger().error(f"Error in measurement worker: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.processing_in_progress = False

    def image_callback(self, msg):
        """Image callback - check timing and trigger measurement update if needed"""
        
        current_time = time()


        # self.frame_count += 1
        # # Print FPS every second
        # if current_time - self.last_fps_print > 1.0:
        #     fps = self.frame_count / (current_time - self.last_fps_print)
        #     self.get_logger().info(f"Image callback rate: {fps:.1f} Hz")
        #     self.frame_count = 0
        #     self.last_fps_print = current_time
        
        # Skip if not initialized yet
        if not self.shared_state.is_initialized():
            return
        
        # Get VIO state first to check altitude
        vio_state = self.shared_state.get_vio_state()
        if vio_state['pos'] is None:
            self.get_logger().warn("No VIO state available for measurement update")
            return
        
        # Check if altitude is greater than 50 meters
        altitude = -(vio_state['pos'][2])  # Absolute value since NED z is down
        if altitude <= 50.0:
            # Skip measurement update if altitude is too low
            if current_time - self.last_warn_time > 1.0:
                self.get_logger().info(f"Altitude {altitude:.1f}m too low for measurement update, skipping...")
                self.last_warn_time = current_time
            return
        
        # Check if enough time has passed for measurement update
        if (current_time - self.last_meas_update_time) < self.dt_meas_update:
            return
        
        # Check if previous measurement is still processing
        if self.processing_in_progress:
            self.get_logger().warn('Previous measurement update still processing, skipping...')
            return
        
        Xnom = self.shared_state.get_nominal_state()
        if Xnom is None:
            self.get_logger().warn("Cannot construct nominal state for measurement update")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Mark processing as in progress
        self.processing_in_progress = True
        self.last_meas_update_time = current_time
        
        # Run measurement update directly in this thread (no pool submission)
        self.get_logger().info(f'Triggering measurement update at altitude: {altitude:.1f}m...')
        self._measurement_update_worker(cv_image, Xnom)
        
        # # Submit measurement update to thread pool
        # self.get_logger().info('Triggering measurement update...')
        # self.processing_future = self.pool.submit(
        #     self._measurement_update_worker,
        #     cv_image,
        #     Xnom
        # )
        # self.last_meas_update_time = current_time

    def destroy_node(self):
        """Cleanup on shutdown"""
        # self.get_logger().info('Shutting down thread pool...')
        # self.pool.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Create shared state manager
    shared_state = SharedStateManager()
    
    # Create nodes with shared state
    image_processor_node = ImageProcessorNode(shared_state)
    vio_processor_node = VIOProcessorNode(shared_state)
    
    # Lightweight VIO node gets single-threaded executor
    vio_executor = SingleThreadedExecutor()
    vio_executor.add_node(vio_processor_node)
    
    # Heavy GPU node gets multi-threaded executor
    image_executor = MultiThreadedExecutor()
    image_executor.add_node(image_processor_node)
    
    # Spin VIO in separate thread
    vio_thread = threading.Thread(
        target=vio_executor.spin,
        daemon=True
    )
    vio_thread.start()
    
    try:
        # Spin image processor in main thread
        image_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        vio_executor.shutdown()
        image_executor.shutdown()
        vio_processor_node.destroy_node()
        image_processor_node.destroy_node()
        rclpy.shutdown()


def main_separate_processes(args=None):
    """
    OPTION 2: Run nodes in completely separate processes (BEST PERFORMANCE)
    
    To use this approach:
    1. Create two separate Python files: vio_node.py and image_node.py
    2. Run them in separate terminals:
       Terminal 1: ros2 run your_package vio_node
       Terminal 2: ros2 run your_package image_node
    
    This completely eliminates GIL contention and gives true parallelism.
    """
    import sys
    
    rclpy.init(args=args)
    
    if len(sys.argv) > 1 and sys.argv[1] == 'vio':
        node = VIOProcessorNode()
        executor = SingleThreadedExecutor()
    else:  # Default to image processing
        node = ImageProcessorNode()
        executor = MultiThreadedExecutor(num_threads=2)
    
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Use OPTION 1 (single process, separate executors)
    main()
    
    # To use OPTION 2 (separate processes), run:
    # python your_script.py vio  (in terminal 1)
    # python your_script.py      (in terminal 2)