import rclpy
import torch
import time
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# from queue import Queue, Full, Empty
from concurrent.futures import ThreadPoolExecutor
# from rclpy.qos import qos_profile_sensor_data
from threading import Lock, RLock
import imageio as imio

# ROS2 messages
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseArray, Pose
# from std_msgs.msg import Header, Float32MultiArray

# Custom libraries
from StateEstimatorVINS import StateEstimatorMPF
from utils import quat2eul, bodyRates2eulerRates, square_crop_from_center, rotate_image, resize_image
from FeatureDetectorMatcher import FeatureDetectorMatcher
from DataBaseScanner import DatabaseScanner
from AerialImageModel import AerialImageModel
from OV.odom_subscriber import OdomAndMavrosSubscriber

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        
        # Callback groups for concurrent execution
        self.reentrant_group = ReentrantCallbackGroup()
        self.prediction_group = MutuallyExclusiveCallbackGroup()
        self.meas_trigger_group = MutuallyExclusiveCallbackGroup()
        
        # # --- GPU setup ---
        # torch.set_grad_enabled(False)
        # torch.backends.cudnn.benchmark = True
        # torch.set_num_threads(2)
        # self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        # # self.device = torch.device('cpu')  # Force CPU for testing
        # print(f"ParticleFilterNode Using device: {self.device}")
        
        # # Warm up CUDA in main thread
        # if torch.cuda.is_available():
        #     dummy = torch.zeros(1, device=self.device)
        #     del dummy
        #     torch.cuda.synchronize()
        
        # --- Initialize components ---
        self._initialize_components()
        
        # --- State variables ---
        # Use RLock (reentrant) to allow same thread to acquire multiple times if needed
        self.state_lock = RLock()
        self.camera_lock = Lock()
        self.camera_image = np.zeros((960, 540), dtype=np.uint8)  # Default initialization
                
        self.is_initialized = False
        self.last_meas_update_time = time.time()
        self.dt_meas_update = 10  # 10 seconds between measurement updates
        
        # VIO state
        self.vio_pos      = None
        self.vio_vel      = None
        self.vio_quat     = None
        self.vio_ang_vel  = None
        self.prev_vio_pos = None
        self.prev_time    = None
        
        # Camera image
        self.latest_camera_image = None
        
        # --- Thread pool with initializer for CUDA context ---
        # # Use only 1-2 workers for GPU-heavy operations to avoid context switching
        # self.pool = ThreadPoolExecutor(
        #     max_workers=6,
        #     initializer=self._worker_initializer
        # )
        self.measurement_future = None
        
        # # --- Subscribers (non-blocking, reentrant) ---
        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom_ned',
            self._vio_callback,
            100,
            callback_group=self.reentrant_group
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._camera_callback,
            2,
            callback_group=self.reentrant_group
        )
        
        # --- Publishers ---
        self.pf_pose_pub = self.create_publisher(
            Odometry,
            '/pf/estimated_pose',
            100
        )
        
        # --- Timer for prediction step (runs at high frequency) ---
        # Use MutuallyExclusiveCallbackGroup to prevent concurrent prediction calls
        self.prediction_timer = self.create_timer(
            0.01,  # 100 Hz
            self._prediction_callback,
            callback_group=self.prediction_group
        )
        
        # --- Timer to trigger measurement updates ---
        # self.meas_trigger_timer = self.create_timer(
        #     1.0,  # Check every second
        #     self._check_measurement_trigger,
        #     callback_group=self.meas_trigger_group
        # )
        

        self._initialize_pf()
        self.get_logger().info('Particle Filter Node initialized')
    
    def _worker_initializer(self):
        """Initialize CUDA context in worker threads"""
        if torch.cuda.is_available():
            # Set the device for this worker thread
            torch.cuda.set_device(0)
            # Create a dummy tensor to initialize CUDA context
            dummy = torch.zeros(1, device='cuda:0')
            del dummy
            torch.cuda.synchronize()
        
    def _initialize_components(self):
        """Initialize PF components"""
        
        # Camera parameters
        self.fx = 635.4374739716663
        self.fy = 633.1552214084261
        self.cx = 486.7922140547102
        self.cy = 289.11649690690723
        
        # Feature detector/matcher
        detector_opt = {'type': 'XFEAT'}
        self.feature_dm = FeatureDetectorMatcher(detector_opt)
        
        # Aerial image database
        MAP = 'bacikoy'
        self.aim = AerialImageModel(MAP, FeatureDM=self.feature_dm, preFeatureFlag=True)
        
        # Database scanner
        snapDim = (200, 200)
        self.db_scanner = DatabaseScanner(
            FeatureDM=self.feature_dm,
            AIM=self.aim,
            snapDim=snapDim,
            showFeatures=False,
            showFrame=False,
            batch_mode=False
        )
        
        # Particle filter (will be initialized after first VIO message)
        self.mpf = None
        self.N = 50  # Number of particles
        self.v = 0.05  # Likelihood parameter
        
    def _vio_callback(self, msg: Odometry):
        """Lightweight VIO callback - just store the data"""
        
        # Extract all data first (no lock needed for reading msg)
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
        
        # Now update state with lock (minimize lock time)
        should_initialize = False
        with self.state_lock:
            self.vio_pos = vio_pos
            self.vio_quat = vio_quat
            self.vio_vel = vio_vel
            self.vio_ang_vel = vio_ang_vel
        
        # Check if we need to initialize
        if not self.is_initialized and self.vio_pos is not None:
            should_initialize = True
        
        # Initialize PF outside the lock
        # if should_initialize:
        #     self._initialize_pf()
                
    def _camera_callback(self, msg: Image):
        """Lightweight camera callback - just store the latest image"""
        
        # Convert ROS Image to numpy array (outside lock)
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == "mono8" or encoding == "8UC1":
            self.camera_image[:, :] = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        else:
            self.get_logger().warn(f'Unsupported encoding: {encoding}')
            return
        
        # Only acquire lock to store the image
        with self.camera_lock:
            self.latest_camera_image = self.camera_image

    def _initialize_pf(self):
        """Initialize the particle filter"""
        
        # with self.state_lock:
        # if self.is_initialized or self.vio_pos is None or self.vio_quat is None:
        #     return
            
        # euler_vio = quat2eul(self.vio_quat)
        
        # mu_part = np.array([
        #     self.vio_pos[0],
        #     self.vio_pos[1],
        #     0,
        #     euler_vio[0]
        # ])
        mu_part  = np.array([0,0,0,0]) # pN, pE, yaw
        std_part = np.array([1,1,0,np.deg2rad(2)])
        circular_var = [0, 0, 0, 1]
        
        self.mpf = StateEstimatorMPF(
            N=self.N,
            mu_part=mu_part,
            std_part=std_part,
            mu_kalman=None,
            cov_kalman=None,
            circular_var=circular_var,
            dt=0.01,
            dt_mpf_meas_update=self.dt_meas_update,
            v=self.v,
            gimballedCamera=False,
            KLDsamplingFlag=False
        )
        
        self.mpf.DataBaseScanner = self.db_scanner
        
        # self.prev_vio_pos = self.vio_pos.copy()
        self.prev_time = time.time()
        # self._initialize_pf = True
        self.is_initialized = True            
        self.get_logger().info('Particle Filter initialized')
            
    def _prediction_callback(self):
        """High-frequency prediction step"""
        # with self.state_lock:
        if not self.is_initialized or self.mpf is None:
            return
        
        if self.vio_pos is None or self.prev_vio_pos is None or self.vio_quat is None or self.vio_ang_vel is None:
            return
        
        # Calculate velocity and dt
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return
        
        # Compute velocity
        dP = self.vio_pos - self.prev_vio_pos
        vio_vel = dP / dt
        
        # Get Euler rates
        euler_vio = quat2eul(self.vio_quat)
        euler_dot = bodyRates2eulerRates(self.vio_ang_vel, euler_vio)
        
        # Particle input
        input_particle = [vio_vel[0], vio_vel[1], euler_dot[2]]
        
        # Update MPF dt
        self.mpf.dt = dt
        
        # Prediction step (lightweight)
        vio_nom = np.hstack((self.vio_pos, self.vio_quat))
        self.mpf._predict(input_particle, vio_nom)
        
        # Update previous values
        self.prev_vio_pos = self.vio_pos.copy()
        self.prev_time = current_time
        
        # Estimate (without measurement update)
        self.mpf._estimate(closedLoop=False, predPerclosedLoop=1)
        
        # Get estimate for publishing (copy while holding lock)
        pf_pos = self.mpf.X[0:3].copy()
        
        # Publish outside the lock
        self._publish_estimate(pf_pos)
                
    def _check_measurement_trigger(self):
        """Check if it's time to trigger a measurement update"""
        
        if not self.is_initialized:
            return
            
        current_time = time.time()
        
        # Check if enough time has passed since last measurement update
        if (current_time - self.last_meas_update_time) < self.dt_meas_update:
            return
        
        # Check if previous measurement is still processing
        if self.measurement_future is not None and not self.measurement_future.done():
            self.get_logger().warn('Previous measurement update still processing, skipping...')
            return
        
        # Get current camera image (with separate lock)
        # with self.camera_lock:
        # if self.latest_camera_image is None:
        #     self.get_logger().warn('No camera image available')
        #     return
        # camera_image = self.latest_camera_image.copy()
        
        # Get current VIO state and particle filter state snapshot (single lock acquisition)
        # with self.state_lock:
        # if self.mpf is None or self.vio_pos is None or self.vio_quat is None:
        #     return
                
        # # Create snapshot of all needed state at once
        # state_snapshot = {
        #     'vio_nom': np.hstack((self.vio_pos, self.vio_quat)),
        #     'vio_pos': self.vio_pos.copy(),
        #     'particles': self.mpf.particles.copy(),  # Copy particles if needed
        #     'altitude': -self.vio_pos[2]
        # }
        
        # Submit measurement update to thread pool
        # self.measurement_future = self.pool.submit(
        #     self._measurement_update_worker,
        #     camera_image,
        #     state_snapshot
        # )
        

        image0 = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0054_20251018_211221.jpg"
        d = 300
        im1_src = np.copy(imio.v2.imread(image0))
        im1_src = resize_image(im1_src, (d,d))    
        if im1_src.ndim == 2:  # grayscale -> add channel dimension
            im1 = np.copy(im1_src[..., np.newaxis])
        else:  # color -> keep same handling as im2 (reverse channels)
            im1 = np.copy(im1_src[..., ::-1])
        
        # Submit measurement update to thread pool
        self.measurement_future = self.pool.submit(
            self._measurement_update_worker,
            im1,
            np.array([0,0,0, 1,0,0,0]),  # pN, pE, pD, qW, qX, qY, qZ

        )

        self.last_meas_update_time = current_time
        self.get_logger().info('Triggered measurement update')
            
    def _measurement_update_worker(self, camera_image, state_snapshot):
        """Heavy measurement update in separate thread"""
        try:
            # start_time = time.time()
            
            # Ensure CUDA is available in this thread (should be set by initializer)
            if torch.cuda.is_available():
                print('TEST')
                torch.cuda.set_device(0)
            
            # Adjust snap dimension based on altitude
            # altitude = state_snapshot['altitude']
            altitude = 70.0  # Use fixed altitude for testing
            snap_dim_value = int(((altitude / self.fx) * 2 * self.cx) * (1 / self.aim.mp))
            
            # Note: db_scanner might not be thread-safe, consider making a copy or using locks
            # For now, we'll update snapDim (assuming it's safe)
            self.db_scanner.snapDim = (snap_dim_value, snap_dim_value)
            
            # Preprocess camera image (CPU operations)
            frame = square_crop_from_center(camera_image)
            frame = rotate_image(frame, np.pi)
            frame = resize_image(frame, snapDim=self.db_scanner.snapDim)
            
            # Extract features (GPU operations)
            _, uav_kp, uav_desc = self.feature_dm.detectFeatures(frame)
            
            # Find likelihood - this is the heavy GPU operation
            # Acquire lock only for accessing mpf
            # with self.state_lock:
            if self.mpf is None:
                return False
                
            # Call measurement update (this should handle its own GPU operations)

            # for i in range(50):

            torch.cuda.synchronize()

            start = time.time()
            self.mpf._find_likelihood_particles(
                state_snapshot, #['vio_nom'], 
                uav_kp, 
                uav_desc
            )
            end = time.time()
            torch.cuda.synchronize()
            print(f"measurement update takes: {end - start:.4f} seconds")

            
            # Update weights, estimate, and resample (quick operations)
            # self.mpf._update_weights()
            # self.mpf._estimate(closedLoop=False, predPerclosedLoop=1)
            # self.mpf._resample()
            
            # Get estimate for publishing
            # pf_pos = self.mpf.X[0:3].copy()
            
            # elapsed = time.time() - start_time
            # self.get_logger().info(f'Measurement update completed in {elapsed:.2f}s')
            
            # Publish updated estimate (outside the lock)
            # self._publish_estimate(pf_pos)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Measurement update error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
            
    def _publish_estimate(self, pf_pos=None):
        """Publish current PF estimate"""
        
        # If position not provided, get it from mpf
        if pf_pos is None:
            with self.state_lock:
                if not self.is_initialized or self.mpf is None:
                    return
            pf_pos = self.mpf.X[0:3].copy()
        
        # Publish (no lock needed, using copied data)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        
        odom_msg.pose.pose.position.x = float(pf_pos[0])
        odom_msg.pose.pose.position.y = float(pf_pos[1])
        odom_msg.pose.pose.position.z = float(pf_pos[2])
        
        self.pf_pose_pub.publish(odom_msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        # Shutdown thread pool
        self.pool.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ParticleFilterNode()
    executor = MultiThreadedExecutor()  # Increased for more concurrency
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()