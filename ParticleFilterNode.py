import numpy as np
import imageio as imio
import torch
from time import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock, RLock
import threading
from utils import quat2eul, bodyRates2eulerRates


from utils import resize_image

# --- ROS2 ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
# --- /ROS2 ---

from StateEstimatorVINS import StateEstimatorMPF
from FeatureDetectorMatcher import FeatureDetectorMatcher
from UAVCamera import UAVCamera
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from Timer import Timer
import pymap3d as pm


def _worker_initializer():
    """Initialize CUDA context in worker threads"""
    if torch.cuda.is_available():
        torch.cuda.set_device(0)
        dummy = torch.zeros(1, device='cuda:0')
        del dummy
        torch.cuda.synchronize()


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
        
        # StateEstimatorMPF instance
        self.state_estimator = None
        
        # Components needed for state estimator
        # self.hFeatureDM = None
        # self.hAIM       = None
        # self.hUAVCamera = None
        # self.hDB        = None
        

        #### Flight parameters
        MAP = 'bacikoy'
        LLA_leftupper = [39.780238, 32.314440, 0]
        LLA_home = [39.7785834, 32.3158889, 0]
        leftupperNED = np.array(
            pm.geodetic2ned(LLA_leftupper[0], LLA_leftupper[1], LLA_leftupper[2],
                            LLA_home[0], LLA_home[1], LLA_home[2]),
            dtype=float
        ) + np.array([5, -8, 0])
        
        # Feature detector
        detector_opt = {'type': 'XFEAT'}
        self.hFeatureDM = FeatureDetectorMatcher(detector_opt=detector_opt)
        
        # Aerial Image DataBase
        preFeatureFlag = True
        self.hAIM = AerialImageModel(MAP, FeatureDM=self.hFeatureDM, preFeatureFlag=preFeatureFlag)
        self.hAIM.leftupperNED = leftupperNED
        
        # UAV Camera
        snapDim = (200, 200)
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
            showFeatures=True,
            showFrame=True,
            batch_mode=batch_mode
        )


        # Initialization flag
        self.initialized = False

    def initialize_components(self, logger, initial_vio_pos=None, initial_vio_quat=None):
        """Initialize all processing components (call once with first VIO state)"""
        # with self.lock:
        if self.initialized:
            logger.warn("Components already initialized, skipping...")
            return
        
        # MPF State Estimator - use initial VIO state if provided
        KLDsamplingFlag = False
        dt = 1/100
        dt_mpf_meas_update = 10
        N = 50
        v = 0.05
        
        # Set particle mean based on first VIO state
        if initial_vio_pos is not None and initial_vio_quat is not None:
            # Extract yaw from quaternion
            eul = quat2eul(initial_vio_quat)
            mu_part = np.array([initial_vio_pos[0], initial_vio_pos[1], initial_vio_pos[2], eul[0]])
            logger.info(f"Initializing particles with VIO state: pos={initial_vio_pos[:2]}, yaw={np.rad2deg(eul[0]):.2f} deg")
        else:
            mu_part = np.array([0, 0, 0, 0])
            logger.warn("No initial VIO state provided, using zero mean for particles")
        
        std_part = np.array([1, 1, 0, np.deg2rad(2)])
        mu_kalman = None
        cov_kalman = None
        circular_var = [0, 0, 0, 1]
        gimballedCamera = False
        
        self.state_estimator = StateEstimatorMPF(
            N, mu_part, std_part, mu_kalman, cov_kalman, circular_var,
            dt, dt_mpf_meas_update, v, gimballedCamera, KLDsamplingFlag
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
        
        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom_ned',
            self._vio_callback,
            10
        )
        
        self.get_logger().info('VIO processor node started')

    def _vio_callback(self, msg: Odometry):
        """VIO callback - store data, initialize if needed, and predict"""
        
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
                
                # Update previous values
                self.prev_vio_pos = vio_pos.copy()
                self.prev_time = current_time
        
        if time() - self.last_print > 1.0:
            self.get_logger().info(f"VIO callback received - pos: {vio_pos[:2]}")
            self.last_print = time()


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
        torch.set_num_threads(2)
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
        self.dt_meas_update = 10.0
        self.last_meas_update_time = time()
        
        self.showFeatures = False
        self.showFrame = False
        
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

    def _measurement_update_worker(self, received_image, Xnom):
        """Heavy measurement update - runs directly in callback thread"""
        try:
            # Ensure CUDA is set for this thread
            if torch.cuda.is_available():
                torch.cuda.set_device(0)
            
            # Process image
            UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = self.shared_state.hUAVCamera.snapUAVImageLive(
                received_image,
                showFeatures=self.showFeatures,
                showFrame=self.showFrame
            )
            
            # Synchronize before timing
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            start = time()
            
            # Heavy GPU computation using shared state estimator
            self.shared_state.state_estimator._find_likelihood_particles(
                Xnom,
                UAVKp,
                UAVDesc
            )
            
            # Update weights and resample
            self.shared_state.state_estimator._update_weights()
            self.shared_state.state_estimator._resample()
            
            # Synchronize after computation
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            elapsed = time() - start
            self.get_logger().info(f"Measurement update completed in {elapsed:.4f} seconds")
            
        except Exception as e:
            self.get_logger().error(f"Error in measurement worker: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.processing_in_progress = False

    def image_callback(self, msg):
        """Image callback - check timing and trigger measurement update if needed"""
        
        self.frame_count += 1
        
        # Print FPS every second
        current_time = time()
        if current_time - self.last_fps_print > 1.0:
            fps = self.frame_count / (current_time - self.last_fps_print)
            self.get_logger().info(f"Image callback rate: {fps:.1f} Hz")
            self.frame_count = 0
            self.last_fps_print = current_time
        
        # Skip if not initialized yet
        if not self.shared_state.is_initialized():
            return
        
        # Check if enough time has passed for measurement update
        if (current_time - self.last_meas_update_time) < self.dt_meas_update:
            return
        
        # Check if previous measurement is still processing
        # if self.processing_future is not None and not self.processing_future.done():
        if self.processing_in_progress:
            self.get_logger().warn('Previous measurement update still processing, skipping...')
            return
        
        # Get VIO state
        vio_state = self.shared_state.get_vio_state()
        if vio_state['pos'] is None:
            self.get_logger().warn("No VIO state available for measurement update")
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
        self.get_logger().info('Triggering measurement update...')
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