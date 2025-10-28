import numpy as np
import imageio as imio
import torch
from time import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock, RLock
import threading

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


class VIOProcessorNode(Node):
    """ROS2 Node for processing VIO data - LIGHTWEIGHT"""
    def __init__(self):
        super().__init__('vio_processor_node')
        
        # Use ReentrantCallbackGroup for non-blocking operation
        self.callback_group = ReentrantCallbackGroup()
        
        self.state_lock = RLock()
        self.last_print = time()
        
        # VIO state
        self.vio_pos = None
        self.vio_quat = None
        self.vio_vel = None
        self.vio_ang_vel = None
        
        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom_ned',
            self._vio_callback,
            10,  # Increased queue size
            callback_group=self.callback_group
        )
        
        self.get_logger().info('VIO processor node started')

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
        
        # Update state with lock (minimize lock time)
        with self.state_lock:
            self.vio_pos = vio_pos
            self.vio_quat = vio_quat
            self.vio_vel = vio_vel
            self.vio_ang_vel = vio_ang_vel
        
        if time() - self.last_print > 1.0:
            self.get_logger().info(f"VIO callback received - pos: {vio_pos[:2]}")
            self.last_print = time()


class ImageProcessorNode(Node):
    """ROS2 Node for heavy GPU-based image processing"""
    
    def __init__(self):
        super().__init__('image_processor_node')
        
        # Use ReentrantCallbackGroup for non-blocking callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # --- CV Bridge ---
        self.bridge = CvBridge()
        
        # --- GPU setup FIRST ---
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = True
        torch.set_num_threads(2)  # Limit CPU threads to reduce GIL contention
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        
        # Warm up CUDA in main thread
        if torch.cuda.is_available():
            dummy = torch.zeros(1, device=self.device)
            del dummy
            torch.cuda.synchronize()
        
        # --- Thread pool with only 1 worker for GPU work ---
        # More workers don't help for GPU-bound tasks and increase GIL contention
        self.pool = ThreadPoolExecutor(
            max_workers=1,
            initializer=_worker_initializer
        )
        
        # Track processing
        self.processing_future = None
        self.last_process_time = time()
        self.frame_count = 0
        self.last_fps_print = time()
        
        # --- Initialize components ---
        self._initialize_components()
        
        # --- ROS Subscriber ---
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,  # Reasonable queue size
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Image processor node started, subscribing to /camera/image_raw')
    
    def _initialize_components(self):
        """Initialize all processing components"""
        
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
        fx, fy, cx, cy = [635.4374739716663, 633.1552214084261, 486.7922140547102, 289.11649690690723]
        snapDim = (200, 200)
        gimballedCamera = False
        useGAN = False
        self.showFeatures = True
        self.showFrame = True
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
        
        # MPF State Estimator
        KLDsamplingFlag = False
        dt = 1/200
        dt_mpf_meas_update = 10
        N = 50
        v = 0.05
        mu_part = np.array([0, 0, 0, 0])
        std_part = np.array([1, 1, 0, np.deg2rad(2)])
        mu_kalman = None
        cov_kalman = None
        circular_var = [0, 0, 0, 1]
        gimballedCamera = False
        
        self.hStateEstimatorMPF = StateEstimatorMPF(
            N, mu_part, std_part, mu_kalman, cov_kalman, circular_var,
            dt, dt_mpf_meas_update, v, gimballedCamera, KLDsamplingFlag
        )
        self.hStateEstimatorMPF.DataBaseScanner = self.hDB
        
        snap_dim = (300, 300)
        self.hStateEstimatorMPF.DataBaseScanner.snapDim = snap_dim
        self.hUAVCamera.snapDim = self.hStateEstimatorMPF.DataBaseScanner.snapDim

    def _measurement_update_worker(self, received_image):
        """Heavy measurement update in separate thread"""
        try:
            # Ensure CUDA is set for this thread
            if torch.cuda.is_available():
                torch.cuda.set_device(0)
            
            # Process image
            UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = self.hUAVCamera.snapUAVImageLive(
                received_image,
                showFeatures=self.showFeatures,
                showFrame=self.showFrame
            )
            
            # Synchronize before timing
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            start = time()
            
            # Heavy GPU computation
            self.hStateEstimatorMPF._find_likelihood_particles(
                np.array([0, 0, 0, 1, 0, 0, 0]),  # pN, pE, pD, qW, qX, qY, qZ
                UAVKp,
                UAVDesc
            )
            
            # Synchronize after computation
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            
            elapsed = time() - start
            self.get_logger().debug(f"Measurement update took {elapsed:.4f} seconds")
            
        except Exception as e:
            self.get_logger().error(f"Error in measurement worker: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def image_callback(self, msg):
        """Lightweight callback - submit work to thread pool"""
        
        self.frame_count += 1
        
        # Print FPS every second
        current_time = time()
        if current_time - self.last_fps_print > 1.0:
            fps = self.frame_count / (current_time - self.last_fps_print)
            self.get_logger().info(f"Image callback rate: {fps:.1f} Hz")
            self.frame_count = 0
            self.last_fps_print = current_time
        
        # Check if previous processing is still running
        if self.processing_future is not None and not self.processing_future.done():
            # self.get_logger().warn('Previous frame still processing, skipping...')
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Submit to thread pool (non-blocking)
        self.processing_future = self.pool.submit(
            self._measurement_update_worker,
            cv_image
        )
        self.last_process_time = current_time

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down thread pool...')
        self.pool.shutdown(wait=True, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes
    image_processor_node = ImageProcessorNode()
    vio_processor_node = VIOProcessorNode()
    
    # OPTION 1: Single process with separate executors (RECOMMENDED)
    # This gives better isolation between lightweight and heavy nodes
    
    # Lightweight VIO node gets single-threaded executor
    vio_executor = SingleThreadedExecutor()
    vio_executor.add_node(vio_processor_node)
    
    # Heavy GPU node gets multi-threaded executor
    image_executor = MultiThreadedExecutor(num_threads=2)
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