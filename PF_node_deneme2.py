import rclpy
import torch
import time
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from queue import Queue, Full, Empty
from concurrent.futures import ThreadPoolExecutor
from rclpy.qos import qos_profile_sensor_data
from threading import Lock
import cv2

# ROS2 messages
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header, Float32MultiArray

# Custom libraries
from StateEstimatorVINS import StateEstimatorMPF
from utils import *
from FeatureDetectorMatcher import FeatureDetectorMatcher
from DataBaseScanner import DatabaseScanner
from AerialImageModel import AerialImageModel
from OV.odom_subscriber import OdomAndMavrosSubscriber

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        
        # Callback groups for concurrent execution
        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()
        
        # --- GPU setup ---
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = True
        torch.set_num_threads(2)
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        
        # --- Initialize components ---
        self._initialize_components()
        
        # --- State variables ---
        self.state_lock = Lock()
        self.is_initialized = False
        self.last_meas_update_time = time.time()
        self.dt_meas_update = 10.0  # 10 seconds between measurement updates
        
        # VIO state
        self.vio_pos      = None
        self.vio_vel      = None
        self.vio_quat     = None
        self.vio_ang_vel  = None
        self.prev_vio_pos = None
        self.prev_time    = None
        
        # Camera image
        self.latest_camera_image = None
        self.camera_lock = Lock()
        
        # Measurement update queue (size=1, keep only latest)
        self.meas_queue = Queue(maxsize=1)

        # --- Thread pool for heavy computation ---
        self.pool = ThreadPoolExecutor(max_workers=4)
        self.measurement_future = None
        
        # --- Subscribers (non-blocking, reentrant) ---
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
            10,
            callback_group=self.reentrant_group
        )
        
        # --- Publishers ---
        self.pf_pose_pub = self.create_publisher(
            Odometry,
            '/pf/estimated_pose',
            100
        )
        
        # self.particles_pub = self.create_publisher(
        #     PoseArray,
        #     '/pf/particles',
        #     10
        # )
        
        # self.likelihood_pub = self.create_publisher(
        #     Float32MultiArray,
        #     '/pf/likelihood',
        #     10
        # )
        
        # --- Timer for prediction step (runs at high frequency) ---
        self.prediction_timer = self.create_timer(
            1/100,  # 100 Hz
            self._prediction_callback,
            callback_group=self.exclusive_group
        )
        
        # --- Timer to trigger measurement updates ---
        self.meas_trigger_timer = self.create_timer(
            1.0,  # Check every second
            self._check_measurement_trigger,
            callback_group=self.reentrant_group
        )
        
        self.get_logger().info('Particle Filter Node initialized')
        
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
        self.N = 10  # Number of particles
        self.v = 0.05  # Likelihood parameter
        
    def _vio_callback(self, msg: Odometry):
        """Lightweight VIO callback - just store the data"""
        should_initialize = False
        
        with self.state_lock:
            # Extract position
            self.vio_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # Extract orientation (quaternion)
            self.vio_quat = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ])
            
            # Extract velocity
            self.vio_vel = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            
            # Extract angular velocity
            self.vio_ang_vel = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            print('vio callback')
            
            # Check if we need to initialize (but don't do it while holding the lock)
            if not self.is_initialized and self.vio_pos is not None:
                should_initialize = True
        
        # Initialize PF outside the lock to avoid deadlock
        if should_initialize:
            print('vio init pf')
            self._initialize_pf()
                
    def _camera_callback(self, msg: Image):
        """Lightweight camera callback - just store the latest image"""
        
        print('cam callback')
        with self.camera_lock:
            # Convert ROS Image to numpy array
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            # Convert based on encoding type
            if encoding == "mono8" or encoding == "8UC1":
                self.latest_camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            elif encoding == "bgr8":
                self.latest_camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            elif encoding == "rgb8":
                self.latest_camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            elif encoding == "rgba8":
                self.latest_camera_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            elif encoding == "16UC1":
                self.latest_camera_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
            else:
                self.get_logger().warn(f'Unsupported encoding: {encoding}')
                self.latest_camera_image = None
                
    def _initialize_pf(self):
        """Initialize the particle filter"""

        with self.state_lock:
            
            euler_vio = quat2eul(self.vio_quat)
            
            mu_part = np.array([
                self.vio_pos[0],
                self.vio_pos[1],
                0,
                euler_vio[0]
            ])
            std_part = np.array([1, 1, 0, np.deg2rad(2)])
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
            
            self.prev_vio_pos = self.vio_pos.copy()
            self.prev_time = time.time()
            self.is_initialized = True
            
            self.get_logger().info('Particle Filter initialized')
            
    def _prediction_callback(self):
        """High-frequency prediction step"""
        if not self.is_initialized or self.mpf is None:
            return
        
        should_publish = False
        
        with self.state_lock:
            if self.vio_pos is None or self.prev_vio_pos is None:
                return
                
            print('pred')
            # Calculate velocity and dt
            current_time = time.time()
            dt = current_time - self.prev_time
            
            if dt > 0:
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
                should_publish = True
        
        # Publish outside the lock
        if should_publish:
            self._publish_estimate()
                
    def _check_measurement_trigger(self):
        """Check if it's time to trigger a measurement update"""
        
        print('meas trigger check')
        if not self.is_initialized or self.mpf is None:
            return
            
        current_time = time.time()
        
        # Check if enough time has passed since last measurement update
        if (current_time - self.last_meas_update_time) >= self.dt_meas_update:
            
            # Check if previous measurement is still processing
            if self.measurement_future is not None and not self.measurement_future.done():
                self.get_logger().warn('Previous measurement update still processing, skipping...')
                return
                
            # Get current camera image
            with self.camera_lock:
                if self.latest_camera_image is None:
                    self.get_logger().warn('No camera image available')
                    return
                camera_image = self.latest_camera_image.copy()
                
            # Get current VIO state
            with self.state_lock:
                vio_nom = np.hstack((self.vio_pos, self.vio_quat))
                vio_pos_copy = self.vio_pos.copy()
                
            # Submit measurement update to thread pool
            self.measurement_future = self.pool.submit(
                self._measurement_update_worker,
                camera_image,
                vio_nom,
                vio_pos_copy
            )
            
            self.last_meas_update_time = current_time
            self.get_logger().info('Triggered measurement update')
            
    def _measurement_update_worker(self, camera_image, vio_nom, vio_pos):
        """Heavy measurement update in separate thread"""
        try:
            start_time = time.time()
            print('meas update work')
            # Adjust snap dimension based on altitude and camera parameters
            altitude = -vio_pos[2]  # Get altitude from VIO NED position (D component is negative)
            snap_dim_value = int(((altitude / self.fx) * 2 * self.cx) * (1 / self.aim.mp))
            self.db_scanner.snapDim = (snap_dim_value, snap_dim_value)
            
            # Preprocess camera image
            frame = square_crop_from_center(camera_image)
            frame = rotate_image(frame, np.pi)
            frame = resize_image(frame, snapDim=self.db_scanner.snapDim)
            
            # Extract features
            _, uav_kp, uav_desc = self.feature_dm.detectFeatures(frame)
            
            # # Find likelihood (heavy computation)
            # with self.state_lock:
            #     particles_pos = self.mpf.particles[0:3, :].T
            #     particles_yaw = self.mpf.particles[3, :]
                
            # _, num_matched = self.db_scanner.find_likelihood(
            #     uav_kp, uav_desc, particles_pos, particles_yaw
            # )
            
            # # Calculate likelihood
            # likelihood = self.mpf._likelihood_func(num_matched)
            
            # # Update weights (quick)
            # with self.state_lock:
            #     self.mpf.likelihood = likelihood
            #     self.mpf._update_weights()
            #     self.mpf._estimate(closedLoop=False, predPerclosedLoop=1)
            #     self.mpf._resample()
            
            with self.state_lock:
                self.mpf._find_likelihood_particles(vio_nom, uav_kp, uav_desc)
                
            # Update weights (quick)
            with self.state_lock:
                self.mpf._update_weights()
                self.mpf._estimate(closedLoop=False, predPerclosedLoop=1)
                self.mpf._resample()

            elapsed = time.time() - start_time
            self.get_logger().info(f'Measurement update completed in {elapsed:.2f}s')
            
            # Publish updated estimate (outside the lock)
            self._publish_estimate()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Measurement update error: {str(e)}')
            return False
            
    def _publish_estimate(self):
        """Publish current PF estimate"""
        
        if not self.is_initialized or self.mpf is None:
            return
            
        print('publish estimate')

        with self.state_lock:
            # Publish estimated pose
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            
            pf_pos = self.mpf.X[0:3]
            odom_msg.pose.pose.position.x = float(pf_pos[0])
            odom_msg.pose.pose.position.y = float(pf_pos[1])
            odom_msg.pose.pose.position.z = float(pf_pos[2])
            
            self.pf_pose_pub.publish(odom_msg)
            
            # # Publish particles
            # particle_msg = PoseArray()
            # particle_msg.header = odom_msg.header
            
            # for i in range(self.mpf.N):
            #     pose = Pose()
            #     pose.position.x = float(self.mpf.particles[0, i])
            #     pose.position.y = float(self.mpf.particles[1, i])
            #     pose.position.z = float(self.mpf.particles[2, i])
            #     particle_msg.poses.append(pose)
                
            # self.particles_pub.publish(particle_msg)
            
            # # Publish likelihood
            # likelihood_msg = Float32MultiArray()
            # likelihood_msg.data = self.mpf.likelihood.tolist()
            # self.likelihood_pub.publish(likelihood_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ParticleFilterNode()
    executor = MultiThreadedExecutor(num_threads=5)
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