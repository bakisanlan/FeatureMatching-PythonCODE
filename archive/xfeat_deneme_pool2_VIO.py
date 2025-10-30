import numpy as np
import imageio as imio
# import os
import torch
from time import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock, RLock


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

aaa = 0  # Global sayaç değişkeni
def _worker_initializer():
    """Initialize CUDA context in worker threads"""
    global aaa
    aaa += 1
    if torch.cuda.is_available():
        # Set the device for this worker thread
        torch.cuda.set_device(0)
        # Create a dummy tensor to initialize CUDA context
        dummy = torch.zeros(1, device='cuda:0')
        del dummy
        torch.cuda.synchronize()

class VIOProcessorNode(Node):
    """ROS2 Node for processing VIO data and images asynchronously"""
    def __init__(self):
        super().__init__('vio_processor_node')

        self.state_lock = RLock()

        self.last_print = time()

        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom_ned',
            self._vio_callback,
            1
        )


    def _vio_callback(self, msg: Odometry):
        """Lightweight VIO callback - just store the data"""

        if time() - self.last_print > 1.0:
            self.get_logger().info("VIO callback received")
            self.last_print = time()

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
        # if not self.is_initialized and self.vio_pos is not None:
        #     should_initialize = True
        
        # Initialize PF outside the lock
        # if should_initialize:
        #     self._initialize_pf()


# class ImageProcessorNode(Node):

#     def __init__(self, ):
#         super().__init__('image_processor_node')
        
#         # --- CV Bridge ---
#         self.bridge = CvBridge()
        
#         # --- ROS Subscriber ---


#         self.image_sub = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # <-- DEĞİŞTİREBİLİRSİNİZ
#             self.image_callback,
#             1)

#         # # # --- Subscribers (non-blocking, reentrant) ---


#         self.get_logger().info('Image processor node started, subscribing to /camera/image_raw')

#         #### Flight parameters
#         ReferenceFrame = 'NED'  #Reference frame for the drone's local coordinate system
#         MAP            = 'bacikoy'  #Satallite map name
#         detector       = 'XFEAT'   #Feature detector type (SP: SuperPoint, ORB: Oriented FAST and Rotated Brief)
#         snapFrame      = True
#         IMUtype        = 2  #deal later
#         LLA_leftupper  = [39.780238,  32.314440, 0]  # LLA reference point (latitude, longitude) left upper corner of the map
#         LLA_home       = [39.7785834, 32.3158889,0] # Initial position of the drone in LLA (latitude, longitude, altitude)
#         leftupperNED   = np.array(pm.geodetic2ned(LLA_leftupper[0], LLA_leftupper[1], LLA_leftupper[2], LLA_home[0], LLA_home[1], LLA_home[2]), dtype=float) + np.array([5,-8,0])

#         detector_opt = {'type' : 'XFEAT'}
#         self.hFeatureDM = FeatureDetectorMatcher(detector_opt= detector_opt)

#         #### Aerial Image DataBase
#         preFeatureFlag = True
#         self.hAIM = AerialImageModel(MAP, FeatureDM = self.hFeatureDM, preFeatureFlag= preFeatureFlag)
#         self.hAIM.leftupperNED = leftupperNED

#         #### UAV Camera
#         fx, fy, cx, cy = [635.4374739716663, 633.1552214084261, 486.7922140547102, 289.11649690690723]  # 4mm lens
#         snapDim = (200,200) #deal later
#         gimballedCamera       = False
#         useGAN                = False
#         self.showFeatures     = True  # Sınıf değişkeni yapıldı
#         self.showFrame        = True  # Sınıf değişkeni yapıldı
#         liveFlag              = True  
#         self.hUAVCamera = UAVCamera(FeatureDM = self.hFeatureDM, snapDim = snapDim, cropFlag = True, 
#                                resizeFlag = True, useGAN = useGAN, liveFlag = liveFlag)

#         #### Database Scanner
#         batch_mode = False
#         self.hDB = DatabaseScanner(FeatureDM = self.hFeatureDM, AIM=self.hAIM, snapDim=snapDim, 
#                               showFeatures= self.showFeatures, showFrame= self.showFrame,
#                               batch_mode = batch_mode)

#         ### MPF State Esimator
#         useMPF          = True
#         KLDsamplingFlag = False
#         dt = 1/200  # NOTE: DEAL LATER!!! UPDATE IN WHILE LOOP
#         dt_mpf_meas_update = 10
#         N = 50
#         v = 0.05
#         mu_part  = np.array([0,0,0,0]) # pN, pE, yaw
#         std_part = np.array([1,1,0,np.deg2rad(2)])
#         mu_kalman  = None
#         cov_kalman = None
#         circular_var = [0,0,0,1]  # Circular variable for yaw
#         self.hStateEstimatorMPF = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman, circular_var,
#                                                         dt,dt_mpf_meas_update,v,gimballedCamera, KLDsamplingFlag)
#         self.hStateEstimatorMPF.DataBaseScanner = self.hDB
#         is_first_IM = False

#         snap_dim = (300, 300)
#         self.hStateEstimatorMPF.DataBaseScanner.snapDim = snap_dim
#         self.hUAVCamera.snapDim                         = self.hStateEstimatorMPF.DataBaseScanner.snapDim

#         # --- Statik Görüntü Yükleme Kısmı Kaldırıldı ---
#         # Artık görüntüler callback'ten gelecek

#         # --- GPU setup ---
#         torch.set_grad_enabled(False)
#         torch.backends.cudnn.benchmark = True
#         torch.set_num_threads(2)
#         self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
#         self.get_logger().info(f'Using device: {self.device}')

#         # Warm up CUDA in main thread
#         if torch.cuda.is_available():
#             dummy = torch.zeros(1, device=self.device)
#             del dummy
#             torch.cuda.synchronize()

#         self.pool = ThreadPoolExecutor(
#             max_workers=1,
#             initializer=_worker_initializer
#         )

#         # xfeat = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, top_k = 1300)
        
#         # Başlangıçta thread çalıştırma kaldırıldı, artık callback tetikleyecek
#         # measurement_future = pool.submit(...)


#     def _measurement_update_worker(self, received_image):
#         """Heavy measurement update in separate thread"""
        
#         # Gelen görüntüyü kullan
#         UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = self.hUAVCamera.snapUAVImageLive(
#             received_image, 
#             showFeatures = self.showFeatures, 
#             showFrame = self.showFrame
#         )

#         # n = 5
#         # x2 = torch.randn(n,3,500,500)

#         torch.cuda.synchronize()
#         start = time()
        
#         self.hStateEstimatorMPF._find_likelihood_particles(
#             np.array([0,0,0, 1,0,0,0]),  # pN, pE, pD, qW, qX, qY, qZ
#             UAVKp, 
#             UAVDesc
#         )            
#         end = time()
#         torch.cuda.synchronize()

#         self.get_logger().debug(f"Measurement update takes {end - start:.4f} seconds")


#     def image_callback(self, msg):
#         """Callback function for ROS Image subscriber"""
#         # self.get_logger().info('Received image frame')
        
#         try:
#             # ROS Görüntü Mesajını OpenCV formatına dönüştür (BGR8)
#             # Orijinal kodunuz imio ile okuyup [..., ::-1] yapıyordu (RGB -> BGR).
#             # Çoğu ROS kamera sürücüsü 'bgr8' veya 'rgb8' yayınlar.
#             # 'bgr8' OpenCV için standarttır.
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")  # Mono8 formatında alınıyor
#         except CvBridgeError as e:
#             self.get_logger().error(f'CV Bridge error: {e}')
#             return
            
#         # Gelen görüntü üzerinde ön işleme gerekliyse burada yapabilirsiniz
#         # (resize_image gibi)
#         # Orijinal kodunuzdaki resize, snapUAVImageLive içinde yapılıyor gibi görünüyor,
#         # bu yüzden burada ekstra bir şey yapmıyorum.

#         # Ağır işlemi thread pool'a gönder
#         self.pool.submit(self._measurement_update_worker, cv_image)




#     def destroy_node(self):
#         """Node kapanırken thread pool'u güvenle kapat"""
#         self.get_logger().info('Shutting down thread pool...')
#         self.pool.shutdown(wait=True)
#         super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    # image_processor_node = ImageProcessorNode()
    vio_processor_node = VIOProcessorNode()

    # executor.add_node(image_processor_node)
    executor.add_node(vio_processor_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Düğümü ve thread pool'u düzgünce kapat
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()