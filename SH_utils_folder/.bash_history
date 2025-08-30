ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
lsusb
cutecom
ls -l /dev/ttyUSB0
lsusb
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
dmesg | grep ttyUSB
sudo dmesg | grep ttyUSB
sudo minicom -D /dev/ttyUSB0 -b 921600
cutecom
dmesg | grep ttyUSB
lsusb
ls -l /dev/ttyUSB0
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 launch xsens_mti_ros2_driver display.launch.py
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
ros2 launch xsens_mti_ros2_driver display.launch.py
cd Desktop/
ls
ros2 bag info rosbag2_2025_08_05-22_49_45/
rosbags-convert --src rosbag2_2025_08_05-22_49_45/ --dst IMU_MTIG700.bag
fan
ros2 topic echo /imu/data 
ros2 topic hz /imu/data 
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
rosbags-convert --src rosbag2_2025_08_05-22_49_45/ --dst IMU_MTIG700.bag
ls -l /dev/ttyUSB0
sudo insmod ./xsens_mt.ko
ls -l /dev/ttyUSB0
roslaunch xsens_mti_driver display.launch
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 launch xsens_mti_ros2_driver display.launch.py
python ros_cam_test.py 
ros2 launch camera_driver camera_driver.launch.py
cd xsens_mt/
ls
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
ros2 launch xsens_mti_ros2_driver display.launch.py
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
rviz2
ros2 launch camera_driver camera_driver.launch.py
python ros_cam_test.py 
sudo dmesg
ls -l /dev/video*	
sudo ls -l /dev/video*	
sudo /opt/nvidia/jetson-io/jetson-io.py
python ros_cam_test.py 
cd xsens_mt/
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 topic hz /imu/data 
ros2 topic echo /imu/data 
ros2 launch camera_driver camera_driver.launch.py
rvi<2
rviz2
cd ..
cd Desktop/
ros2 bag record bag_IMUxsens_cam4mm /camera/image_raw /imu/data
rosbags-convert --src rosbag2_2025_08_07-13_58_59/ --dst calib_IMUxsens_cam4mm.bag
ros2 topic list
ros2 bag record bag_IMUxsens_cam4mm /camera/image_raw /imu/data
ros2 topic echo /imu/data
ros2 topic hz /imu/data
ros2 bag record bag_IMUxsens_cam4mm /camera/image_raw /imu/data
rqt
rqt2
rqt
ros2 topic hz /imu/data 
rviz2
ros2 bag record bag_IMUxsens_cam4mm /camera/image_raw /imu/data
ros2 bag info rosbag2_2025_08_07-15_56_05/
ros2 topic hz /imu/data 
ros2 topic hz /imu/data
ros2 topic hz /imu/data -w 20
ros2 topic hz /imu/data -w 400
ls
ros2 bag record bag_IMUxsens_cam4mm /camera/image_raw /imu/data
ros2 bag info rosbag2_2025_08_07-16_30_38/
fan
rosbags-convert --src rosbag2_2025_08_07-16_30_38/ --dst IMUCAM.bag
ros2 topic hz /imu/data 
ros2 launch camera_driver camera_driver.launch.py
ros2 topic list
ros2 topic hz /imu/data 
ros2 launch camera_driver camera_driver.launch.py
cd ..
cd Desktop/
python runner.py 
cd xsens_mt/
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
ls -l /dev/ttyUSB0
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
cd ..
cd Desktop/
python runner.py 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
python runner.py 
cd xsens_mt/
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
setserial /dev/ttyUSB0 low_latency
ls -l /dev/ttyUSB0
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 topic list
ros2 topic hz /imu/data
ros2 launch camera_driver camera_driver.launch.py
ros2 topic hz /imu/data
ros2 topic hz /imu/data -w 10
ros2 topic hz /imu/data -w 400
cd  ..
cd Desktop/
./setup_macro.sh 
cd Desktop/
./setup_macro.sh 
cd DEs
cd Desktop/
./setup_macro.sh 
cd
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
ls
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
sudo shutdown now
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./record_bag.sh 
/usr/bin/python /home/ituarc/Documents/Github/FeatureMatching-PythonCODE/OV/VIO_GT_comp.py
cd OV
/usr/bin/python /home/ituarc/Documents/Github/FeatureMatching-PythonCODE/OV/VIO_GT_comp.py
/usr/bin/python /home/ituarc/Documents/Github/FeatureMatching-PythonCODE/OV/VIO_GT_comp.py,
/usr/bin/python /home/ituarc/Documents/Github/FeatureMatching-PythonCODE/OV/VIO_GT_comp.py
cd Desktop/
./setup_macro_xsens.sh 
echo "1" | sudo -S make HAVE_LIBUSB=1
./setup_macro_xsens.sh 
cd Desktop/
./setup_macro_xsens.sh 
cd Desktop/
./setup_macro_xsens.sh 
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/
cd OV
python test_sim_VIOodom_square
python test_sim_VIOodom_square.py 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
echo "1" | sudo -S make HAVE_LIBUSB=1
sleep 1
sudo modprobe usbserial
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
echo "1" | sudo -S make HAVE_LIBUSB=1
sleep 1
sudo modprobe usbserial
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
echo "1" | sudo -S make HAVE_LIBUSB=1
sleep 1
sudo modprobe usbserial
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2
echo "1" | sudo -S make HAVE_LIBUSB=1
sleep 1
sudo modprobe usbserial
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2
printf "1\n" | sudo -S make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2
sudo make HAVE_LIBUSB=1
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/ &
sleep 2
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 1
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 1
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 1
sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 10 && sudo make HAVE_LIBUSB=1
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 10 && sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2 && sudo make HAVE_LIBUSB=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 1 && sudo modprobe usbserial
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
cd Desktop/
./setup_macro_all_ssh.sh 
ros2 topic hz /imu/data
tmux kill-server
[200~sudo visudo
sudo visudo
sudo -S make HAVE_LIBUSB=1
sudo visudo
tmux kill-server
[A
tmux kill-server
[A
tmux kill-server
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
tmux kill-server
ros2 topic echo /imu/data
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/ &
sleep 1 && sudo make HAVE_LIBUSB=1 &
sleep 2 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/ &
sleep 1 && sudo make HAVE_LIBUSB=1 &
sleep 2 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
tmux kill-server
ros2 topic echo /imu/data
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 1 && sudo make HAVE_LIBUSB=1 &
sleep 2 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 2 && sudo make HAVE_LIBUSB=1 &
sleep 2 && sudo modprobe usbserial &
sleep 2 && sudo insmod ./xsens_mt.ko &
sleep 2 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
tmux kill-server
cd ~/xsens_mt/
[200~sudo make HAVE_LIBUSB=1~
sudo make HAVE_LIBUSB=1
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
sudo setserial /dev/ttyUSB0 low_latency
cd DEs
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
tmux kill-server
python test_sim_VIOodom_square.py 
tmux kill-server
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./record_bag.sh 
sudo shutdown now
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
sudo shutdown now
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
tmux kill-server
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
sudo shutdown now
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
tmux kill-server
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square.py 
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /imu/data --field linear_acceleration
ros2 topic echo /mavros/imu/data_raw --field linear_acceleration
ros2 topic echo /mavros/imu/data_raw
ros2 topic echo /mavros/imu/data_raw --field linear_acceleration
ros2 topic echo /imu/data --field linear_acceleration
sudo shutdown now
cd Desktop/
ros2 bag play rosbag2_2025_08_18-13_13_40/
tmux kill-server
fan
ros2 bag play rosbag2_2025_08_18-13_13_40/
cd rosbag_08_03/
ros2 bag play rosbag2_2025_08_03-16_01_26/
cd ..
ros2 bag play rosbag2_2025_08_18-13_13_40/
python VIO_odom_test_mag.py 
cd OV
python VIO_odom_test_mag.py 
ros2 topic list
cd Desktop/
./setup_macro.sh 
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python 
python VIO_odom_test_mag.py 
ros2 topic list
ros2 topic echo /mavros/global_position/altitude
ros2 topic echo /mavros/imu/data
ros2 topic echo /mavros/global_position/
ros2 topic echo /mavros/altitude 
ros2 topic echo /mavros/global_position/rel_alt
python test_sim_VIOodom_square_oto_climb_sitl.py 
fan
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_sitl.py 
cd Desktop/
./record_bag.sh 
sudo shutdown now
ls
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching XSENS IMU…"
cd ~/xsens_mt/
sleep 5 && sudo make HAVE_LIBUSB=1 &
sleep 1 && sudo modprobe usbserial &
sleep 1 && sudo insmod ./xsens_mt.ko &
sleep 1 && sudo setserial /dev/ttyUSB0 low_latency &
sleep 1 && ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 1 && ros2 topic hz /imu/data -w 400
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
cd Desktop/
ros2 bag play rosbag2_2025_08_18-17_26_11/
./setup_macro.sh 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
fan
cd Desktop/
./setup_macroç
./setup_macro.sh 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_sitl.py 
ls
cd ~/Documents/
cd ..
cd Desktop/
ls
rm -r rosbag_07_14/
rm -rf rosbag_07_14/
ls
rm -rf rosbag_07_20/
sudo shutdown now
python VIO_GT_comp.py 
cd Desktop/
fan
cd Desktop/
fan
cd Desktop/
./setup_macro_all_ssh.sh 
cd D
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_sitl.py 
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
./record_bag.sh 
sudo shutdown now
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
python VIO_GT_comp.py 
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
sudo shutdown now
fan
cd Desktop/
./setup_macro.sh 
fan
cd Desktop/
./setup_macro.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
tmux kill-server
cd De
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
fan
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
tmux kill-server
cd Desktop/
ls
./record_bag.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
fan
./record_bag.sh 
sudo shutdown now
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd OV
python VIO_GT_comp.py 
python VIO_odom_test_mag.py 
python VIO_odom_test_mag.py
fan
cd Desktop/
ros2 bag play rosbag2_2025_08_19-08_30_19/
ros2 topic list
ros2 topic echo /mavros/global_position/local 
ros2 topic echo /mavros/global_position/local --fifield pose.pose.position
ros2 topic echo /mavros/global_position/local --field pose.pose.position
ros2 launch camera_driver camera_driver.launch.py
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
fan
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
cd Desktop/
./setup_macro.sh 
cd Desktop/
./setup_macro.sh 
fan
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Desktop/
./record_bag.sh 
sudo shutdown now
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./record_bag.sh 
sudo shutdown now
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
fan
./record_bag.sh 
tmux kill-server
sudo shutdonw now
sudo shutdown now
cd Desktop/
./setup_macro_all_ssh.sh 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
cd Desktop/
ros2 bag play rosbag2_2025_08_21-09_29_05/ 
python ros_cam_test.py 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
sudo shutdown now
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd DEs
cd Desktop/
ros2 bag play rosbag2_2025_08_21-10_12_09/
python ros_cam_test.py 
pyton ros_cam_test.py 
python ros_cam_test.py 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
ls
rm -rf rosbag2_2025_08_21-10_12_09/
./record_bag.sh 
sudo shutdown now
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
fan
gst-launch-1.0   nvarguscamerasrc sensor-id=0 sensor-mode=2                    tnr-mode=0  tnr-strength=0                    ee-mode=0   ee-strength=0   ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1'   ! nvvidconv flip-method=0  interpolation-method=5   ! 'video/x-raw(memory:NVMM),width=960,height=720'   ! nvegltransform ! nveglglessink -e
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
gst-launch-1.0   nvarguscamerasrc sensor-id=1 sensor-mode=2                    tnr-mode=0  tnr-strength=0                    ee-mode=0   ee-strength=0   ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1'   ! nvvidconv flip-method=0  interpolation-method=5   ! 'video/x-raw(memory:NVMM),width=960,height=720'   ! nvegltransform ! nveglglessink -e
gst-launch-1.0   nvarguscamerasrc sensor-id=1 sensor-mode=2                    tnr-mode=0  tnr-strength=0                    ee-mode=0   ee-strength=0   ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1'   ! nvvidconv flip-method=0  interpolation-method=5   ! 'video/x-raw(memory:NVMM),width=960,height=720'   ! nvegltransform ! nveglglessink -e
gst-launch-1.0   nvarguscamerasrc sensor-id=0 sensor-mode=2                    tnr-mode=0  tnr-strength=0                    ee-mode=0   ee-strength=0   ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1'   ! nvvidconv flip-method=0  interpolation-method=5   ! 'video/x-raw(memory:NVMM),width=960,height=720'   ! nvegltransform ! nveglglessink -e
nano kernel_tegra234-p3768-0000+p3767-0001-nv-super.dtb 
cd Desktop/
ros2 bag play rosbag2_2025_08_21-10_48_40/
ls
ros2 bag play rosbag2_2025_08_19-12_10_28/
ros2 bag play rosbag2_2025_08_21-10_48_40/
# simple preview with gstreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink
# simple preview with gstreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink
ls /dev/video*
python ros_cam_test.py 
[200~ls -l /dev/video*
~
ls -l /dev/video*
ls /dev/video*
sudo /opt/nvidia/jetson-io/jetson-io.py
[200~sudo /opt/nvidia/jetson-io/jetson-io.py
sudo /opt/nvidia/jetson-io/jetson-io.py
sudo reboot
ls /dev/video*
sudo /opt/nvidia/jetson-io/jetson-io.py
ls /dev/video*
dmesg | grep -Ei 'imx|ov|cam|vi5'
sudo dmesg | grep -Ei 'imx|ov|cam|vi5'
ls -l /dev/video*
sudo /opt/nvidia/jetson-io/jetson-io.py
sudo nano /boot/extlinux/extlinux.conf
ls -1 /boot/*imx477*.dtbo /boot/dtb/*imx477*.dtbo 2>/dev/null
sudo /opt/nvidia/jetson-io/jetson-io.py
ls -l /dev/video*
grep -H . /sys/class/i2c-adapter/i2c-*/name | grep -i cam_i2cmux
i2cdetect -y -r 9
i2cdetect -y -r 8
sudo systemctl restart nvargus-daemon
dmesg | grep -Ei 'imx477|vi5|nvcsi'
sudo dmesg | grep -Ei 'imx477|vi5|nvcsi'
ls -l /dev/video*
gst-launch-1.0 nvarguscamerasrc sensor-id=0 !   'video/x-raw(memory:NVMM),width=640,height=480' ! nvoverlaysink -e
python ros_
python ros_cam_test.py 
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=200
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --set-ctrl=gain=100
v4l2-ctl -d /dev/video0 --set-ctrl=gain=300
sudo /opt/nvidia/jetson-io/jetson-io.py
python ros_cam_test.py 
cd Desktop/
./setup_macro.sh 
cd Desktop/
./setup_macro.sh 
cd Desktop/
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
fam
fan
cd Desktop/
fan
1
./record_bag.sh 
sudo shutdown now
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
cd Desktop/
ros2 bag play rosbag2_2025_08_21-14_36_02/ 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd ..
python ros_cam_test.py 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
fan
python ros_cam_test.py 
cd Desktop/
./setup_macro.sh 
cd ..
python ros_cam_test.py 
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./setup_macro.sh 
cd Desktop/
./setup_macro.sh 
cd Desktop/
./setup_macro.sh 
cd Desktop/
./record_bag.sh 
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
sudo shutdown now
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
fan
ros2 bag play rosbag2_2025_08_21-15_39_35/
ros2 bag play rosbag2_2025_08_21-15_39_35/ --start-offset 19
ros2 bag play rosbag2_2025_08_21-15_39_35/ --start-offset 10
./setup_macro.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
ls
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
rm -rf rosbag2_2025_08_21-15_39_35/
./record_bag.sh 
sudo shutdown now
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
ros2 bag play rosbag2_2025_08_21-16_28_08/
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./record_bag.sh 
sudo shutdown now
/usr/bin/python /home/ituarc/Documents/Github/FeatureMatching-PythonCODE/OV/VIO_GT_comp.py
cd OV
python VIO_GT_comp.py 
python VIO_odom_comp_testMAE.py 
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python VIO_odom_comp_testMAE.py 
ros2 topic list
ros2 toic list
ros2 topic list
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
ros2 topic list
ros2 topic echo /mavros/global_position/local  
fan
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position.z
cd Desktop/
ros2 bag play rosbag2_2025_08_21-16_28_08/
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
tmux kill-server
python VIO_GT_comp.py 
cd /Documents/Github/FeatureMatching-PythonCODE/OV
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
cd Desktop/
fan
./record_bag.sh 
cd Desktop/
./setup_macro_all_ssh.sh
cd Desktop/
cd Desktop/
fan
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
tmux kill-server
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
ros2 bag play rosbag2_2025_08_21-18_43_20/
sudo shutdown now
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
tmux kill-server
sudo shutdown now
cd Desktop/
./record_bag.sh 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
ros2 bag play rosbag2_2025_08_22-07_45_09/
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
ros2 bag play rosbag2_2025_08_22-08_15_10/
tmux kill-server
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
ls
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
tmux kill-server
cd DES
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/O
cd Documents/Github/FeatureMatching-PythonCODE/OV/
python test_sim_VIOodom_square_oto_climb_land.py 
fan
ros2 bag play rosbag2_2025_08_22-08_34_36/
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
tmux kill-serevr
tmux kill-server
cd Desktop/
python VIO_GT_comp.py 
ls -l /dev/ttyUSB0
dmesg | grep tty
sudo dmesg | grep tty
stty -F /dev/ttyACM0 921600 raw -echo
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
sudo modprobe -r cdc_acm
sudo modprobe cdc_acm nrurbs=1
fan
cd Desktop/
./setup_macro.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd OV
python VIO_GT_comp.py 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
tmux kill-server
cd Desktop/
ros2 bag play rosbag2_2025_08_22-09_19_20/
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
tmux kill-server
python VIO_GT_comp.py 
stty -F /dev/ttyACM0 921600 raw -echo
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
ros2 topic list
stty -F /dev/ttyACM0 921600 raw -echo
reboot now
sudo reboot
rviz2
cd Desktop/
stty -F /dev/ttyACM0 921600 raw -echo
sharpness.v5.enable = TRUE; # turn off edge enhancer
sudo tee /var/nvidia/nvcam/settings/camera_overrides.isp >/dev/null <<'EOF'


sudo tee /var/nvidia/nvcam/settings/camera_overrides.isp >/dev/null <<'EOF'

tnr.v1.enable      = 0;      # turn off temporal noise reduction

sharpness.v5.enable = TRUE; # turn off edge enhancer


EOF

sudo tee /var/nvidia/nvcam/settings/camera_overrides.isp >/dev/null <<'EOF'
tnr.v1.enable      = 2;      # turn off temporal noise reduction
sharpness.v5.enable = TRUE; # turn off edge enhancer
EOF

sudo chmod 644 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo systemctl restart nvargus-daemon
reboot now
sudo reboot 
rviz2
ros2 launch camera_driver camera_driver.launch.py
rivz2
rviz2
reboot now
1
sudo reboot
rviz2
sudo tee /var/nvidia/nvcam/settings/camera_overrides.isp >/dev/null <<'EOF'
tnr.v1.enable      = 0;      # turn off temporal noise reduction
sharpness.v5.enable = FALSE; # turn off edge enhancer
EOF

sudo chmod 644 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo systemctl restart nvargus-daemon
sudo reboot
stty -F /dev/ttyACM0 921600 raw -echo
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
sudo modprobe -r cdc_acm
sudo modprobe cdc_acm nrurbs=1
cd Desktop/
ros2 bag record bag /camera/image_raw /mavros/imu/data_raw
rosbags-convert --src rosbag2_2025_08_22-18_21_25/ --dst IMUCAM.bag
fan
rviz
rviz2
ros2 bag record bag /camera/image_raw /mavros/imu/data_raw
rosbags-convert --src rosbag2_2025_08_22-18_49_02/ --dst IMUCAM_0822_2.bag
cd Desktop/
ros2 bag record bag /camera/image_raw /mavros/imu/data_raw
rosbags-convert --src rosbag2_2025_08_22-19_01_54/ --dst IMUCAM_0822_3.bag
sudo shutdownnow
sudo shutdown now
fan
cd Desktop/
./setup_macro.sh 
sudo shutdown now
stty -F /dev/ttyACM0 921600 raw -echo
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
sudo modprobe -r cdc_acm
sudo modprobe cdc_acm nrurbs=1
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
./setup_macro.sh 
fan
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
./setup_macro.sh 
fan
cd Desktop/
./setup_macro.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./record_bag.sh 
tmux kill-server
sudo shutdown now
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
python VIO_GT_comp.py 
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd..
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python VIO_odom_comp_testMAE.py 
cd Desktop/
ros2 bag play rosbag2_2025_08_30-10_42_04/
ros2 bag play rosbag2_2025_08_30-10_42_04/,
ros2 bag play rosbag2_2025_08_30-10_42_04/
ros2 topic hz /mavros/imu/data_raw 
ros2 topic hz /mavros/imu/data_raw -w
ros2 topic hz /mavros/imu/data_raw -w 200
./setup_macro.sh 
python test_sim_VIOodom_square_oto_climb_land.py 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Desktop/
./record_bag_all.sh
sudo shutdown now
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Desktop/
./setup_macro_all_ssh.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd Desktop/
./record_bag_all.sh 
cd Documents/Github/FeatureMatching-PythonCODE/ov
cd Documents/Github/FeatureMatching-PythonCODE/OV/
python test_sim_VIOodom_square_oto_climb_land.py 
sudo shutdown now
cd Desktop/
cd ..
cd ros2_ws/
ls
cd src
ls
cd open_vins/
ls
cd config/
cd my_config/
nano estimator_config.yaml 
cd ..
cd Desktop/
./setup_macro_all_ssh.sh 
python VIO_GT_comp.py 
jtop
./setup_macro.sh 
rviz2 -d ~/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
jtop
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
fan
cd Desktop/
ros2 bag info rosbag2_2025_08_30-11_53_30/
ros2 bag play rosbag2_2025_08_30-11_53_30/
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
fan
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
cd Documents/Github/FeatureMatching-PythonCODE/
cd OV
python test_sim_VIOodom_square_oto_climb_land.py 
which fan
type fan
which jetson_clocks
sudo visudo
fan
echo "▶ Launching MAVROS…"
fan
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
fan
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
tmux kill-server
sudo reboot
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./setup_macro_all_ssh.sh 
cd ..
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land.py 
cd OV
python VIO_GT_comp.py 
cd DES
cd Desktop/
ls
ros2 bag play rosbag2_2025_08_30-13_27_03/
ros2 bag info rosbag2_2025_08_30-13_27_03/
ros2 bag info rosbag2_2025_08_30-13_27_03/ --topic 
ros2 bag play rosbag2_2025_08_30-13_27_03/ --topic
ros2 bag play rosbag2_2025_08_30-13_27_03/ --topics 
ros2 bag play rosbag2_2025_08_30-13_27_03/ --topics /camera/image_raw /mavros/imu/data_raw
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
python VIO_GT_comp.py 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
python VIO_GT_comp.py 
tmux kill-server
cd Desktop/
ros2 bag play rosbag2_2025_08_30-14_35_32/
ros2 topic list
ros2 topic echo /ov_msckf/odomimu
ros2 topic echo /ov_msckf/odomimu --field twist.twist.linear
ros2 topic list
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
cd Desktop/
./record_bag.sh 
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./setup_macro_all_ssh.sh 
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./record_bag.sh 
python VIO_GT_comp.py 
cd OV
python VIO_GT_comp.py 
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
cd OV
python VIO_GT_comp.py 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
sudo shutdown now
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 20
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 200
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting system monitoring (CPU, RAM, Temp)…"
while true; do   timestamp=$(date +"%Y-%m-%d %H:%M:%S");   cpu=$(mpstat 1 1 | awk "/Average:/ {print 100-\$12}");   cpu=$(printf "%.2f" "$cpu");   ram=$(free -m | awk "NR==2{printf \"%.2f\", \$3*100/\$2 }");   ram=$(printf "%.2f" "$ram");   temp=$(cat /sys/class/thermal/thermal_zone0/temp);   temp=$(printf "%.2f" "$((${temp}/1000))");   echo "$timestamp, CPU: ${cpu}%, RAM: ${ram}%, Temp: ${temp}°C" >> system_monitor.log;   sleep 5; done
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 200
jtop
./setup_macro.sh 
jtop
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
jtop
ros2 launch ov_msckf subscribe.launch.py config:=my_config max_cameras:=1
./setup_macro.sh 
tmux kill-server
cd Desktop/
ros2 bag play rosbag2_2025_08_30-17_40_21/
fan
1
top -b -n 1 | grep "Cpu(s)"
free -h >> ram_usage.log
#!/bin/bash
while true; do
    mpstat -P ALL 1 | tail -n 1 >> cpu_usage.log    
    free -h >> ram_usage.log    
    cat /sys/class/thermal/thermal_zone0/temp >> temperature.log    
    sleep 1; done
sudo apt install sysstat
chmod +x systemlog.sh 
./systemlog.sh 
mpstat -P ALL 1 >> cpu_usage.log
./systemlog.sh 
./setup_macro_all_ssh.sh 
cd Desktop/
./record_bag.sh 
cd Desktop/
./setup_macro_all_ssh
./setup_macro_all_ssh.sh 
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
echo "▶ Starting system monitoring (CPU, RAM, Temp)…"
while true; do   timestamp=$(date +"%Y-%m-%d %H:%M:%S");   cpu=$(mpstat 1 1 | awk "/Average:/ {print 100-\$12}");   cpu=$(printf "%.2f" "$cpu");   ram=$(free -m | awk "NR==2{printf \"%.2f\", \$3*100/\$2 }");   ram=$(printf "%.2f" "$ram");   temp=$(cat /sys/class/thermal/thermal_zone0/temp);   temp=$(printf "%.2f" "$((${temp}/1000))");   echo "$timestamp, CPU: ${cpu}%, RAM: ${ram}%, Temp: ${temp}°C" >> system_monitor.log;   sleep 5; done
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
sudo shutdown now
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 200
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 200
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Running runner_sh.py…"
sleep 10
python runner_sh.py
cd Desktop/
ros2 bag play rosbag2_2025_08_30-18_37_22/ --start-time 30
ros2 bag play -h
ros2 bag play rosbag2_2025_08_30-18_37_22/ --start-offset 30
ros2 bag play rosbag2_2025_08_30-18_37_22/
./setup_macro_all_ssh.sh 
echo "▶ Starting camera driver…"
fan &
ros2 launch camera_driver camera_driver.launch.py
echo "▶ Starting system monitoring (CPU, RAM, Temp)…"
while true; do   timestamp=$(date +"%Y-%m-%d %H:%M:%S");   cpu=$(mpstat 1 1 | awk "/Average:/ {print 100-\$12}");   cpu=$(printf "%.2f" "$cpu");   ram=$(free -m | awk "NR==2{printf \"%.2f\", \$3*100/\$2 }");   ram=$(printf "%.2f" "$ram");   temp=$(cat /sys/class/thermal/thermal_zone0/temp);   temp=$(printf "%.2f" "$((${temp}/1000))");   echo "$timestamp, CPU: ${cpu}%, RAM: ${ram}%, Temp: ${temp}°C" >> system_monitor.log;   sleep 5; done
cd Documents/Github/FeatureMatching-PythonCODE/OV
python test_sim_VIOodom_square_oto_climb_land_nogps.py 
sudo shutdown now
echo "▶ Launching MAVROS…"
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
sleep 5
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 1, on_off: true}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 33,  message_rate: 20.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 27, message_rate: 200.0}"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 105, message_rate: 200.0}"
ros2 topic hz /mavros/imu/data_raw -w 200
cd Desktop/
./record_bag.sh 
cd Desktop/
./setup_macro_all_ssh
./setup_macro_all_ssh.sh 
