from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time
import math
import pymap3d as pm
import numpy as np
from utils import eul2quat

from mavlinkHandler import MAVLinkHandlerDronekit, MAVLinkHandlerPymavlink


class MAVHandler:
    """
    MAVHandler is a helper class that uses DroneKit to manage MAVLink-based drones.
    It provides methods for connecting, arming, takeoff, navigation, and retrieving telemetry data.
    """

    def __init__(self, connection_string, baud_rate=115200):
        """
        Initialize the MAVHandler by connecting to the vehicle.

        :param connection_string: The address string for connecting to the vehicle
                                  (e.g., '/dev/ttyAMA0', 'udp:127.0.0.1:14550', etc.)
        :param baud_rate: Baud rate for serial connection (ignored for UDP/TCP connections).
        """
        print(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=True, rate=1000)
        print("Connection established.")

        self.imu_raw_data = {'xacc' : 0, 'yacc' : 0, 'zacc' : 0 ,
                         'xgyro': 0, 'ygyro': 0, 'zgyro': 0 ,
                         'timestamp' : 0}
        self.imu_scaled_data = {'xacc' : 0, 'yacc' : 0, 'zacc' : 0 ,
                            'xgyro': 0, 'ygyro': 0, 'zgyro': 0 ,
                            'timestamp' : 0}
        self.imu_highres_data = {'xacc' : 0, 'yacc' : 0, 'zacc' : 0 ,
                            'xgyro': 0, 'ygyro': 0, 'zgyro': 0 ,
                            'timestamp' : 0}
        
        self.timestamp = []
        self.boot_time = time.time()
        self.g = 9.80665

        # Tell DroneKit to call our method on RAW_IMU messages
        self.vehicle.add_message_listener('RAW_IMU', self.receivedIMU)
        self.vehicle.add_message_listener('SCALED_IMU', self.receivedIMU_scaled)
        self.vehicle.add_message_listener('HIGHRES_IMU', self.receivedIMU_highres)

    def receivedIMU(self, vehicle, name, msg):
        # Now `self` is the MAVHandler instance, and
        # `vehicle` is the dronekit.Vehicle object
        self.imu_raw_data['xacc']      = msg.xacc  / 1000.0 * self.g # m/s^2
        self.imu_raw_data['yacc']      = msg.yacc  / 1000.0 * self.g # m/s^2
        self.imu_raw_data['zacc']      = msg.zacc  / 1000.0 * self.g # m/s^2
        self.imu_raw_data['xgyro']     = msg.xgyro / 1000.0 # rad/s
        self.imu_raw_data['ygyro']     = msg.ygyro / 1000.0
        self.imu_raw_data['zgyro']     = msg.zgyro / 1000.0
        self.imu_raw_data['timestamp'] = msg.time_usec #microseconds
        
    def receivedIMU_scaled(self, vehicle, name, msg):
        # Now `self` is the MAVHandler instance, and
        # `vehicle` is the dronekit.Vehicle object
        self.imu_scaled_data['xacc']      = msg.xacc / 1000.0 * self.g # 
        self.imu_scaled_data['yacc']      = msg.yacc / 1000.0 * self.g # 
        self.imu_scaled_data['zacc']      = msg.zacc / 1000.0 * self.g #
        self.imu_scaled_data['xgyro']     = msg.xgyro / 1000.0
        self.imu_scaled_data['ygyro']     = msg.ygyro / 1000.0
        self.imu_scaled_data['zgyro']     = msg.zgyro / 1000.0
        self.imu_scaled_data['timestamp'] = msg.time_usec #miliseconds

    def receivedIMU_highres(self, vehicle, name, msg):
        # Now `self` is the MAVHandler instance, and
        # `vehicle` is the dronekit.Vehicle object
        self.imu_highres_data['xacc']      = msg.xacc      #mG
        self.imu_highres_data['yacc']      = msg.yacc      #mG
        self.imu_highres_data['zacc']      = msg.zacc      #mG
        self.imu_highres_data['xgyro']     = msg.xgyro     #mrad/s
        self.imu_highres_data['ygyro']     = msg.ygyro     #mrad/s
        self.imu_highres_data['zgyro']     = msg.zgyro     #mrad/s
        self.imu_highres_data['timestamp'] = msg.time_usec
        
    def get_states(self, LLA0 = np.array([41.10077353260357, 29.02430814005722 , 0.0])): #defult LLA0 is the ARC location
        """
        Retrieve the current state of the vehicle.

        :return: A numpy array containing the vehicle's state as [pNED, vNED, quat] with respect to the LLA0.
        """
        
        LLA = self.get_locationLLA()
        pN, pE, pD = pm.geodetic2ned(LLA[0], LLA[1], LLA[2], LLA0[0], LLA0[1], 0)
        vN, vE, vD = self.get_velocityNED()
        quat = eul2quat(self.get_attitude())
        
        return np.array([pN, pE, pD, vN, vE, vD, quat[0], quat[1], quat[2], quat[3]])
        
        
    def get_parameter_value(self, parameter_name):
        """
        Get the value of a parameter from the vehicle.

        :param parameter_name: The name of the parameter to retrieve.
        :return: The value of the parameter.
        """
        return self.vehicle.parameters.get(parameter_name)

    def get_locationLLA(self):
        """
        Retrieve the current global-relative location of the drone.

        :return: A LocationGlobalRelative object with the current location.
        """
        return self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon, self.vehicle.location.global_relative_frame.alt
    
    def get_velocityNED(self):
        """
        Get the xyz velocity of aircraft.
        """
        return self.vehicle.velocity


    def get_attitude(self):
        """
        Retrieve the current attitude (yaw,pitch, roll) of the drone.

        :return: An Attitude object with yaw, pitch, roll in radians.
        """
        return self.vehicle.attitude.yaw, self.vehicle.attitude.pitch, self.vehicle.attitude.roll

    def set_mode(self, mode_name):
        """
        Set vehicle mode (e.g., 'GUIDED', 'LOITER', 'AUTO', etc.).

        :param mode_name: A valid flight mode string.
        """
        print(f"Changing mode to {mode_name}...")
        self.vehicle.mode = VehicleMode(mode_name)
        while self.vehicle.mode.name != mode_name:
            print("Waiting for mode to change...")
            time.sleep(1)
        print(f"Vehicle mode changed to {mode_name}.")

    def close_connection(self):
        """
        Closes the connection to the vehicle.
        """
        print("Closing vehicle connection...")
        self.vehicle.close()
        print("Connection closed.")


# Example usage:
if __name__ == "__main__":
    # Replace with your connection details
    # connection_str = "127.0.0.1:14563"
    # handler = MAVHandler(connection_str)

    # while True:
    #     # Get the angular velocity of the drone
    #     handler.set_velocity_body(8, 0, 0)
    #     time.sleep(0.1)


    

    connection_str = "/dev/ttyACM0"
    handler = MAVHandler(connection_str)

    while True:
        print(handler.get_parameter_value("WP_YAW_BEHAVIOR"))
        handler.set_parameter_value("WP_YAW_BEHAVIOR", 0)



      