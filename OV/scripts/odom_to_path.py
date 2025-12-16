#!/usr/bin/env python3
"""
Convert Odometry messages to Path messages for RViz2 visualization.
Also publishes a static TF for the odom_ned frame.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Declare parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('max_length', 100000)  # Large value to keep path permanent
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('flip_z', True)  # Flip Z for NED to visualization (Down -> Up)
        self.declare_parameter('flip_y', False)  # Default: don't flip Y (East stays East)
        
        # Get parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.max_length = self.get_parameter('max_length').get_parameter_value().integer_value
        publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.flip_z = self.get_parameter('flip_z').get_parameter_value().bool_value
        self.flip_y = self.get_parameter('flip_y').get_parameter_value().bool_value
        
        # Initialize path (frame_id will be set from first odometry message)
        self.path = Path()
        
        # Static TF broadcaster (only if this node is responsible for TF)
        if publish_tf:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()
        
        # Create subscriber and publisher
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            10
        )
        
        self.get_logger().info(f'Converting {odom_topic} -> {path_topic} (flip_y={self.flip_y}, flip_z={self.flip_z})')
    
    def _publish_static_tf(self):
        """Publish static transform from world to odom_ned frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom_ned'
        
        # Identity transform (odom_ned is at world origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static TF: world -> odom_ned')
    
    def odom_callback(self, msg):
        """Convert odometry to path."""
        # Create pose from odometry
        pose = PoseStamped()
        pose.header = msg.header  # Use the same header (including frame_id) from odometry
        pose.header.frame_id = 'world'  # Use world frame for visualization
        
        # Copy pose data with axis flips for visualization
        pose.pose.position.x = msg.pose.pose.position.x
        
        # Flip Y axis if needed (East direction correction)
        if self.flip_y:
            pose.pose.position.y = -msg.pose.pose.position.y
        else:
            pose.pose.position.y = msg.pose.pose.position.y
        
        # Flip Z axis if needed (NED Down -> RViz Up)
        if self.flip_z:
            pose.pose.position.z = -msg.pose.pose.position.z
        else:
            pose.pose.position.z = msg.pose.pose.position.z
        
        pose.pose.orientation = msg.pose.pose.orientation
        
        # Add to path
        self.path.poses.append(pose)
        
        # Limit path length
        if len(self.path.poses) > self.max_length:
            self.path.poses.pop(0)
        
        # Update path header
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = 'world'  # Use world frame for visualization
        
        # Publish path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
