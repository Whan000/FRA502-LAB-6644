#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
try:
    from tf_transformations import quaternion_from_euler  # (roll, pitch, yaw) -> (x,y,z,w)
except Exception:
    quaternion_from_euler = None




def yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) for a yaw-only rotation; fallback if tf_transformations is absent."""
    if quaternion_from_euler is not None:
        return quaternion_from_euler(0.0, 0.0, yaw)
    half = 0.5 * yaw
    cz = math.cos(half)
    sz = math.sin(half)
    return (0.0, 0.0, sz, cz)


class TurtlesimPose(Node):
    def __init__(self):
        super().__init__('turtlesim_pose_node')

        self.declare_parameter('root_frame', 'odom')   
        self.declare_parameter('global_frame', 'world')  
        self.declare_parameter('center_offset', 5.44) 

        self.root_frame = self.get_parameter('root_frame').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.center = float(self.get_parameter('center_offset').value)

        self.odom1_pub = self.create_publisher(Odometry, '/odom1', 10)
        self.odom2_pub = self.create_publisher(Odometry, '/odom2', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_world_to_odom_static()

        self.create_subscription(Pose, '/turtle1/pose',
                                 partial(self._pose_cb, turtle_name='turtle1'), 10)
        self.create_subscription(Pose, '/turtle2/pose',
                                 partial(self._pose_cb, turtle_name='turtle2'), 10)

        self.get_logger().info('turtlesim_pose: Odometry + TF broadcasting is ON')

    def _broadcast_world_to_odom_static(self):
        t = TransformStamped()
        now = self.get_clock().now().to_msg()
        t.header.stamp = now
        t.header.frame_id = self.global_frame   
        t.child_frame_id = self.root_frame     
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f"Static TF set: {self.global_frame} -> {self.root_frame}")

    def _pose_cb(self, msg: Pose, turtle_name: str):

        stamp = self.get_clock().now().to_msg()

        x = float(msg.x) - self.center
        y = float(msg.y) - self.center
        yaw = float(msg.theta)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.root_frame       
        odom.child_frame_id = turtle_name        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if turtle_name == 'turtle1':
            self.odom1_pub.publish(odom)
        else:
            self.odom2_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.root_frame
        t.child_frame_id = turtle_name
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
