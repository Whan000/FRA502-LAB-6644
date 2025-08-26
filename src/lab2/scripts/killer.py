#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int64
from turtlesim.srv import Kill   
import math as m


class killer(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.self_pose = None
        self.target_pose = None
        self.follow_enabled = False
        self.is_turtle_spawned = False
        self.target_removed = False
        self.target_name = 'turtle1' 
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(Pose,  '/turtle2/pose', self._self_pose_cb, 10)
        self.create_subscription(Pose,  '/turtle1/pose', self._target_pose_cb, 10)
        self.create_subscription(Int64, '/masterkey',    self._masterkey_cb,  10)

    
        self.remove_cli = self.create_client(Kill, '/remove_turtle')
        self.kill_cli   = self.create_client(Kill, '/kill')

        self.create_timer(0.01, self._timer_cb)

    def _self_pose_cb(self, msg: Pose):
        self.is_turtle_spawned = True
        self.self_pose = (msg.x, msg.y, msg.theta)

    def _target_pose_cb(self, msg: Pose):
        self.target_pose = (msg.x, msg.y, msg.theta)

    def _masterkey_cb(self, msg: Int64):
        if self.target_removed:
            return
        self.follow_enabled = (int(msg.data) == 1)
        if not self.follow_enabled:
            self._cmdvel(0.0, 0.0)

    def _timer_cb(self):
        if self.target_removed:
            self._cmdvel(0.0, 0.0)
            return

        if not self.is_turtle_spawned or not self.follow_enabled:
            self._cmdvel(0.0, 0.0)
            return

        if not (self.self_pose and self.target_pose):
            self._cmdvel(0.0, 0.0)
            return

        x, y, th = self.self_pose
        tx, ty, _ = self.target_pose
        dx, dy  = (tx - x), (ty - y)
        dist    = (dx*dx + dy*dy) ** 0.5
        bearing = m.atan2(dy, dx)
        ang_err = m.atan2(m.sin(bearing - th), m.cos(bearing - th))

        K_lin, K_ang = 5.0, 10.0
        vx = max(-5.0,  min(K_lin * dist,    5.0))
        wz = max(-10.0, min(K_ang * ang_err, 10.0))

        if dist < 0.4:
            self._cmdvel(0.0, 0.0)
            self._remove_target()  
            return

        self._cmdvel(vx, wz)

    def _remove_target(self):
        """Attempt /remove_turtle (Kill), else fall back to /kill (Kill)."""
        if self.target_removed:
            return


        req = Kill.Request()
        req.name = self.target_name

        called = False

        if self.remove_cli.wait_for_service(timeout_sec=0.2):
            self.remove_cli.call_async(req)
            called = True
            self.get_logger().info(f'Removal requested via /remove_turtle for {self.target_name}')

        elif self.kill_cli.wait_for_service(timeout_sec=0.2):
            self.kill_cli.call_async(req)
            called = True
            self.get_logger().info(f'Removal requested via /kill for {self.target_name}')

        if called:
            self.target_removed = True
            self.follow_enabled = False
            self._cmdvel(0.0, 0.0)
        else:
            self.get_logger().warn('No removal service available (/remove_turtle or /kill).')

    def _cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = killer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
