#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Int64
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition

import math as m


class eater(Node):

    def __init__(self):
        super().__init__('eater_node')
        self.pose = None
        self.queue = []
        self.max_pizzas = None
        self.spawned_pizzas = 0
        self.eaten_pizzas = 0
        self.mode = 'PIZZA'
        self.escape_target = None
        self._masterkey_published = False

        # pubs/subs
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.masterkey_pub = self.create_publisher(Int64, '/masterkey', 10)
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.create_subscription(Point, '/mouse_position', self._gui_click_cb, 10)

        # IMPORTANT: RViz2's 2D Nav Goal publishes geometry_msgs/PoseStamped on /goal_pose
        self.create_subscription(PoseStamped, '/goal_pose', self._rviz_click_cb, 10)

        self.create_subscription(Int64, '/max_pizza', self._max_pizza_cb, 10)

        # services
        self.eat_cli = self.create_client(Empty, '/turtle1/eat')
        self.spawn_pizza_cli = self.create_client(GivePosition, '/spawn_pizza')

        self.create_timer(0.01, self._timer_cb)

    def _pose_cb(self, msg: Pose):
        self.pose = (msg.x, msg.y, msg.theta)

    def _max_pizza_cb(self, msg: Int64):
        self.max_pizzas = max(0, int(msg.data))
        self.get_logger().info(f'max_pizzas set to {self.max_pizzas}')
        if self.eaten_pizzas >= self.max_pizzas:
            self.mode = 'ESCAPE'

    def _gui_click_cb(self, msg: Point):
        x, y = self._clip(msg.x, msg.y)
        self._handle_click(x, y)

    # UPDATED: use PoseStamped from RViz2's /goal_pose
    def _rviz_click_cb(self, msg: PoseStamped):
    
        tx = msg.pose.position.x + 5.44
        ty = msg.pose.position.y + 5.44
        x, y = self._clip(tx, ty)
        self._handle_click(x, y)

    def _handle_click(self, x: float, y: float):
        if self.max_pizzas is None:
            self.get_logger().warn('Ignored')
            return
        if self.eaten_pizzas >= self.max_pizzas:
            self.mode = 'ESCAPE'

        if self.mode == 'PIZZA' and self.spawned_pizzas < self.max_pizzas:
            self._spawn_pizza(x, y)
            self.queue.append({"x": x, "y": y, "kind": "PIZZA"})
            self.spawned_pizzas += 1
            self.get_logger().info('Spawned pizza')
            if self.spawned_pizzas >= self.max_pizzas:
                self.get_logger().info('incase')
        else:
            self.mode = 'ESCAPE'
            self.escape_target = (x, y)
            self.get_logger().info(f'ESCAPE target updated to ({x:.2f}, {y:.2f}).')

    def _spawn_pizza(self, x: float, y: float):
        if not self.spawn_pizza_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Ignored')
            return
        req = GivePosition.Request()
        req.x = float(x)
        req.y = float(y)
        self.spawn_pizza_cli.call_async(req)

    def _eat(self):
        if not self.eat_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('service unavailable')
            return
        self.eat_cli.call_async(Empty.Request())

    def _publish_masterkey_once(self, value: int):
        if self._masterkey_published:
            return
        msg = Int64()
        msg.data = int(value)
        self.masterkey_pub.publish(msg)
        self._masterkey_published = True
        self.get_logger().info(f'Publish masterkey = {value}')

    def _timer_cb(self):
        if self.pose is None:
            self._cmd(0.0, 0.0)
            return

        active_goal = None
        active_kind = None

        if self.queue:
            active_goal = (self.queue[0]["x"], self.queue[0]["y"])
            active_kind = self.queue[0]["kind"]
        elif self.mode == 'ESCAPE' and self.escape_target is not None:
            active_goal = self.escape_target
            active_kind = "ESCAPE"

        if active_goal is None:
            self._cmd(0.0, 0.0)
            return

        x, y, th = self.pose
        tx, ty = active_goal
        dx, dy = (tx - x), (ty - y)
        dist = (dx*dx + dy*dy) ** 0.5
        bearing = m.atan2(dy, dx)
        ang_err = m.atan2(m.sin(bearing - th), m.cos(bearing - th))

        K_lin, K_ang = 5.0, 20.0
        vx = max(-10.0, min(K_lin * dist, 10.0))
        wz = max(-20.0, min(K_ang * ang_err, 20.0))
        self._cmd(vx, wz)

        if dist < 0.12:
            if active_kind == "PIZZA":
                self._cmd(0.0, 0.0)
                self._eat()
                self.eaten_pizzas += 1
                if self.max_pizzas is not None and self.eaten_pizzas == self.max_pizzas:
                    self._publish_masterkey_once(1)
                    self.mode = 'ESCAPE'
                    self.get_logger().info('Escaped mode onn')
                self.queue.pop(0)

            elif active_kind == "ESCAPE":
                self._cmd(0.0, 0.0)

    def _cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    @staticmethod
    def _clip(x: float, y: float):
        x = max(0.0, min(11.0, float(x)))
        y = max(0.0, min(11.0, float(y)))
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = eater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
