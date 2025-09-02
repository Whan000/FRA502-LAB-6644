#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from controller_interfaces.srv import SetMaxPizza, SetParam

import math

class eater(Node):
    def __init__(self):
        super().__init__('eater_node')
        self.name_space = self.get_namespace()
        self.declare_parameter('sampling_frequency', 20.0)
        self.freq = self.get_parameter('sampling_frequency').value

        self.robot_pose = None
        self.waypoint = []
        self.kp_linear = 4.0
        self.kp_angular = 20.0
        self.pizza_max = 0
        self.pizza_spawn_count = 0
        self.pizza_count = 0
        

        
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name_space}/cmd_vel', 10)
        self.eat_pub = self.create_publisher(Bool, f'{self.name_space}/eat_status', 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_click_cb, 10)
        self.create_subscription(Pose, f'{self.name_space}/pose', self.pose_cb, 10)
        self.create_subscription(Int64, f'{self.name_space}/pizza_count', self.pizza_count_cb,10)
        self.create_service(SetMaxPizza, f'{self.name_space}/set_max_pizza', self.set_max_pizza_cb)
        self.create_service(SetParam, f'{self.name_space}/set_param', self.set_control_cb)
        self.create_pizza_client = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, f'{self.name_space}/eat')
        self.create_timer(1.0/self.freq, self.timer_cb)
        self.get_logger().info(f"{self.name_space} is running at: {self.freq} Hz")

    def set_control_cb(self, request, response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        self.get_logger().info(f"kp_linear set to: {self.kp_linear} kp_angular set to: {self.kp_angular}")
        return response

    def set_max_pizza_cb(self, request, response):
        if self.pizza_max <= request.max_pizza.data:
            self.pizza_max = request.max_pizza.data
            response.log.data = f"SetMaxPizza to {self.pizza_max}"
            response.log.data = f"New availible pizza is {self.pizza_max - self.pizza_count}"
            self.get_logger().info(f"SetMaxPizza to {self.pizza_max}")
            self.get_logger().info(f"New availible pizza is {self.pizza_max - self.pizza_count}")
        else:
            response.log.data = f"Pizza is less than what the turtle have eaten"
        return response
    
    def pizza_count_cb(self, msg):
        self.pizza_count = msg.data

    def eat_pizza(self):
        req = Empty.Request()
        self.eat_pizza_client.call_async(req)
    
    def spawn_pizza(self, x, y):
        req = GivePosition.Request()
        req.x = x
        req.y = y
        if self.pizza_spawn_count < self.pizza_max:
            self.create_pizza_client.call_async(req)
            self.pizza_spawn_count += 1
            self.get_logger().info(f"You can spawn {self.pizza_max - self.pizza_spawn_count}, Pizza Max {self.pizza_max}")

    def mouse_click_cb(self, msg):
        waypoint = [msg.x, msg.y]
        self.spawn_pizza(waypoint[0], waypoint[1])
        if self.pizza_count == self.pizza_max and self.pizza_spawn_count >= self.pizza_max:
            self.waypoint = [waypoint]
        else:
            self.waypoint.append(waypoint)

    def pose_cb(self, msg):
        self.robot_pose = [msg.x, msg.y, msg.theta]
    
    def _cmd_vel(self, vx, wz):
        data = Twist()
        data.linear.x = vx
        data.angular.z = wz
        self.cmd_vel_pub.publish(data)
    
    def go_eat(self, state):
        data = Bool()
        data.data = state
        self.eat_pub.publish(data)
    
    def timer_cb(self):
        if self.robot_pose is None:return
        if len(self.waypoint) == 0:
            self._cmd_vel(0.0,0.0)

            if self.pizza_count == self.pizza_max:
                self.go_eat(True)
            elif self.pizza_count < self.pizza_max:
                self.go_eat(False)
            return
        
        delta_x = self.waypoint[0][0] - self.robot_pose[0]
        delta_y = self.waypoint[0][1] - self.robot_pose[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)

        goal_theta = math.atan2(delta_y , delta_x)
        error_theta = goal_theta - self.robot_pose[2]
        theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
        vx = distance * self.kp_linear
        wz = theta * self.kp_angular
        self._cmd_vel(vx, wz)

        if distance < 0.1:
            self._cmd_vel(0.0,0.0)
            if self.pizza_count != self.pizza_max:
                self.waypoint.pop(0)
                if self.pizza_spawn_count <= self.pizza_max:
                    self.eat_pizza()
        
        if self.pizza_count == self.pizza_max:
            self.go_eat(True)
        elif self.pizza_count < self.pizza_max:
            self.go_eat(False)

        return

def main(args=None):
    rclpy.init(args=args)
    node = eater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
