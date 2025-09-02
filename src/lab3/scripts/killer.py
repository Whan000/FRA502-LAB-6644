#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim.srv import Kill, Spawn
from controller_interfaces.srv import SetParam

import math

class killer(Node):
    def __init__(self):
        super().__init__('killer_node')
        self.name_space = self.get_namespace()
        self.robot_pose_1 = None
        self.robot_pose_2 = None
        self.kp_linear = 0.3
        self.kp_angular = 10.0
        self.is_kill = False
        self.eat_state = False

        self.declare_parameter('sampling_frequency', 30.0)
        self.declare_parameter('kill_turtle', "eater_turtle")
        self.freq = self.get_parameter('sampling_frequency').value
        self.kill_turtle_name = self.get_parameter('kill_turtle').value

        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name_space}/cmd_vel', 10)
        self.create_subscription(Pose, f'/{self.kill_turtle_name}/pose', self.pose1_callback, 10)
        self.create_subscription(Pose, f'{self.name_space}/pose', self.pose2_callback, 10)
        self.create_subscription(Bool, f'/{self.kill_turtle_name}/eat_status', self.eat_status_callback, 10)
        self.create_service(SetParam, f'{self.name_space}/set_param', self.set_control_cb)
        self.kill_client = self.create_client(Kill, '/remove_turtle')
        self.create_timer(1/self.freq, self.timer_callback)
        self.get_logger().info(f"{self.name_space} is running at: {self.freq} Hz")

    def set_control_cb(self, request, response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        self.get_logger().info(f"kp_linear set to: {self.kp_linear} kp_angular set to: {self.kp_angular}")
        return response
    
    def kill_turtle(self):
        req = Kill.Request()
        req.name = self.kill_turtle_name
        self.kill_client.call_async(req)
        self.is_kill = True

    def pose1_callback(self, msg):
        self.robot_pose_1 = [msg.x, msg.y, msg.theta]

    def pose2_callback(self, msg):
        self.robot_pose_2 = [msg.x, msg.y, msg.theta]
    
    def eat_status_callback(self, msg):
        self.eat_state = msg.data
    
    def _cmd_vel(self, vx, wz):
        data = Twist()
        data.linear.x = vx
        data.angular.z = wz
        self.cmd_vel_pub.publish(data)
    
    def timer_callback(self):
        if self.robot_pose_1 is None or self.robot_pose_2 is None:return
        
        if self.eat_state:
            delta_x = self.robot_pose_1[0] - self.robot_pose_2[0]
            delta_y = self.robot_pose_1[1] - self.robot_pose_2[1]
            distance = math.sqrt(delta_x**2 + delta_y**2)

            goal_theta = math.atan2(delta_y , delta_x)
            error_theta = goal_theta - self.robot_pose_2[2]
            theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

            vx =  distance * self.kp_linear
            wz = theta * self.kp_angular

            self._cmd_vel(vx, wz)

            if distance < 0.04:
                self.kill_turtle()
                self._cmd_vel(0.0,0.0)
                return
        else:
            self._cmd_vel(0.0,0.0)
            
        if self.is_kill:
            self._cmd_vel(0.0,0.0)
    


def main(args=None):
    rclpy.init(args=args)
    node = killer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
