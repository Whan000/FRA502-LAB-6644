#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from tf_transformations import quaternion_from_euler
import time
import threading

try:
    from robot_service.srv import SetMode, InverseKinematics, GetRandomPose
    SERVICES_AVAILABLE = True
except ImportError:
    SERVICES_AVAILABLE = False

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Parameters
        self.declare_parameter('am_speed_multiplier', 1.0)  # 1.0x = default speed
        self.declare_parameter('am_max_joint_step', 0.60)  # Increased 5x to compensate for 1.0x default
        self.declare_parameter('r_min', 0.010)
        self.declare_parameter('r_max', 0.550)
        self.declare_parameter('z_min', -0.350)
        self.declare_parameter('z_max', 0.750)
        
        self.am_speed = self.get_parameter('am_speed_multiplier').get_parameter_value().double_value
        self.am_joint_step = self.get_parameter('am_max_joint_step').get_parameter_value().double_value
        self.r_min = self.get_parameter('r_min').get_parameter_value().double_value
        self.r_max = self.get_parameter('r_max').get_parameter_value().double_value
        self.z_min = self.get_parameter('z_min').get_parameter_value().double_value
        self.z_max = self.get_parameter('z_max').get_parameter_value().double_value

        # Robot model
        L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
        L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
        L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
        tool = SE3(0.28, 0, 0)
        self.robot = rtb.DHRobot([L1, L2, L3], name='RRR_robot', tool=tool)

        # State variables
        self.q = np.zeros(self.robot.n)
        self.target_pose = None
        self.last_target_pose = None 
        self.control_mode = None
        self.delta_q = 0.0
        self.task_space_velocity = np.zeros(3)
        self.start_time = None
        self.waiting_for_new_pose = False
        self.move = False
        self.hz = 1000.0

        # Auto mode state
        self.auto_mode_active = False
        self.auto_mode_thread = None
        self.am_move_count = 0
        self.last_error = None
        self.stuck_counter = 0
        self.stuck_threshold = 3000
        self.service_request_pending = False
        self.last_request_time = 0
        self.min_request_interval = 0.5
        self.target_reached = False
        self.in_singularity = False

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.end_effector_pub = self.create_publisher(PoseStamped, '/end_effector', 10)
        self.singularity_pub = self.create_publisher(Bool, '/singularity_warning', 10)
        
        # Subscribers
        self.velocity_sub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.mode_sub = self.create_subscription(String, 'control_mode', self.mode_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.gui_speed_sub = self.create_subscription(String, 'gui_speed', self.gui_speed_callback, 10)

        # Services
        if SERVICES_AVAILABLE:
            self.set_mode_srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
            self.ik_srv = self.create_service(InverseKinematics, 'inverse_kinematics', self.ik_callback)
            self.random_pose_client = self.create_client(GetRandomPose, 'get_random_pose')

        # Timer
        self.create_timer(1.0 / self.hz, self.timer_callback)

        # Initialize
        self.q = np.radians([0, 0, 0])
        self.publish_joints()
        self.get_logger().info("Controller initialized")

    def set_mode_callback(self, request, response):
        mode = request.mode
        
        if mode == 'IPK':
            self.control_mode = 'IPK'
            self.check_singularity_and_warn()
            response.success = True
            response.message = "IPK Mode activated"

        elif mode == 'TO_WF' or mode == 'TO':
            self.control_mode = 'TO_WF'
            self.move = True
            self.check_singularity_and_warn()
            response.success = True
            response.message = "TO_WF Mode activated"

        elif mode == 'TO_EF':
            self.control_mode = 'TO_EF'
            self.move = True
            self.check_singularity_and_warn()
            response.success = True
            response.message = "TO_EF Mode activated"

        elif mode == 'AM':
            if self.auto_mode_active:
                self.stop_auto_mode()
            
            self.control_mode = 'AM'
            self.check_singularity_and_warn()
            self.start_auto_mode()
            response.success = True
            response.message = "AM Mode started"

        elif mode == 'MANUAL':
            self.control_mode = None
            self.stop_auto_mode()
            self.move = False
            self.check_singularity_and_warn()
            response.success = True
            response.message = "MANUAL Mode activated"

        else:
            response.success = False
            response.message = f"Unknown mode: {mode}"

        return response

    def ik_callback(self, request, response):
        try:
            pose = request.target_pose
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            
            target_pose = SE3([x, y, z])
            success = self.compute_ik_solution(target_pose)
            
            if success:
                response.success = True
                response.joint_positions = self.q.tolist()
                response.message = "IK success"
                
                self.set_ipk_target(x, y, z)
                if self.control_mode == "MANUAL" or self.control_mode is None:
                    self.control_mode = "IPK"
                
                self.publish_pose(x, y, z)
            else:
                response.success = False
                response.joint_positions = []
                response.message = "No IK solution"
                
        except Exception as e:
            response.success = False
            response.joint_positions = []
            response.message = f"IK error: {str(e)}"
        
        return response

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.control_mode:
            if new_mode == "TO":
                self.control_mode = "TO_WF"
            else:
                self.control_mode = new_mode
            
            if new_mode not in ["AM"] and new_mode == "MANUAL":
                self.stop_auto_mode()

    def solve_ipk_target(self):
        if self.target_pose is None:
            return
        
        try:
            target_position = self.target_pose.t.flatten()
            success = self.compute_ik_solution(self.target_pose)
            
            if success:
                self.ipk_target_joints = self.q.copy()
                self.last_ipk_target = target_position.copy()
                
        except Exception as e:
            self.get_logger().error(f"IPK error: {e}")

    def set_ipk_target(self, x, y, z):
        self.target_pose = SE3([x, y, z])

    def start_auto_mode(self):
        self.auto_mode_active = True
        self.waiting_for_new_pose = False
        self.start_time = None
        self.am_startup_requested = False
        self.am_move_count = 0
        self.last_error = None
        self.stuck_counter = 0
        self.service_request_pending = False
        self.last_request_time = 0
        self.target_reached = False
        
        import threading
        threading.Timer(0.5, lambda: self.request_random_pose()).start()

    def stop_auto_mode(self):
        self.auto_mode_active = False
        self.waiting_for_new_pose = False
        self.start_time = None
        self.service_request_pending = False

    def request_random_pose(self):
        if self.service_request_pending:
            return
        
        current_time = time.time()
        time_since_last = current_time - self.last_request_time
        if time_since_last < self.min_request_interval:
            return
        
        if not self.random_pose_client.service_is_ready():
            return

        self.service_request_pending = True
        self.last_request_time = current_time
        
        request = GetRandomPose.Request()
        request.request = True
        
        future = self.random_pose_client.call_async(request)
        future.add_done_callback(self.handle_random_pose_response)

    def handle_random_pose_response(self, future):
        try:
            self.service_request_pending = False
            
            response = future.result()
            if response and response.success:
                random_pose = response.target_pose
                
                # Check minimum distance
                fk_pose = self.robot.fkine(self.q)
                current_pos = fk_pose.t.flatten()
                target_pos = np.array([random_pose.position.x, random_pose.position.y, random_pose.position.z])
                distance = np.linalg.norm(target_pos - current_pos)
                
                if distance < 0.05:
                    import threading
                    threading.Timer(0.5, lambda: self.request_random_pose()).start()
                    return
                
                self.target_pose = SE3([random_pose.position.x, random_pose.position.y, random_pose.position.z])
                self.start_time = time.time()
                self.waiting_for_new_pose = True
                self.last_error = None
                self.stuck_counter = 0
                self.target_reached = False  # Reset for new target
                
                self.publish_pose(random_pose.position.x, random_pose.position.y, random_pose.position.z)
                
            else:
                import threading
                threading.Timer(1.0, lambda: self.request_random_pose()).start()
                
        except Exception as e:
            self.service_request_pending = False
            import threading
            threading.Timer(1.0, lambda: self.request_random_pose()).start()

    def compute_ik_solution(self, target_pose):
        success = False
        for _ in range(5):
            q0 = np.random.uniform(self.robot.qlim[0], self.robot.qlim[1])
            sol = self.robot.ikine_LM(target_pose, q0=q0, mask=[1, 1, 1, 0, 0, 0], tol=1e-5)
            if sol.success:
                self.q = sol.q
                success = True
                break
        return success

    def timer_callback(self):
        if self.control_mode is None:
            self.publish_joints()
            return
        
        # Check singularity continuously in all modes
        self.check_singularity_and_warn()

        if self.control_mode == 'IPK' and self.target_pose is not None:
            fk_pose = self.robot.fkine(self.q)
            current_position = fk_pose.t.flatten()
            target_position = self.target_pose.t.flatten()

            if not hasattr(self, 'ipk_target_joints') or not hasattr(self, 'last_ipk_target'):
                self.solve_ipk_target()
            elif not np.allclose(target_position, getattr(self, 'last_ipk_target', target_position), atol=0.0001):
                self.solve_ipk_target()
            
            if hasattr(self, 'ipk_target_joints'):
                # Check if already at target (0.1mm tolerance - very strict!)
                position_error = np.linalg.norm(target_position - current_position)
                
                if position_error > 0.0001:  # Only move if error > 0.1mm
                    joint_error = self.ipk_target_joints - self.q
                    max_joint_step = 0.05
                    joint_step = np.clip(joint_error, -max_joint_step, max_joint_step)
                    new_q = self.q + joint_step
                    
                    q_min = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
                    q_max = np.array([np.pi/2, np.pi/2, np.pi/2])
                    self.q = np.clip(new_q, q_min, q_max)
            
            self.publish_joints()

        elif self.control_mode == 'AM' and self.auto_mode_active:
            if self.target_pose is not None:
                fk_pose = self.robot.fkine(self.q)
                current_position = fk_pose.t.flatten()
                target_position = self.target_pose.t.flatten()

                delta_x = target_position - current_position 
                position_error = np.linalg.norm(delta_x)
                
                # Stuck detection
                if self.last_error is not None:
                    error_change = abs(position_error - self.last_error)
                    
                    if error_change < 0.001:
                        self.stuck_counter += 1
                    else:
                        self.stuck_counter = 0
                    
                    if self.stuck_counter >= self.stuck_threshold:
                        self.stuck_counter = 0
                        self.am_move_count += 1
                        import threading
                        threading.Timer(0.5, lambda: self.request_random_pose()).start()
                        self.publish_joints()
                        return
                
                self.last_error = position_error
                
                J = self.robot.jacob0(self.q)
                J_trans = J[0:3, :]

                # Singularity check
                manipulability = np.sqrt(np.linalg.det(J_trans @ J_trans.T))
                if manipulability < 5e-5:
                    import threading
                    threading.Timer(0.5, lambda: self.request_random_pose()).start()
                    return

                # Velocity control - fixed base gain, GUI speed in integration
                base_velocity_gain = 2.5
                
                if position_error < 0.001:
                    velocity_gain = base_velocity_gain * 0.1
                elif position_error < 0.005:
                    velocity_gain = base_velocity_gain * 0.3
                else:
                    velocity_gain = base_velocity_gain
                
                desired_velocity = delta_x * velocity_gain
                desired_delta_q = np.linalg.pinv(J_trans) @ desired_velocity
                
                joint_step = np.clip(desired_delta_q, -self.am_joint_step, self.am_joint_step)
                new_q = self.q + joint_step * (self.am_speed / self.hz)
                
                q_min = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
                q_max = np.array([np.pi/2, np.pi/2, np.pi/2])
                self.q = np.clip(new_q, q_min, q_max)
                
                # Check if reached (2mm tolerance)
                if position_error < 0.002 and not self.target_reached:
                    self.target_reached = True  # Mark as reached to prevent duplicates
                    self.am_move_count += 1
                    self.waiting_for_new_pose = False
                    import threading
                    threading.Timer(0.5, lambda: self.request_random_pose()).start()
                
                # Check 10-second timeout
                if self.start_time is not None:
                    elapsed_time = time.time() - self.start_time
                    if elapsed_time >= 10.0 and not self.target_reached:
                        self.target_reached = True  # Mark to prevent duplicate timeout logs
                        self.am_move_count += 1
                        self.waiting_for_new_pose = False
                        import threading
                        threading.Timer(0.5, lambda: self.request_random_pose()).start()
                
                self.publish_joints()
            else:
                if not hasattr(self, 'am_startup_requested') or not self.am_startup_requested:
                    self.am_startup_requested = True
                self.publish_joints()
        
        elif self.control_mode in ['TO_WF', 'TO', 'TO_EF'] and self.task_space_velocity.any():
            J = self.robot.jacob0(self.q)
            J_trans = J[0:3, :]
            
            if self.check_singularity_and_warn():
                return
            
            fk_pose = self.robot.fkine(self.q)
            current_x, current_y, current_z = fk_pose.t.flatten()
            
            # Check if at workspace boundary - if so, STOP all motion
            if self.at_workspace_boundary(current_x, current_y, current_z):
                return  # Stop completely at boundary, no sliding!

            if self.control_mode == 'TO_EF':
                R = fk_pose.R
                desired_velocity_world = R @ self.task_space_velocity
            else:
                desired_velocity_world = self.task_space_velocity

            try:
                desired_delta_q = np.linalg.pinv(J_trans) @ desired_velocity_world
            except:
                return

            max_joint_step = 0.1
            joint_step = np.clip(desired_delta_q, -max_joint_step, max_joint_step)
            new_q = self.q + joint_step / self.hz
            
            q_min = np.array([-np.pi/2, -np.pi/2, -np.pi/2])
            q_max = np.array([np.pi/2, np.pi/2, np.pi/2])
            
            fk_next = self.robot.fkine(new_q)
            next_x, next_y, next_z = fk_next.t.flatten()
            
            # Double check: if next position out of workspace, don't move
            if not self.in_workspace(next_x, next_y, next_z):
                return
            
            new_q_clamped = np.clip(new_q, q_min, q_max)
            
            if not np.allclose(new_q, new_q_clamped, atol=1e-6):
                self.q = new_q_clamped
            else:
                self.q = new_q

            self.publish_joints()

    def publish_joints(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['base_to_link1', 'link1_link2', 'link2_link3']
        joint_msg.position = self.q.tolist()
        self.joint_pub.publish(joint_msg)
        
        try:
            fk_pose = self.robot.fkine(self.q)
            ee_pos = fk_pose.t.flatten()
            
            ee_msg = PoseStamped()
            ee_msg.header.stamp = joint_msg.header.stamp
            ee_msg.header.frame_id = "link_0"
            ee_msg.pose.position.x = float(ee_pos[0])
            ee_msg.pose.position.y = float(ee_pos[1])
            ee_msg.pose.position.z = float(ee_pos[2])
            ee_msg.pose.orientation.w = 1.0
            
            self.end_effector_pub.publish(ee_msg)
        except:
            pass

    def publish_pose(self, x, y, z):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "link_0"
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        pose_msg.pose.orientation.w = 1.0
        self.target_pub.publish(pose_msg)

    def joint_state_callback(self, msg):
        if len(msg.position) >= 3 and self.control_mode is None:
            self.q = np.array(msg.position[:3])

    def velocity_callback(self, msg):
        self.task_space_velocity = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    
    def gui_speed_callback(self, msg):
        try:
            new_speed = float(msg.data)
            self.am_speed = new_speed
        except ValueError:
            pass
    
    def in_workspace(self, x, y, z):
        rho2 = x**2 + y**2
        radius_check = (self.r_min**2 <= rho2 <= self.r_max**2)
        height_check = (self.z_min <= z <= self.z_max)
        return radius_check and height_check
    
    def at_workspace_boundary(self, x, y, z, margin=0.0001):
        """Check if at actual workspace boundary (0.1mm tolerance - very strict!)"""
        radius = np.sqrt(x**2 + y**2)
        
        # Check if at any boundary (within 0.1mm)
        at_min_radius = abs(radius - self.r_min) < margin
        at_max_radius = abs(radius - self.r_max) < margin
        at_min_height = abs(z - self.z_min) < margin
        at_max_height = abs(z - self.z_max) < margin
        
        return at_min_radius or at_max_radius or at_min_height or at_max_height

    def check_singularity_and_warn(self, manipulability_threshold=1e-3):
        J = self.robot.jacob0(self.q)
        J_trans = J[0:3, :]
        manipulability_index = np.sqrt(np.linalg.det(J_trans.T @ J_trans))
        
        singularity_msg = Bool()
        
        if manipulability_index < manipulability_threshold:
            singularity_msg.data = True
            self.singularity_pub.publish(singularity_msg)
            
            if not self.in_singularity:
                self.in_singularity = True
                print(f"\rWARNING: Robot approaching singularity - cannot continue (manipulability: {manipulability_index:.6f})", flush=True)
            
            return True
        else:
            singularity_msg.data = False
            self.singularity_pub.publish(singularity_msg)
            
            if self.in_singularity:
                self.in_singularity = False
                print(f"\rRobot cleared singularity region (manipulability: {manipulability_index:.6f})              ", flush=True)
            
            return False

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()