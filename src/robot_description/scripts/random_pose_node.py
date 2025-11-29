#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import random

try:
    from robot_service.srv import GetRandomPose
    SERVICES_AVAILABLE = True
except ImportError:
    SERVICES_AVAILABLE = False

class RandomPoseNode(Node):
    def __init__(self):
        super().__init__('random_pose_node')
        
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        
        if SERVICES_AVAILABLE:
            self.random_pose_srv = self.create_service(
                GetRandomPose, 'get_random_pose', self.get_random_pose_callback)
        
        self.get_logger().info("Random Pose Node initialized")
    
    def generate_random_pose(self):
        try:
            import roboticstoolbox as rtb
            from spatialmath import SE3
            
            L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
            L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
            L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
            tool = SE3(0.28, 0, 0)
            robot = rtb.DHRobot([L1, L2, L3], name='random_pose_robot', tool=tool)
            
            max_attempts = 50
            for attempt in range(max_attempts):
                try:
                    if random.random() < 0.6:
                        q1 = np.random.uniform(-np.pi/3, np.pi/3)
                        q2 = np.random.uniform(-np.pi/3, np.pi/3)
                        q3 = np.random.uniform(-np.pi/3, np.pi/3)
                    else:
                        q1 = np.random.uniform(-np.pi/2 * 0.8, np.pi/2 * 0.8)
                        q2 = np.random.uniform(-np.pi/2 * 0.8, np.pi/2 * 0.8)
                        q3 = np.random.uniform(-np.pi/2 * 0.8, np.pi/2 * 0.8)
                    
                    q = [q1, q2, q3]
                    fk = robot.fkine(q)
                    pos = fk.t
                    
                    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
                    
                    if not self.is_pose_safe(x, y, z, q1, q2, q3):
                        continue
                    
                    from geometry_msgs.msg import Pose
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation.w = 1.0
                    
                    return pose
                    
                except Exception:
                    continue
            
        except ImportError:
            pass
        
        # Fallback
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = 0.35
        pose.position.y = 0.0
        pose.position.z = 0.35
        pose.orientation.w = 1.0
        return pose
    
    def is_pose_safe(self, x, y, z, q1, q2, q3):
        base_z = 0.2
        
        if z < base_z + 0.08:
            return False
        if z > base_z + 0.40:
            return False
        
        radius = np.sqrt(x**2 + y**2)
        if radius < 0.15:
            return False
        if radius > 0.52:
            return False
        
        for q in [q1, q2, q3]:
            if abs(q) > np.pi/2 * 0.80:
                return False
        
        extreme_count = sum(1 for q in [q1, q2, q3] if abs(q) > np.pi/2.5)
        if extreme_count >= 2:
            return False
        
        height_above_base = z - base_z
        
        if height_above_base > 0.20 and radius < 0.20:
            return False
        
        if height_above_base > 0.30 and radius < 0.28:
            return False
        
        if height_above_base > 0.32 and radius > 0.45:
            return False
        
        distance_3d = np.sqrt(x**2 + y**2 + (z - base_z)**2)
        if distance_3d > 0.48:
            return False
        
        workspace_center_radius = 0.32
        workspace_center_height = base_z + 0.22
        
        distance_from_center = np.sqrt((radius - workspace_center_radius)**2 + 
                                       (z - workspace_center_height)**2)
        
        if distance_from_center > 0.18:
            if random.random() < 0.30:
                return False
        
        return True
    
    def get_random_pose_callback(self, request, response):
        try:
            if request.request:
                random_pose = self.generate_random_pose()
                
                response.success = True
                response.target_pose = random_pose
                response.message = f"Pose generated"
                
                self.publish_pose_stamped(random_pose)
                
            else:
                response.success = False
                response.target_pose = self.generate_random_pose()
                response.message = "Request false"
                
        except Exception as e:
            response.success = False
            response.target_pose = self.generate_random_pose()
            response.message = f"Error: {str(e)}"
        
        return response
    
    def publish_pose_stamped(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "link_0"
        pose_stamped.pose = pose
        self.target_pub.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()