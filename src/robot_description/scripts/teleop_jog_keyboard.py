#!/usr/bin/env python3

"""
Teleop Jog Keyboard Node - LAB 4 Complete Implementation
Manual keyboard control for robot teleoperation
Supports both TO_WF (World Frame) and TO_EF (End Effector Frame) modes
Single keypress control without Enter confirmation
"""

import sys
import select
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Import robot service for mode switching
try:
    from robot_service.srv import SetMode
    SERVICES_AVAILABLE = True
except ImportError:
    SERVICES_AVAILABLE = False
    print("Warning: SetMode service not available - frame switching disabled")

class TeleopJogKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_jog_keyboard')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, 'control_mode', 10)
        
        # Service client for mode switching
        if SERVICES_AVAILABLE:
            self.set_mode_client = self.create_client(SetMode, 'set_mode')
        
        # State variables
        self.current_frame = "WORLD"  # "WORLD" or "END_EFFECTOR"
        self.current_velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.velocity_step = 0.01  # m/s per keypress
        self.max_velocity = 0.1   # m/s maximum
        self.velocity_decay = 0.85  # Decay factor for safety
        self.running = True
        self.update_counter = 0  # Counter for less frequent updates
        
        # Setup terminal for single keypress input
        self.setup_terminal()
        
        # Velocity publishing timer
        self.publish_timer = self.create_timer(0.05, self.publish_velocity)  # 20Hz
        
        # Print instructions
        self.print_instructions()
        
        # Initialize to TO_WF mode
        self.set_controller_mode("TO_WF")
        
        # Show initial status
        print("Press keys for immediate control:")
        self.update_status_display()  # Set initial status display

    def setup_terminal(self):
        """Setup terminal for single keypress input"""
        try:
            # Save original terminal settings
            self.old_settings = termios.tcgetattr(sys.stdin)
            
            # Create new settings for raw input
            new_settings = termios.tcgetattr(sys.stdin)
            new_settings[3] &= ~(termios.ICANON | termios.ECHO)  # Disable canonical mode and echo
            new_settings[6][termios.VMIN] = 1  # Minimum characters to read
            new_settings[6][termios.VTIME] = 0  # Timeout (0 = no timeout)
            
            # Apply new settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
            
        except Exception:
            self.old_settings = None  # Silent fallback

    def restore_terminal(self):
        """Restore original terminal settings"""
        try:
            if self.old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass  # Silent restore

    def print_instructions(self):
        """Print keyboard control instructions"""
        instructions = """

                    TELEOP JOG KEYBOARD CONTROL - LAB 4               
                         TO_WF + TO_EF SUPPORT                        

  CONTROLS: w/s(±X) a/d(±Y) q/e(±Z) f(toggle) +/-(speed) 0(stop) x(quit) 


"""
        print(instructions)

    def update_status_display(self):
        """Update status on a single line in place"""
        mode_name = "TO_WF" if self.current_frame == "WORLD" else "TO_EF"
        
        # Update status line in place using \r to return to beginning of line
        status = f"Mode: {mode_name}   Velocity: [{self.current_velocity[0]:+.3f}, {self.current_velocity[1]:+.3f}, {self.current_velocity[2]:+.3f}]"
        print(f"\r{status:<75}", end="", flush=True)  # Pad with spaces to clear old text

    def set_controller_mode(self, mode):
        """Set controller mode via service call"""
        if not SERVICES_AVAILABLE:
            return
            
        if not self.set_mode_client.service_is_ready():
            return
        
        try:
            request = SetMode.Request()
            request.mode = mode
            
            future = self.set_mode_client.call_async(request)
            # Don't block on response for responsive control
            
        except Exception as e:
            pass  # Silent operation for clean display

    def toggle_reference_frame(self):
        """Toggle between World Frame and End Effector Frame"""
        if self.current_frame == "WORLD":
            self.current_frame = "END_EFFECTOR"
            new_mode = "TO_EF"
        else:
            self.current_frame = "WORLD"
            new_mode = "TO_WF"
        
        # Set new controller mode
        self.set_controller_mode(new_mode)
        
        # Zero velocities when switching frames for safety
        self.current_velocity = [0.0, 0.0, 0.0]
        self.publish_zero_velocity()
        
        self.update_status_display()

    def emergency_stop(self):
        """Emergency stop - zero all velocities"""
        self.current_velocity = [0.0, 0.0, 0.0]
        self.publish_zero_velocity()
        self.update_status_display()

    def adjust_velocity_step(self, increase=True):
        """Adjust velocity step size"""
        if increase:
            self.velocity_step = min(self.velocity_step + 0.005, self.max_velocity)
        else:
            self.velocity_step = max(self.velocity_step - 0.005, 0.005)
        
        # No logging - just continue with clean display

    def get_key(self):
        """Get single keypress without blocking"""
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
                return key
            return None
        except Exception:
            return None

    def process_key(self, key):
        """Process keyboard input and update velocities"""
        # Movement controls - silent operation
        if key == 'w':
            self.current_velocity[0] += self.velocity_step  # +X
        elif key == 's':
            self.current_velocity[0] -= self.velocity_step  # -X
        elif key == 'a':
            self.current_velocity[1] += self.velocity_step  # +Y  
        elif key == 'd':
            self.current_velocity[1] -= self.velocity_step  # -Y
        elif key == 'q':
            self.current_velocity[2] += self.velocity_step  # +Z
        elif key == 'e':
            self.current_velocity[2] -= self.velocity_step  # -Z
        
        # Frame control
        elif key == 'f':
            self.toggle_reference_frame()
            return True
        
        # Speed control
        elif key == '+' or key == '=':
            self.adjust_velocity_step(increase=True)
            return True
        elif key == '-' or key == '_':
            self.adjust_velocity_step(increase=False)
            return True
        
        # Emergency stop
        elif key == '0' or key == ' ':
            self.emergency_stop()
            return True
        
        # Quit
        elif key == '\x1b' or key == 'x':  # ESC or 'x' key
            self.running = False
            return False
        
        # Clamp velocities to limits
        for i in range(3):
            self.current_velocity[i] = max(-self.max_velocity, 
                                         min(self.max_velocity, self.current_velocity[i]))
        
        # Update status display immediately on keypress
        self.update_status_display()
        return True

    def publish_velocity(self):
        """Publish current velocity and apply decay"""
        if not self.running:
            return
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.current_velocity[0]
        twist.linear.y = self.current_velocity[1]
        twist.linear.z = self.current_velocity[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # Apply velocity decay for safety
        for i in range(3):
            if abs(self.current_velocity[i]) > 0.001:  # Only decay non-zero velocities
                self.current_velocity[i] *= self.velocity_decay
            else:
                self.current_velocity[i] = 0.0  # Stop very small velocities
        
        # Update status display every 5 cycles (smooth updates)
        self.update_counter += 1
        if self.update_counter % 5 == 0:
            self.update_status_display()
    
    def publish_zero_velocity(self):
        """Publish zero velocity immediately"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Main control loop"""
        try:
            while self.running and rclpy.ok():
                # Check for keyboard input
                key = self.get_key()
                if key:
                    if not self.process_key(key):
                        break
                
                # Spin ROS once
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            pass  # Silent shutdown
        
        finally:
            # Restore terminal settings
            self.restore_terminal()
            
            # Send final zero velocity
            self.current_velocity = [0.0, 0.0, 0.0]
            self.publish_zero_velocity()
            
            print(f"\n\nTeleop stopped.")  # Simple exit message

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopJogKeyboard()
        teleop_node.run()
        
    except Exception:
        pass  # Silent operation
    
    finally:
        try:
            teleop_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()