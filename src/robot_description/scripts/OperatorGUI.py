#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tkinter as tk
from tkinter import ttk
from math import pi
import time
import numpy as np
import struct

# Import your custom services (uncomment when services are ready)
try:
    from robot_service.srv import SetMode, InverseKinematics, GetRandomPose
    SERVICES_AVAILABLE = True
except ImportError:
    SERVICES_AVAILABLE = False
    print("Warning: Custom services not found. GUI will work in manual mode only.")

class AdvancedOperatorGUI(Node):
    def __init__(self):
        super().__init__('advanced_operator_gui')
        
        # ROS Publishers
        self.joint_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.mode_publisher_ = self.create_publisher(String, 'control_mode', 10)
        
        # NEW: Speed control publisher for AM mode
        self.speed_publisher_ = self.create_publisher(String, 'gui_speed', 10)
        
        # Workspace visualization publisher
        self.workspace_pub = self.create_publisher(PointCloud2, '/robot_workspace', 10)
        
        # Ghost trail publisher (using Path for smooth visualization)
        self.trail_pub = self.create_publisher(Path, '/end_effector_path', 10)
        
        # Teleoperation publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ROS Subscribers
        self.target_subscriber = self.create_subscription(
            PoseStamped, '/target', self.target_callback, 10)
        self.end_effector_subscriber = self.create_subscription(
            PoseStamped, '/end_effector', self.end_effector_callback, 10)
        
        # Singularity warning subscriber
        from std_msgs.msg import Bool
        self.singularity_subscriber = self.create_subscription(
            Bool, '/singularity_warning', self.singularity_callback, 10)
        
        # Service clients (only if services available)
        if SERVICES_AVAILABLE:
            self.set_mode_client = self.create_client(SetMode, 'set_mode')
            self.ik_client = self.create_client(InverseKinematics, 'inverse_kinematics')
            self.random_pose_client = self.create_client(GetRandomPose, 'get_random_pose')
            
            # Wait for services
            self.get_logger().info("Waiting for services...")
            # Don't block GUI startup on missing services
        
        # Joint Configuration
        self.joints = [
            ("base_to_link1", 0.0, "BASE ROTATION", "J1"),
            ("link1_link2", 0.0, "SHOULDER JOINT", "J2"),
            ("link2_link3", 0.0, "ELBOW JOINT", "J3")
        ]
        
        # Control State
        self.sliders = {}
        self.position_displays = {}
        self.degree_displays = {}
        self.current_positions = [0.0, 0.0, 0.0]
        self.current_mode = "MANUAL"
        self.target_pose = None
        self.end_effector_pose = None
        self.singularity_active = False  # Track singularity warnings
        
        # Reference frame state for teleoperation
        self.reference_frame = "WORLD"  # "WORLD" or "END_EFFECTOR"
        self.to_active = False  # Teleoperation active state
        
        # Ghost trail functionality
        self.trail_active = False
        self.trail_points = []
        self.max_trail_points = 1000  # Maximum trail length
        self.trail_publish_rate = 1  # Publish every single point
        self.trail_counter = 0
        
        self.setup_gui()
        
        # Publish initial mode
        initial_mode = String()
        initial_mode.data = "MANUAL"
        self.mode_publisher_.publish(initial_mode)
        
        # Publish initial speed
        initial_speed = String()
        initial_speed.data = "1.0"
        self.speed_publisher_.publish(initial_speed)
        
        # Start update loop
        self.root.after(50, self.update_loop)
    
    def target_callback(self, msg):
        """Callback for target pose updates"""
        self.target_pose = msg
        
    def end_effector_callback(self, msg):
        """Callback for end effector pose updates"""
        self.end_effector_pose = msg
        
        # Record position for ghost trail
        self.record_end_effector_position()
    
    def singularity_callback(self, msg):
        """Callback for singularity warnings - responds to real-time status"""
        if msg.data:  # Singularity detected
            self.singularity_active = True
            self.singularity_display.config(text=" WARNING", fg=self.colors['warn'])
        else:  # Safe to move
            self.singularity_active = False
            self.singularity_display.config(text="SAFE", fg=self.colors['text'])
        
    def reset_singularity_warning(self):
        """Reset singularity warning display"""
        self.singularity_active = False
        self.singularity_display.config(text="SAFE", fg=self.colors['text'])
    
    def setup_gui(self):
        """Create the simplified control GUI"""
        self.root = tk.Tk()
        self.root.title("Operator GUI")
        self.root.geometry("1200x650")
        self.root.minsize(1100, 600)  # Set minimum window size
        self.root.configure(bg='#2b2b2b')
        
        # Simplified color scheme
        self.colors = {
            'bg_dark': '#2b2b2b',
            'bg_panel': '#3a3a3a',
            'bg_control': '#3a3a3a',
            'text': '#00ff00',
            'text_alt': '#ffff00',
            'warn': '#ff0000',
            'accent': '#00ffff',
            'border': '#5a5a5a'
        }
        
        # Style configuration
        self.setup_styles()
        
        # Header bar
        self.build_header()
        
        # Main container
        main_frame = tk.Frame(self.root, bg=self.colors['bg_dark'])
        main_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left Panel - Joint Controls
        left_panel = tk.Frame(main_frame, bg=self.colors['bg_panel'], 
                             relief='flat', bd=1)
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 3))
        
        # Right Panel - Control Features
        right_panel = tk.Frame(main_frame, bg=self.colors['bg_panel'], 
                              relief='flat', bd=1)
        right_panel.pack(side='left', fill='both', expand=True, padx=(3, 0))
        
        # Build each panel
        self.build_joint_controls(left_panel)
        self.build_control_features(right_panel)
        
        # Status bar at bottom
        self.build_status_bar()
    
    def build_header(self):
        """Build the header bar"""
        header = tk.Frame(self.root, bg='#1a1a1a', height=50)
        header.pack(fill='x', side='top')
        header.pack_propagate(False)
        
        # Title
        title = tk.Label(header, text="3R ROBOT CONTROL",
                        font=('Courier', 16, 'bold'),
                        bg='#1a1a1a', fg=self.colors['text'])
        title.pack(side='left', padx=20, pady=10)
        
        # System time
        self.time_label = tk.Label(header, text="",
                                  font=('Courier', 11),
                                  bg='#1a1a1a', fg=self.colors['accent'])
        self.time_label.pack(side='right', padx=20)
        self.update_time()
    
    def update_time(self):
        """Update system time display"""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=f"SYS TIME: {current_time}")
        self.root.after(1000, self.update_time)
    
    def setup_styles(self):
        """Configure ttk styles for legacy look"""
        style = ttk.Style()
        style.theme_use('alt')
        
        # Standard button
        style.configure('Legacy.TButton', 
                       background='#4a4a4a', 
                       foreground='#ffffff',
                       borderwidth=2,
                       relief='raised',
                       focuscolor='none',
                       padding=8,
                       font=('Courier', 9, 'bold'))
        style.map('Legacy.TButton',
                 background=[('active', '#5a5a5a'), ('pressed', '#3a3a3a')])
        
        # Action button
        style.configure('Action.TButton',
                       background='#2a4a2a',
                       foreground='#00ff00',
                       borderwidth=3,
                       relief='raised',
                       focuscolor='none',
                       padding=10,
                       font=('Courier', 10, 'bold'))
        style.map('Action.TButton',
                 background=[('active', '#3a5a3a'), ('pressed', '#1a3a1a')])
    
    def build_joint_controls(self, parent):
        """Build the joint control panel with SPEED CONTROL"""
        # Title
        title = tk.Label(parent, text="JOINT CONTROL", 
                        font=('Courier', 12, 'bold'),
                        bg=self.colors['bg_panel'], fg=self.colors['text'])
        title.pack(pady=10)
        
        # Create controls for each joint
        for i, (name, default_val, display_name, joint_id) in enumerate(self.joints):
            self.create_joint_control(parent, i, name, default_val, display_name, joint_id)
        
        # ENHANCED Speed control with AM mode integration
        speed_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        speed_frame.pack(fill='x', padx=10, pady=10)
        
        speed_title = tk.Label(speed_frame, text="Speed Control",
                              font=('Courier', 10, 'bold'),
                              bg=self.colors['bg_control'], fg='#ffffff')
        speed_title.pack(pady=5)
        
        # MODIFIED: Extended range and connected to controller
        self.speed_slider = tk.Scale(speed_frame, from_=0.1, to=3.0,  # Extended to 3.0x
                                    orient='horizontal',
                                    resolution=0.1,
                                    bg=self.colors['bg_panel'], 
                                    fg=self.colors['text'],
                                    highlightthickness=0,
                                    troughcolor=self.colors['bg_dark'],
                                    font=('Courier', 8),
                                    length=300,
                                    showvalue=0)
        self.speed_slider.set(1.0)
        self.speed_slider.pack(padx=10, pady=5)
        
        self.speed_label = tk.Label(speed_frame, text="1.0x", 
                                    font=('Courier', 10, 'bold'),
                                    bg=self.colors['bg_control'], 
                                    fg=self.colors['text_alt'])
        self.speed_label.pack(pady=5)
        
        # CRITICAL CHANGE: Modified command to publish speed
        self.speed_slider.configure(command=self.on_speed_change)
        
        # Speed status indicator
        self.speed_status = tk.Label(speed_frame, text=" Real-time control active", 
                                    font=('Courier', 8),
                                    bg=self.colors['bg_control'], 
                                    fg=self.colors['accent'])
        self.speed_status.pack(pady=2)
        
        # Smooth motion toggle
        mode_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        mode_frame.pack(fill='x', padx=10, pady=5)
        
        self.interp_var = tk.BooleanVar(value=True)
        interp_check = tk.Checkbutton(mode_frame, text="Smooth Motion",
                                     variable=self.interp_var,
                                     font=('Courier', 9),
                                     bg=self.colors['bg_control'], 
                                     fg='#ffffff',
                                     selectcolor=self.colors['bg_panel'],
                                     activebackground=self.colors['bg_control'])
        interp_check.pack(anchor='w', padx=5, pady=2)
        
        # Status displays (moved from right panel for compact layout)
        # Current Mode
        mode_info_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        mode_info_frame.pack(fill='x', padx=10, pady=10)
        
        info_title = tk.Label(mode_info_frame, text="Current Mode:",
                             font=('Courier', 9),
                             bg=self.colors['bg_control'], fg='#aaaaaa')
        info_title.pack()
        
        self.mode_display = tk.Label(mode_info_frame, text="MANUAL",
                                     font=('Courier', 11, 'bold'),
                                     bg=self.colors['bg_control'], fg=self.colors['text_alt'])
        self.mode_display.pack(pady=5)
        
        # Service status
        service_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        service_frame.pack(fill='x', padx=10, pady=10)
        
        service_label = tk.Label(service_frame, text="Services:",
                               font=('Courier', 9),
                               bg=self.colors['bg_control'], fg='#aaaaaa')
        service_label.pack()
        
        service_status = "Available" if SERVICES_AVAILABLE else "Not Available"
        service_color = self.colors['text'] if SERVICES_AVAILABLE else self.colors['warn']
        
        self.service_display = tk.Label(service_frame, text=service_status,
                                       font=('Courier', 9, 'bold'),
                                       bg=self.colors['bg_control'], fg=service_color)
        self.service_display.pack()
        
        # Singularity warning display
        singularity_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        singularity_frame.pack(fill='x', padx=10, pady=10)
        
        singularity_label = tk.Label(singularity_frame, text="Singularity:",
                                    font=('Courier', 9),
                                    bg=self.colors['bg_control'], fg='#aaaaaa')
        singularity_label.pack()
        
        self.singularity_display = tk.Label(singularity_frame, text="SAFE",
                                           font=('Courier', 9, 'bold'),
                                           bg=self.colors['bg_control'], fg=self.colors['text'])
        self.singularity_display.pack()
    
    # NEW METHOD: Handle speed slider changes
    def on_speed_change(self, value):
        """Handle speed slider changes - update label AND publish to controller"""
        speed_value = float(value)
        
        # Update GUI label
        self.speed_label.config(text=f"{speed_value:.1f}x")
        
        # Publish speed to controller for real-time AM mode control
        speed_msg = String()
        speed_msg.data = str(speed_value)
        self.speed_publisher_.publish(speed_msg)
        
        # Log the speed change
        self.get_logger().info(f" Speed changed to {speed_value:.1f}x")
        
        # Update status to show speed control working
        if hasattr(self, 'service_response_label'):
            self.service_response_label.config(
                text=f" AM Speed: {speed_value:.1f}x (Real-time control active)",
                fg=self.colors['text']
            )
        
        # Update speed status indicator
        self.speed_status.config(text=f" Broadcasting {speed_value:.1f}x to robot")
    
    def create_joint_control(self, parent, index, name, default_val, display_name, joint_id):
        """Create a control panel for a single joint"""
        frame = tk.Frame(parent, bg=self.colors['bg_control'])
        frame.pack(fill='x', padx=10, pady=5)
        
        # Header with joint name
        header = tk.Frame(frame, bg=self.colors['bg_control'])
        header.pack(fill='x', pady=3)
        
        name_label = tk.Label(header, text=f"{joint_id} - {display_name}",
                             font=('Courier', 10, 'bold'),
                             bg=self.colors['bg_control'], fg='#ffffff')
        name_label.pack(side='left')
        
        # Position displays
        display_frame = tk.Frame(header, bg=self.colors['bg_control'])
        display_frame.pack(side='right')
        
        rad_display = tk.Label(display_frame, text=f"{default_val:+.3f} rad",
                              font=('Courier', 9),
                              bg=self.colors['bg_control'], fg=self.colors['text'],
                              width=12)
        rad_display.pack(side='left', padx=5)
        self.position_displays[name] = rad_display
        
        deg_display = tk.Label(display_frame, text=f"{default_val*180/pi:+.1f} deg",
                              font=('Courier', 9),
                              bg=self.colors['bg_control'], fg=self.colors['text_alt'],
                              width=12)
        deg_display.pack(side='left', padx=5)
        self.degree_displays[name] = deg_display
        
        # Slider
        slider = tk.Scale(frame, from_=-1.57, to=1.57,
                         orient='horizontal',
                         resolution=0.01,
                         bg=self.colors['bg_panel'], 
                         fg='#ffffff',
                         highlightthickness=0,
                         troughcolor='#2a2a2a',
                         sliderlength=30,
                         width=20,
                         font=('Courier', 7),
                         showvalue=0)
        slider.set(default_val)
        slider.pack(fill='x', pady=3)
        self.sliders[name] = slider
        
        # Quick adjustment buttons (in radians)
        btn_frame = tk.Frame(frame, bg=self.colors['bg_control'])
        btn_frame.pack(pady=3)
        
        adjustments = [
            ("-0.1", -0.1), ("-0.01", -0.01),
            ("ZERO", 0),
            ("+0.01", 0.01), ("+0.1", 0.1)
        ]
        
        for text, delta_rad in adjustments:
            if text == "ZERO":
                btn = ttk.Button(btn_frame, text=text,
                               command=lambda s=slider: s.set(0),
                               style='Action.TButton',
                               width=6)
            else:
                btn = ttk.Button(btn_frame, text=text,
                               command=lambda idx=index, d=delta_rad: self.adjust_joint(idx, d),
                               style='Legacy.TButton',
                               width=6)
            btn.pack(side='left', padx=2)
    
    def adjust_joint(self, joint_index, delta_rad):
        """Adjust a joint by a delta amount based on current actual position"""
        name = self.joints[joint_index][0]
        # Use actual robot position, not slider position
        current_actual = self.current_positions[joint_index]
        new_value = current_actual + delta_rad
        # Clamp to joint limits
        new_value = max(-1.57, min(1.57, new_value))
        self.sliders[name].set(new_value)
    
    def build_control_features(self, parent):
        """Build the control features panel"""
        # Title
        title = tk.Label(parent, text="CONTROL FEATURES",
                        font=('Courier', 12, 'bold'),
                        bg=self.colors['bg_panel'], fg=self.colors['text'])
        title.pack(pady=10)
        
        # Mode selection
        mode_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        mode_frame.pack(fill='x', padx=10, pady=10)
        
        mode_label = tk.Label(mode_frame, text="Select Mode:",
                             font=('Courier', 10, 'bold'),
                             bg=self.colors['bg_control'], fg='#ffffff')
        mode_label.pack(pady=5)
        
        self.mode_var = tk.StringVar(value="MANUAL")
        
        modes = [
            ("MANUAL", "Manual Joint Control"),
            ("IPK", "Inverse Position Kinematics"),
            ("TO", "Teleoperation (Velocity)"),
            ("AM", "Auto Mode")
        ]
        
        for mode_val, mode_desc in modes:
            radio_frame = tk.Frame(mode_frame, bg=self.colors['bg_control'])
            radio_frame.pack(fill='x', pady=2)
            
            radio = tk.Radiobutton(radio_frame, 
                                  text=mode_val,
                                  variable=self.mode_var,
                                  value=mode_val,
                                  font=('Courier', 9, 'bold'),
                                  bg=self.colors['bg_control'],
                                  fg=self.colors['text'],
                                  selectcolor=self.colors['bg_panel'],
                                  activebackground=self.colors['bg_control'],
                                  command=self.on_mode_change)
            radio.pack(side='left', padx=5)
            
            desc_label = tk.Label(radio_frame, text=mode_desc,
                                 font=('Courier', 8),
                                 bg=self.colors['bg_control'], fg='#aaaaaa')
            desc_label.pack(side='left', padx=5)
        
        # Mode control buttons
        btn_frame = tk.Frame(parent, bg=self.colors['bg_control'])
        btn_frame.pack(fill='x', padx=10, pady=10)
        
        # IPK Mode Controls
        ipk_frame = tk.LabelFrame(parent, text="IPK MODE", 
                                font=('Courier', 9, 'bold'),
                                bg=self.colors['bg_control'], fg=self.colors['text'])
        ipk_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        # IPK coordinate inputs
        coord_frame = tk.Frame(ipk_frame, bg=self.colors['bg_control'])
        coord_frame.pack(padx=5, pady=5)
        
        tk.Label(coord_frame, text="Target Position:", 
                font=('Courier', 8, 'bold'),
                bg=self.colors['bg_control'], fg='#ffffff').pack(pady=2)
        
        # X input
        x_frame = tk.Frame(coord_frame, bg=self.colors['bg_control'])
        x_frame.pack(fill='x', pady=2)
        tk.Label(x_frame, text="X:", font=('Courier', 8), 
                bg=self.colors['bg_control'], fg='#aaaaaa').pack(side='left')
        self.ipk_x_entry = tk.Entry(x_frame, width=8, font=('Courier', 8))
        self.ipk_x_entry.insert(0, "0.4")
        self.ipk_x_entry.pack(side='right')
        
        # Y input
        y_frame = tk.Frame(coord_frame, bg=self.colors['bg_control'])
        y_frame.pack(fill='x', pady=2)
        tk.Label(y_frame, text="Y:", font=('Courier', 8),
                bg=self.colors['bg_control'], fg='#aaaaaa').pack(side='left')
        self.ipk_y_entry = tk.Entry(y_frame, width=8, font=('Courier', 8))
        self.ipk_y_entry.insert(0, "0.0")
        self.ipk_y_entry.pack(side='right')
        
        # Z input
        z_frame = tk.Frame(coord_frame, bg=self.colors['bg_control'])
        z_frame.pack(fill='x', pady=2)
        tk.Label(z_frame, text="Z:", font=('Courier', 8),
                bg=self.colors['bg_control'], fg='#aaaaaa').pack(side='left')
        self.ipk_z_entry = tk.Entry(z_frame, width=8, font=('Courier', 8))
        self.ipk_z_entry.insert(0, "0.3")
        self.ipk_z_entry.pack(side='right')
        
        # IPK control buttons
        ipk_btn_frame = tk.Frame(ipk_frame, bg=self.colors['bg_control'])
        ipk_btn_frame.pack(pady=5)
        
        # Solve and Go button
        solve_btn = tk.Button(ipk_btn_frame, text="SOLVE & GO",
                             command=self.ipk_solve_and_go,
                             font=('Courier', 8, 'bold'),
                             bg='#2a4a2a', fg=self.colors['text'],
                             relief='raised', bd=2)
        solve_btn.pack(side='left', padx=2)
        
        # Workspace visualization button (simple show only)
        workspace_btn = tk.Button(ipk_frame, text="SHOW TASK SPACE",
                                 command=self.show_detailed_workspace,
                                 font=('Courier', 8, 'bold'),
                                 bg='#2a2a4a', fg=self.colors['accent'],
                                 relief='raised', bd=2)
        workspace_btn.pack(pady=(5, 0))
        
        # Add ghost trail controls
        self.create_trail_controls(parent)
        
        # ENHANCED: Service response display with speed control status
        self.service_response_label = tk.Label(parent, text=" ",
                                             font=('Courier', 8),
                                             bg=self.colors['bg_control'], fg='#aaaaaa')
        self.service_response_label.pack(pady=5)
        
        # Teleoperation Controls (Enhanced with Frame Support)
        to_frame = tk.LabelFrame(parent, text="TELEOPERATION CONTROL", 
                               font=('Courier', 9, 'bold'),
                               bg=self.colors['bg_control'], fg=self.colors['text'])
        to_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        # Reference Frame Toggle Section
        frame_toggle_section = tk.Frame(to_frame, bg=self.colors['bg_control'])
        frame_toggle_section.pack(pady=5, padx=5)
        
        tk.Label(frame_toggle_section, text="Reference Frame:", 
                font=('Courier', 8, 'bold'),
                bg=self.colors['bg_control'], fg='#ffffff').pack(side='left', padx=5)
        
        # Frame toggle button
        self.frame_btn = tk.Button(frame_toggle_section, text="WORLD FRAME",
                                  command=self.toggle_reference_frame,
                                  font=('Courier', 8, 'bold'),
                                  bg='#2a2a4a', fg=self.colors['accent'],
                                  relief='raised', bd=2, width=15)
        self.frame_btn.pack(side='left', padx=5)
        
        # Frame status indicator
        self.frame_status = tk.Label(frame_toggle_section, text="(World coordinates)",
                                    font=('Courier', 7),
                                    bg=self.colors['bg_control'], fg='#aaaaaa')
        self.frame_status.pack(side='left', padx=5)
        
        # Discrete velocity controls with buttons
        vel_grid = tk.Frame(to_frame, bg=self.colors['bg_control'])
        vel_grid.pack(padx=5, pady=5)
        
        # Initialize velocity values
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        
        # Define button values - minus on LEFT, plus on RIGHT
        self.vel_values = [-0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03]
        
        # Row 1: X velocity
        tk.Label(vel_grid, text="Vx:", font=('Courier', 7),
                bg=self.colors['bg_control'], fg='#aaaaaa').grid(row=0, column=0, sticky='w')
        
        vx_btn_frame = tk.Frame(vel_grid, bg=self.colors['bg_control'])
        vx_btn_frame.grid(row=0, column=1, padx=2)
        
        for i, val in enumerate(self.vel_values):
            if val == 0.0:
                btn_text = "ZERO"
                color = '#4a4a4a'
            else:
                btn_text = f"{val:+.2f}"  # Show + or - clearly
                color = '#2a4a2a' if val > 0 else '#4a2a2a'
            btn = tk.Button(vx_btn_frame, text=btn_text, width=5,
                           command=lambda v=val: self.set_velocity('x', v),
                           font=('Courier', 6, 'bold'),
                           bg=color, fg='#ffffff', relief='raised', bd=1)
            btn.pack(side='left', padx=1)
        
        # Display current Vx value
        self.vx_display = tk.Label(vel_grid, text="0.0", font=('Courier', 7, 'bold'),
                                  bg=self.colors['bg_control'], fg=self.colors['text'])
        self.vx_display.grid(row=0, column=2, padx=5, sticky='w')
        
        # Row 2: Y velocity  
        tk.Label(vel_grid, text="Vy:", font=('Courier', 7),
                bg=self.colors['bg_control'], fg='#aaaaaa').grid(row=1, column=0, sticky='w')
        
        vy_btn_frame = tk.Frame(vel_grid, bg=self.colors['bg_control'])
        vy_btn_frame.grid(row=1, column=1, padx=2)
        
        for i, val in enumerate(self.vel_values):
            if val == 0.0:
                btn_text = "ZERO"
                color = '#4a4a4a'
            else:
                btn_text = f"{val:+.2f}"  # Show + or - clearly
                color = '#2a4a2a' if val > 0 else '#4a2a2a'
            btn = tk.Button(vy_btn_frame, text=btn_text, width=5,
                           command=lambda v=val: self.set_velocity('y', v),
                           font=('Courier', 6, 'bold'),
                           bg=color, fg='#ffffff', relief='raised', bd=1)
            btn.pack(side='left', padx=1)
        
        # Display current Vy value
        self.vy_display = tk.Label(vel_grid, text="0.0", font=('Courier', 7, 'bold'),
                                  bg=self.colors['bg_control'], fg=self.colors['text'])
        self.vy_display.grid(row=1, column=2, padx=5, sticky='w')
        
        # Row 3: Z velocity
        tk.Label(vel_grid, text="Vz:", font=('Courier', 7),
                bg=self.colors['bg_control'], fg='#aaaaaa').grid(row=2, column=0, sticky='w')
        
        vz_btn_frame = tk.Frame(vel_grid, bg=self.colors['bg_control'])
        vz_btn_frame.grid(row=2, column=1, padx=2)
        
        for i, val in enumerate(self.vel_values):
            if val == 0.0:
                btn_text = "ZERO"
                color = '#4a4a4a'
            else:
                btn_text = f"{val:+.2f}"  # Show + or - clearly
                color = '#2a4a2a' if val > 0 else '#4a2a2a'
            btn = tk.Button(vz_btn_frame, text=btn_text, width=5,
                           command=lambda v=val: self.set_velocity('z', v),
                           font=('Courier', 6, 'bold'),
                           bg=color, fg='#ffffff', relief='raised', bd=1)
            btn.pack(side='left', padx=1)
        
        # Display current Vz value
        self.vz_display = tk.Label(vel_grid, text="0.0", font=('Courier', 7, 'bold'),
                                  bg=self.colors['bg_control'], fg=self.colors['text'])
        self.vz_display.grid(row=2, column=2, padx=5, sticky='w')
        
    
    # ===========================================
    # TELEOPERATION FRAME METHODS  
    # ===========================================
    
    def toggle_reference_frame(self):
        """Toggle between World Frame and End Effector Frame"""
        if self.reference_frame == "WORLD":
            self.reference_frame = "END_EFFECTOR"
            self.frame_btn.config(text="EE FRAME", bg='#4a2a2a', fg=self.colors['text_alt'])
            self.frame_status.config(text="(End effector coordinates)")
            
            # Set TO_EF mode in controller if currently in TO mode
            if self.current_mode == "TO":
                self.call_service_mode("TO_EF")
                # Republish velocity so it takes effect immediately in new frame
                self.publish_velocity()
                self.get_logger().info(" Switched to End Effector Frame")
                
        else:
            self.reference_frame = "WORLD"
            self.frame_btn.config(text="WORLD FRAME", bg='#2a2a4a', fg=self.colors['accent'])
            self.frame_status.config(text="(World coordinates)")
            
            # Set TO_WF mode in controller if currently in TO mode
            if self.current_mode == "TO":
                self.call_service_mode("TO_WF")
                # Republish velocity so it takes effect immediately in new frame
                self.publish_velocity()
                self.get_logger().info(" Switched to World Frame")
    
    # ===========================================
    # MODE CONTROL METHODS
    # ===========================================
    
    def on_mode_change(self):
        """Handle mode change from radio buttons - automatically send service calls"""
        new_mode = self.mode_var.get()
        
        # DEBUG: Always log mode changes
        self.get_logger().info(f" GUI Mode change detected: {self.current_mode} → {new_mode}")
        
        if new_mode != self.current_mode:
            self.current_mode = new_mode
            self.mode_display.config(text=new_mode)
            
            # Publish mode change
            mode_msg = String()
            mode_msg.data = new_mode
            self.mode_publisher_.publish(mode_msg)
            
            # Update service response for AM mode with speed control info
            if new_mode == "AM":
                speed_value = self.speed_slider.get()
                self.service_response_label.config(
                    text=f"Auto Mode: {speed_value:.1f}x speed",
                    fg=self.colors['text']
                )
            
            # Automatically call appropriate service based on mode
            if new_mode == "MANUAL":
                self.get_logger().info(" Calling MANUAL mode service...")
                self.call_service_mode("MANUAL")
            elif new_mode == "IPK":
                self.get_logger().info(" Calling IPK mode service...")
                self.call_service_mode("IPK")
            elif new_mode == "TO":
                # Use current reference frame to determine TO_WF or TO_EF
                if self.reference_frame == "WORLD":
                    self.get_logger().info(" Calling TO_WF mode service...")
                    self.call_service_mode("TO_WF")
                else:
                    self.get_logger().info(" Calling TO_EF mode service...")
                    self.call_service_mode("TO_EF")
            elif new_mode == "AM":
                self.get_logger().info(" Calling AM mode service...")
                self.call_service_mode("AM")
        else:
            self.get_logger().info(f" Mode {new_mode} already active, no change needed")
        
        self.get_logger().info(f" Mode change to {new_mode} completed")
    
    def call_set_mode_service(self, mode):
        """Legacy method - now just calls call_service_mode"""
        self.call_service_mode(mode)
    
    def emergency_stop(self):
        """Emergency stop - return to manual mode"""
        self.mode_var.set("MANUAL")
        self.on_mode_change()
        self.get_logger().info("EMERGENCY STOP - Returned to manual mode")
    
    def build_status_bar(self):
        """Build status bar at the bottom"""
        status_bar = tk.Frame(self.root, bg='#1a1a1a', height=35)
        status_bar.pack(fill='x', side='bottom')
        status_bar.pack_propagate(False)
        
        # Status indicators
        self.status_label = tk.Label(status_bar, text="System Ready",
                                    font=('Courier', 9),
                                    bg='#1a1a1a', fg=self.colors['text'])
        self.status_label.pack(side='left', padx=10, pady=5)
        
        # Connection status
        conn_status = "CONNECTED" if SERVICES_AVAILABLE else "DISCONNECTED"
        conn_color = self.colors['text'] if SERVICES_AVAILABLE else self.colors['warn']
        self.connection_label = tk.Label(status_bar, text=f"Services: {conn_status}",
                                        font=('Courier', 9),
                                        bg='#1a1a1a', fg=conn_color)
        self.connection_label.pack(side='right', padx=10, pady=5)
    
    def show_detailed_workspace(self):
        """Generate high-detail rainbow workspace visualization"""
        try:
            self.service_response_label.config(text="Generating detailed rainbow workspace...", fg=self.colors['accent'])
            self.root.update()
            
            self.get_logger().info("Generating high-detail rainbow robot workspace")
            
            # Import robotics toolbox for FK calculations
            import roboticstoolbox as rtb
            from spatialmath import SE3
            
            # Create robot model (match YOUR exact DH parameters)
            L1 = rtb.RevoluteMDH(alpha=0, a=0, d=0.2, offset=0, qlim=[-np.pi/2, np.pi/2])
            L2 = rtb.RevoluteMDH(alpha=np.pi/2, a=0, d=0.12, offset=0, qlim=[-np.pi/2, np.pi/2])
            L3 = rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0, qlim=[-np.pi/2, np.pi/2])
            tool = SE3(0.28, 0, 0)
            robot = rtb.DHRobot([L1, L2, L3], name='workspace_robot', tool=tool)
            
            workspace_points = []
            
            # HIGH DETAIL - Much higher resolution for beautiful visualization
            resolution = 20  # 20³ = 8,000 points for detailed workspace
            self.get_logger().info(f"Generating high-detail workspace: {resolution}³ = {resolution**3:,} points")
            
            q1_range = np.linspace(-np.pi/2, np.pi/2, resolution)
            q2_range = np.linspace(-np.pi/2, np.pi/2, resolution)  
            q3_range = np.linspace(-np.pi/2, np.pi/2, resolution)
            
            point_count = 0
            for q1 in q1_range:
                for q2 in q2_range:
                    for q3 in q3_range:
                        try:
                            q = [q1, q2, q3]
                            fk = robot.fkine(q)
                            pos = fk.t
                            
                            # Store position with joint angles for rainbow coloring
                            workspace_points.append({
                                'pos': [float(pos[0]), float(pos[1]), float(pos[2])],
                                'joints': [q1, q2, q3],
                                'distance': float(np.sqrt(pos[0]**2 + pos[1]**2))
                            })
                            point_count += 1
                            
                        except Exception as e:
                            self.get_logger().debug(f"FK error at {q}: {e}")
                            continue
            
            self.get_logger().info(f"High-detail workspace complete: {len(workspace_points):,} points")
            
            if workspace_points:
                # Calculate workspace bounds for rainbow mapping
                zs = [p['pos'][2] for p in workspace_points]
                distances = [p['distance'] for p in workspace_points]
                
                z_min, z_max = min(zs), max(zs)
                dist_min, dist_max = min(distances), max(distances)
                
                range_info = f"Z:[{z_min:.3f},{z_max:.3f}] R:[{dist_min:.3f},{dist_max:.3f}]"
                self.get_logger().info(f"Rainbow workspace bounds: {range_info}")
            
            # Create beautiful rainbow point cloud
            point_cloud = self.create_rainbow_pointcloud2(workspace_points, z_min, z_max)
            self.workspace_pub.publish(point_cloud)
            
            # Update GUI status
            self.service_response_label.config(
                text=f" Rainbow workspace: {len(workspace_points):,} points {range_info}", 
                fg=self.colors['text']
            )
            
        except ImportError:
            self.get_logger().error("robotics-toolbox-python required for workspace generation")
            self.service_response_label.config(text="Error: robotics-toolbox-python required", fg=self.colors['warn'])
        except Exception as e:
            self.service_response_label.config(text=f"Workspace error: {str(e)[:50]}...", fg=self.colors['warn'])
            self.get_logger().error(f"Workspace generation error: {e}")
    
    def create_rainbow_pointcloud2(self, workspace_points, z_min, z_max):
        """Create stunning rainbow-colored point cloud based on height"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "link_0"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        def hsv_to_rgb(h, s, v):
            """Convert HSV to RGB (h: 0-360, s,v: 0-1)"""
            import colorsys
            r, g, b = colorsys.hsv_to_rgb(h/360.0, s, v)
            return int(r*255), int(g*255), int(b*255)
        
        point_data = []
        if workspace_points:
            z_range = z_max - z_min if z_max != z_min else 1.0
            
            for point_info in workspace_points:
                pos = point_info['pos']
                z = pos[2]
                distance = point_info['distance']
                
                # RAINBOW COLORING SCHEME
                # Method 1: Height-based rainbow (bottom to top)
                height_ratio = (z - z_min) / z_range
                hue = height_ratio * 270  # 0=red, 60=yellow, 120=green, 180=cyan, 240=blue, 270=purple
                
                # Method 2: Distance-based brightness variation
                max_distance = 0.6  # Approximate max reach
                distance_ratio = min(distance / max_distance, 1.0)
                brightness = 0.7 + 0.3 * distance_ratio  # Brighter at edges
                
                # Convert to RGB
                r, g, b = hsv_to_rgb(hue, 0.9, brightness)  # High saturation, variable brightness
                
                # Pack RGB as uint32
                rgb = (r << 16) | (g << 8) | b
                
                # Pack point data: x, y, z, rgb
                point_data.extend(struct.pack('ffff', 
                                            float(pos[0]), 
                                            float(pos[1]), 
                                            float(pos[2]), 
                                            float(rgb)))
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(workspace_points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = bytes(point_data)
        cloud_msg.is_dense = True
        
        return cloud_msg
    
    # ===========================================
    # TELEOPERATION METHODS
    # ===========================================
    
    def set_velocity(self, axis, value):
        """Add/subtract velocity for specified axis and publish immediately"""
        if axis == 'x':
            if value == 0.0:  # Zero button - reset to zero
                self.current_vx = 0.0
            else:  # Add/subtract to current velocity
                self.current_vx += value
            self.vx_display.config(text=f"{self.current_vx:+.3f}")
        elif axis == 'y':
            if value == 0.0:  # Zero button - reset to zero
                self.current_vy = 0.0
            else:  # Add/subtract to current velocity
                self.current_vy += value
            self.vy_display.config(text=f"{self.current_vy:+.3f}")
        elif axis == 'z':
            if value == 0.0:  # Zero button - reset to zero
                self.current_vz = 0.0
            else:  # Add/subtract to current velocity
                self.current_vz += value
            self.vz_display.config(text=f"{self.current_vz:+.3f}")
        
        # Clamp velocities to reasonable limits
        self.current_vx = max(-0.1, min(0.1, self.current_vx))
        self.current_vy = max(-0.1, min(0.1, self.current_vy)) 
        self.current_vz = max(-0.1, min(0.1, self.current_vz))
        
        # Update displays with clamped values
        self.vx_display.config(text=f"{self.current_vx:+.3f}")
        self.vy_display.config(text=f"{self.current_vy:+.3f}")
        self.vz_display.config(text=f"{self.current_vz:+.3f}")
        
        # Auto-start continuous velocity publishing if in TO mode
        if self.current_mode == "TO" and not self.to_active:
            if abs(self.current_vx) > 0.001 or abs(self.current_vy) > 0.001 or abs(self.current_vz) > 0.001:
                self.start_teleoperation()
        # Auto-stop if all velocities are zero
        elif self.to_active:
            if abs(self.current_vx) < 0.001 and abs(self.current_vy) < 0.001 and abs(self.current_vz) < 0.001:
                self.stop_teleoperation()
        
        # Publish immediately for responsive control
        self.publish_velocity()
    
    def reset_all_velocities(self):
        """Reset all velocities to zero"""
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        
        # Update displays
        self.vx_display.config(text="0.0")
        self.vy_display.config(text="0.0")
        self.vz_display.config(text="0.0")
        
        # Publish zero velocity
        self.publish_velocity()
        
        self.service_response_label.config(text=" All velocities reset to zero", fg=self.colors['text'])
        self.get_logger().info(" All velocities reset to zero")
    
    def publish_velocity(self):
        """Publish current velocity as Twist message"""
        try:
            twist = Twist()
            twist.linear.x = float(self.current_vx)
            twist.linear.y = float(self.current_vy)
            twist.linear.z = float(self.current_vz)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist)
            
            if abs(twist.linear.x) > 0.001 or abs(twist.linear.y) > 0.001 or abs(twist.linear.z) > 0.001:
                frame_name = "EE" if self.reference_frame == "END_EFFECTOR" else "World"
                self.service_response_label.config(text=f" Velocity: [{self.current_vx:+.3f}, {self.current_vy:+.3f}, {self.current_vz:+.3f}] ({frame_name} frame)", fg=self.colors['text'])
            else:
                self.service_response_label.config(text=" Teleoperation ready (click buttons)", fg=self.colors['text'])
        except Exception as e:
            self.get_logger().error(f"Immediate velocity error: {e}")

    def toggle_teleoperation(self):
        """Start/stop teleoperation based on current state"""
        if self.to_active:
            self.stop_teleoperation()
        else:
            self.start_teleoperation()
    
    def start_teleoperation(self):
        """Start sending velocity commands from GUI buttons"""
        if self.current_mode != "TO":
            self.service_response_label.config(text=" Set TO mode first!", fg=self.colors['warn'])
            return
        
        self.to_active = True
        
        # Enhanced status message with frame info
        frame_name = "World" if self.reference_frame == "WORLD" else "End Effector"
        self.service_response_label.config(
            text=f" Teleoperation active ({frame_name} frame)",
            fg=self.colors['text']
        )
        
        # Start timer for sending velocity commands
        self.teleop_timer = self.root.after(50, self.send_velocity_commands)
        
        self.get_logger().info(f" Teleoperation started in {frame_name} frame")
    
    def stop_teleoperation(self):
        """Stop teleoperation and zero all velocities"""
        self.to_active = False
        
        # Stop timer
        if hasattr(self, 'teleop_timer'):
            self.root.after_cancel(self.teleop_timer)
        
        # Zero all velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        
        # Update displays
        self.vx_display.config(text="0.0")
        self.vy_display.config(text="0.0")
        self.vz_display.config(text="0.0")
        
        # Send zero velocity
        self.publish_velocity()
        
        self.service_response_label.config(text=" Teleoperation stopped", fg=self.colors['warn'])
        self.get_logger().info(" Teleoperation stopped")
    
    def send_velocity_commands(self):
        """Continuous velocity publishing for teleoperation"""
        if self.to_active:
            self.publish_velocity()
            # Schedule next update
            self.teleop_timer = self.root.after(50, self.send_velocity_commands)
    
    # ===========================================
    # SERVICE METHODS
    # ===========================================
    
    def call_service_mode(self, mode):
        """Call service mode with enhanced TO mode handling"""
        if not SERVICES_AVAILABLE:
            self.service_response_label.config(text="Services not available!", fg=self.colors['warn'])
            return
            
        if not hasattr(self, 'set_mode_client') or not self.set_mode_client.service_is_ready():
            self.service_response_label.config(text="SetMode service not ready!", fg=self.colors['warn'])
            return
        
        # Update display mode for TO variants
        display_mode = mode
        if mode == "TO_WF":
            display_mode = "TO (World Frame)"
            self.reference_frame = "WORLD"
            self.frame_btn.config(text="WORLD FRAME", bg='#2a2a4a', fg=self.colors['accent'])
            self.frame_status.config(text="(World coordinates)")
        elif mode == "TO_EF":
            display_mode = "TO (End Effector Frame)"
            self.reference_frame = "END_EFFECTOR"
            self.frame_btn.config(text="EE FRAME", bg='#4a2a2a', fg=self.colors['text_alt'])
            self.frame_status.config(text="(End effector coordinates)")
        
        self.service_response_label.config(text=f"Setting mode to {display_mode}...", fg=self.colors['accent'])
        
        # Create and send request
        request = SetMode.Request()
        request.mode = mode
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_set_mode_response(f, mode))
    
    def handle_set_mode_response(self, future, mode):
        """Handle SetMode service response"""
        try:
            response = future.result()
            if response and response.success:
                # Enhanced status for AM mode with speed control
                if mode == "AM":
                    speed_value = self.speed_slider.get()
                    self.service_response_label.config(
                        text=f" Auto Mode: {speed_value:.1f}x speed",
                        fg=self.colors['text']
                    )
                else:
                    self.service_response_label.config(text=f" {response.message}", fg=self.colors['text'])
                
                self.current_mode = mode if mode not in ['TO_WF', 'TO_EF'] else 'TO'
                self.mode_display.config(text=self.current_mode)
                self.get_logger().info(f"Successfully set mode to {mode}: {response.message}")
            else:
                self.service_response_label.config(text=f" {response.message}", fg=self.colors['warn'])
                self.get_logger().error(f"Failed to set mode {mode}: {response.message}")
        except Exception as e:
            self.service_response_label.config(text=f" Service error: {e}", fg=self.colors['warn'])
            self.get_logger().error(f"SetMode service error: {e}")
    
    def ipk_solve_and_go(self):
        """IPK mode: solve IK and move to target position"""
        if not SERVICES_AVAILABLE:
            self.service_response_label.config(text="Services not available!", fg=self.colors['warn'])
            return
            
        if not hasattr(self, 'ik_client') or not self.ik_client.service_is_ready():
            self.service_response_label.config(text="IK service not ready!", fg=self.colors['warn'])
            return
        
        try:
            # Get target coordinates from entries
            x = float(self.ipk_x_entry.get())
            y = float(self.ipk_y_entry.get())
            z = float(self.ipk_z_entry.get())
            
            self.service_response_label.config(text=f"Solving IK for ({x}, {y}, {z})...", fg=self.colors['accent'])
            
            # Create service request with Pose (not PoseStamped!)
            from geometry_msgs.msg import Pose
            
            request = InverseKinematics.Request()
            request.target_pose = Pose()  # Use Pose directly
            request.target_pose.position.x = x
            request.target_pose.position.y = y
            request.target_pose.position.z = z
            request.target_pose.orientation.x = 0.0
            request.target_pose.orientation.y = 0.0
            request.target_pose.orientation.z = 0.0
            request.target_pose.orientation.w = 1.0
            
            # Call service
            future = self.ik_client.call_async(request)
            future.add_done_callback(lambda f: self.handle_ik_response(f, x, y, z))
            
        except ValueError:
            self.service_response_label.config(text=" Invalid coordinates", fg=self.colors['warn'])
        except Exception as e:
            self.service_response_label.config(text=f" IK error: {e}", fg=self.colors['warn'])
            self.get_logger().error(f"IK service error: {e}")
    
    def handle_ik_response(self, future, x, y, z):
        """Handle InverseKinematics service response"""
        try:
            response = future.result()
            if response and response.success:
                joint_degs = [f"{j*180/pi:.1f}°" for j in response.joint_positions]
                self.service_response_label.config(
                    text=f" IK Success: {joint_degs} → Moving to ({x:.3f}, {y:.3f}, {z:.3f})",
                    fg=self.colors['text']
                )
                self.get_logger().info(f"IK Success: {response.message}")
            else:
                self.service_response_label.config(text=f" IK Failed: {response.message}", fg=self.colors['warn'])
                self.get_logger().warning(f"IK Failed: {response.message}")
        except Exception as e:
            self.service_response_label.config(text=f" IK service error: {e}", fg=self.colors['warn'])
            self.get_logger().error(f"IK service error: {e}")
    
    # ===========================================
    # UPDATE LOOP
    # ===========================================
    
    def update_loop(self):
        """Main update loop"""
        # Publish joint states
        if self.current_mode == "MANUAL":
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "robot_description"
            
            for i, (name, default_val, display_name, joint_id) in enumerate(self.joints):
                msg.name.append(name)
                
                # Get target position from slider
                target_pos = self.sliders[name].get()
                
                if self.interp_var.get():
                    # Smooth interpolation - gradually move toward target
                    speed = self.speed_slider.get()
                    current = self.current_positions[i]
                    delta = target_pos - current
                    step = delta * 0.1 * speed
                    new_pos = current + step
                    msg.position.append(new_pos)
                    self.current_positions[i] = new_pos
                else:
                    # Direct control - move immediately to slider position
                    msg.position.append(target_pos)
                    self.current_positions[i] = target_pos
                
                # Update displays
                self.position_displays[name].config(text=f"{self.current_positions[i]:+.3f} rad")
                self.degree_displays[name].config(text=f"{self.current_positions[i]*180/pi:+.1f} deg")
            
            self.joint_publisher_.publish(msg)
        
        # ROS spin
        rclpy.spin_once(self, timeout_sec=0)
        
        # Schedule next update
        self.root.after(50, self.update_loop)
    
    def run(self):
        """Run the GUI"""
        self.root.mainloop()

    # ===========================================
    # GHOST TRAIL FUNCTIONS (PATH-BASED)
    # ===========================================
    
    def create_trail_controls(self, parent):
        """Create ghost trail controls"""
        trail_frame = tk.LabelFrame(parent, text=" GHOST TRAIL", 
                                  font=('Courier', 9, 'bold'),
                                  bg=self.colors['bg_control'], fg=self.colors['text_alt'])
        trail_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        # Trail control buttons
        trail_btn_frame = tk.Frame(trail_frame, bg=self.colors['bg_control'])
        trail_btn_frame.pack(pady=5, padx=5)
        
        # Start/Stop Trail button
        self.trail_btn = tk.Button(trail_btn_frame, text="START TRAIL",
                                  command=self.toggle_trail,
                                  font=('Courier', 8, 'bold'),
                                  bg='#2a4a2a', fg=self.colors['text'],
                                  relief='raised', bd=2, width=12)
        self.trail_btn.pack(side='left', padx=2)
        
        # Clear Trail button
        clear_btn = tk.Button(trail_btn_frame, text="CLEAR TRAIL",
                             command=self.clear_trail,
                             font=('Courier', 8, 'bold'),
                             bg='#4a2a2a', fg=self.colors['warn'],
                             relief='raised', bd=2, width=12)
        clear_btn.pack(side='left', padx=2)
        
        # Trail settings
        settings_frame = tk.Frame(trail_frame, bg=self.colors['bg_control'])
        settings_frame.pack(fill='x', pady=2, padx=5)
        
        # Max points slider
        tk.Label(settings_frame, text="Trail Length:", font=('Courier', 8),
                bg=self.colors['bg_control'], fg='#aaaaaa').pack(side='left')
        self.trail_length_var = tk.IntVar(value=1000)
        length_scale = tk.Scale(settings_frame, from_=100, to=5000, resolution=100,
                               orient='horizontal', variable=self.trail_length_var,
                               command=self.update_trail_length,
                               bg=self.colors['bg_control'], fg=self.colors['text'],
                               highlightthickness=0, length=150)
        length_scale.pack(side='right', fill='x', expand=True)
        
        # Trail status display
        self.trail_status_label = tk.Label(trail_frame, 
                                          text="Trail: INACTIVE (0 points)",
                                          font=('Courier', 8),
                                          bg=self.colors['bg_control'], 
                                          fg=self.colors['text'])
        self.trail_status_label.pack(pady=2)

    def toggle_trail(self):
        """Toggle ghost trail recording on/off"""
        self.trail_active = not self.trail_active
        
        if self.trail_active:
            self.trail_btn.config(text="STOP TRAIL", bg='#4a2a2a', fg=self.colors['warn'])
            self.get_logger().info(" Ghost trail STARTED - Publishing to /end_effector_path")
        else:
            self.trail_btn.config(text="START TRAIL", bg='#2a4a2a', fg=self.colors['text'])
            self.get_logger().info(" Ghost trail STOPPED")
        
        self.update_trail_status()

    def clear_trail(self):
        """Clear the ghost trail"""
        self.trail_points = []
        self.trail_counter = 0
        
        # Publish empty trail to remove visualization
        self.publish_trail_as_path()
        
        self.get_logger().info(" Ghost trail CLEARED")
        self.update_trail_status()

    def update_trail_length(self, value):
        """Update maximum trail length"""
        self.max_trail_points = int(value)
        
        # Trim existing trail if too long
        if len(self.trail_points) > self.max_trail_points:
            self.trail_points = self.trail_points[-self.max_trail_points:]
        
        self.update_trail_status()

    def update_trail_status(self):
        """Update trail status display"""
        if self.trail_active:
            status = f"Trail: ACTIVE ({len(self.trail_points)} points)"
            color = self.colors['text']
        else:
            status = f"Trail: INACTIVE ({len(self.trail_points)} points)"
            color = '#888888'
        
        self.trail_status_label.config(text=status, fg=color)

    def record_end_effector_position(self):
        """Record current end effector position for trail"""
        if not self.trail_active or self.end_effector_pose is None:
            return
        
        # Get current end effector position
        pos = self.end_effector_pose.pose.position
        point = [float(pos.x), float(pos.y), float(pos.z)]
        
        # Add to trail (avoid duplicates)
        if not self.trail_points or \
           any(abs(point[i] - self.trail_points[-1][i]) > 0.001 for i in range(3)):
            
            self.trail_points.append(point)
            
            # Limit trail length
            if len(self.trail_points) > self.max_trail_points:
                self.trail_points = self.trail_points[-self.max_trail_points:]
            
            self.trail_counter += 1
            
            # Publish trail immediately for every point
            if self.trail_counter % self.trail_publish_rate == 0:
                self.publish_trail_as_path()
                self.update_trail_status()

    def publish_trail_as_path(self):
        """Publish ghost trail as ROS Path message (much cleaner than MarkerArray!)"""
        path_msg = Path()
        path_msg.header.frame_id = "link_0"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Convert trail points to PoseStamped messages
        for point in self.trail_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "link_0"
            pose_stamped.header.stamp = path_msg.header.stamp
            
            # Set position
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = float(point[2])
            
            # Set orientation (identity quaternion)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        # Publish the path
        self.trail_pub.publish(path_msg)
        
        # Log progress occasionally
        if len(self.trail_points) % 50 == 0 and len(self.trail_points) > 0:
            self.get_logger().info(f" Path trail: {len(self.trail_points)} points published to /end_effector_path")

def main(args=None):
    rclpy.init(args=args)
    gui = AdvancedOperatorGUI()
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()