import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
from threading import Thread
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CalibratedServoControlGUI(Node):
    def __init__(self):
        super().__init__('calibrated_servo_control_gui')
        
        # Declare parameters
        self.declare_parameter('calibration_file', 'servo_calibration.yaml')
        
        # Load calibration data
        self.calibration_file = self.get_parameter('calibration_file').value
        self.calibration = self.load_calibration(self.calibration_file)
        if not self.calibration:
            self.get_logger().error(f'Failed to load calibration file: {self.calibration_file}')
            rclpy.shutdown()
            return
        
        # Create a publisher for the /calibrated_servo_input topic instead of /hand_servo_input
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/calibrated_servo_input',
            10
        )
        
        # Create the GUI interface
        self.root = tk.Tk()
        self.root.title("Calibrated Hand Servo Control GUI")
        self.root.geometry("1000x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Store slider values for both raw and calibrated
        self.raw_sliders = []
        self.raw_values = []
        self.calibrated_sliders = []
        self.calibrated_values = []
        
        # Flag to prevent recursive updates between raw and calibrated sliders
        self.updating = False
        
        # Create a frame for the sliders
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Add column headers
        header_frame = ttk.Frame(main_frame)
        header_frame.pack(fill=tk.X, pady=5)
        
        # Joint label header
        joint_header = ttk.Label(header_frame, text="Joint", font=("Arial", 10, "bold"), width=20)
        joint_header.pack(side=tk.LEFT, padx=(0, 10), anchor='w')
        
        # Raw value header
        raw_header = ttk.Label(header_frame, text="Raw Value", font=("Arial", 10, "bold"))
        raw_header.pack(side=tk.LEFT, padx=10, anchor='center', expand=True)
        
        # Calibrated value header
        cal_header = ttk.Label(header_frame, text="Calibrated", font=("Arial", 10, "bold"))
        cal_header.pack(side=tk.RIGHT, padx=10, anchor='center', expand=True)
        
        # Define joint labels
        joint_labels = [
            "thumb CMC abduction",
            "index MCP abduction",
            "middle MCP abduction",
            "ring MCP abduction",
            "pinky MCP abduction",
            "thumb MCP abduction",
            "index MCP flexion",
            "middle MCP flexion",
            "ring MCP flexion",
            "pinky MCP flexion",
            "thumb MCP flexion",
            "index PIP flexion",
            "middle PIP flexion",
            "ring PIP flexion",
            "pinky PIP flexion",
            "thumb CMC flexion"
        ]
        
        # Define the raw slider range
        raw_min = -140.0
        raw_max = 140.0
        
        # Create sliders for all 16 servos with both raw and calibrated values
        for i in range(16):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=5)
            
            # Joint label
            label = ttk.Label(frame, text=joint_labels[i], width=20)
            label.pack(side=tk.LEFT, padx=(0, 10))
            
            # Create container frames for raw and calibrated controls
            raw_container = ttk.Frame(frame)
            raw_container.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            cal_container = ttk.Frame(frame)
            cal_container.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
            
            # Get calibration parameters
            channel_key = f'channel_{i}'
            gain = self.calibration[channel_key]['gain']
            offset = self.calibration[channel_key]['offset']
            
            # Calculate calibrated slider range
            cal_min = gain * raw_min + offset
            cal_max = gain * raw_max + offset
            
            # Ensure min is always less than max (in case of negative gain)
            if cal_min > cal_max:
                cal_min, cal_max = cal_max, cal_min
            
            # Raw value controls
            raw_value = tk.DoubleVar(value=0.0)
            self.raw_values.append(raw_value)
            
            raw_label = ttk.Label(raw_container, textvariable=raw_value, width=5)
            raw_label.pack(side=tk.RIGHT, padx=(10, 0))
            
            raw_slider = ttk.Scale(
                raw_container, 
                from_=raw_min, 
                to=raw_max,
                variable=raw_value,
                orient=tk.HORIZONTAL,
                command=lambda v, i=i: self.update_raw_value(i)
            )
            raw_slider.pack(fill=tk.X, expand=True)
            self.raw_sliders.append(raw_slider)
            
            # Calibrated value controls
            cal_value = tk.DoubleVar(value=offset)  # Initialize to offset (equivalent to raw=0)
            self.calibrated_values.append(cal_value)
            
            cal_label = ttk.Label(cal_container, textvariable=cal_value, width=5)
            cal_label.pack(side=tk.RIGHT, padx=(10, 0))
            
            cal_slider = ttk.Scale(
                cal_container, 
                from_=cal_min, 
                to=cal_max,
                variable=cal_value,
                orient=tk.HORIZONTAL,
                command=lambda v, i=i: self.update_calibrated_value(i)
            )
            cal_slider.pack(fill=tk.X, expand=True)
            self.calibrated_sliders.append(cal_slider)
            
            # Add calibration info to the label
            if gain != 1.0 or offset != 0.0:
                calib_text = f" (g={gain:.2f}, o={offset:.2f})"
                label.config(text=label.cget("text") + calib_text)
        
        # Control buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=10)
        
        send_button = ttk.Button(control_frame, text="Send Position", command=self.send_positions)
        send_button.pack(side=tk.LEFT, padx=5)
        
        # Split reset buttons
        reset_raw_button = ttk.Button(control_frame, text="Reset Raw to 0°", command=self.reset_raw_sliders)
        reset_raw_button.pack(side=tk.LEFT, padx=5)
        
        reset_cal_button = ttk.Button(control_frame, text="Reset Calibrated to 0°", command=self.reset_calibrated_sliders)
        reset_cal_button.pack(side=tk.LEFT, padx=5)
        
        # Checkbox for auto-sending
        self.auto_send = tk.BooleanVar(value=False)
        auto_check = ttk.Checkbutton(
            control_frame, 
            text="Auto-send on change", 
            variable=self.auto_send
        )
        auto_check.pack(side=tk.RIGHT, padx=5)
        
        # Create a status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Add calibration file info to status bar
        calib_info = ttk.Label(
            self.root, 
            text=f"Calibration: {self.calibration_file}", 
            relief=tk.SUNKEN, 
            anchor=tk.E
        )
        calib_info.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.get_logger().info('Calibrated Servo Control GUI initialized')
    
    def load_calibration(self, filename):
        """Load calibration data from YAML file."""
        try:
            # Try to find the file in the package share directory first
            try:
                package_share_dir = get_package_share_directory('hand_servo_control')
                config_path = os.path.join(package_share_dir, 'config', filename)
                if not os.path.exists(config_path):
                    # Fall back to absolute path
                    config_path = filename
            except Exception:
                # If package not found, use the filename directly
                config_path = filename
                
            with open(config_path, 'r') as file:
                calibration_data = yaml.safe_load(file)
                
            # Validate calibration data structure
            for i in range(16):
                channel_key = f'channel_{i}'
                if channel_key not in calibration_data:
                    self.get_logger().warn(f'Missing calibration for {channel_key}, using defaults (gain=1.0, offset=0.0)')
                    calibration_data[channel_key] = {'gain': 1.0, 'offset': 0.0}
                else:
                    # Ensure gain and offset exist
                    if 'gain' not in calibration_data[channel_key]:
                        self.get_logger().warn(f'Missing gain for {channel_key}, using default (1.0)')
                        calibration_data[channel_key]['gain'] = 1.0
                    if 'offset' not in calibration_data[channel_key]:
                        self.get_logger().warn(f'Missing offset for {channel_key}, using default (0.0)')
                        calibration_data[channel_key]['offset'] = 0.0
                        
            return calibration_data
            
        except Exception as e:
            self.get_logger().error(f'Error loading calibration file: {e}')
            return None
    
    def update_raw_value(self, index):
        """Called when a raw slider value changes"""
        if self.updating:
            return
            
        self.updating = True
        
        # Round the value to one decimal place for display
        raw_value = round(self.raw_values[index].get(), 1)
        self.raw_values[index].set(raw_value)
        
        # Calculate and update the calibrated value
        channel_key = f'channel_{index}'
        gain = self.calibration[channel_key]['gain']
        offset = self.calibration[channel_key]['offset']
        
        # Apply calibration: gain * position + offset
        calibrated_value = round(gain * raw_value + offset, 1)
        self.calibrated_values[index].set(calibrated_value)
        
        # If auto-send is enabled, send positions
        if self.auto_send.get():
            self.send_positions()
            
        self.updating = False
    
    def update_calibrated_value(self, index):
        """Called when a calibrated slider value changes"""
        if self.updating:
            return
            
        self.updating = True
        
        # Round the value to one decimal place for display
        calibrated_value = round(self.calibrated_values[index].get(), 1)
        self.calibrated_values[index].set(calibrated_value)
        
        # Calculate and update the raw value
        channel_key = f'channel_{index}'
        gain = self.calibration[channel_key]['gain']
        offset = self.calibration[channel_key]['offset']
        
        # Invert calibration: (calibrated_value - offset) / gain
        if gain != 0:  # Prevent division by zero
            raw_value = round((calibrated_value - offset) / gain, 1)
        else:
            raw_value = 0
            self.get_logger().warn(f'Gain for channel {index} is zero, cannot calculate raw value')
            
        self.raw_values[index].set(raw_value)
        
        # If auto-send is enabled, send positions
        if self.auto_send.get():
            self.send_positions()
            
        self.updating = False
    
    def send_positions(self):
        """Send the current calibrated slider positions to the /calibrated_servo_input topic"""
        msg = Float32MultiArray()
        
        # Now we send the calibrated values instead of raw
        msg.data = [float(val.get()) for val in self.calibrated_values]
        
        self.publisher.publish(msg)
        self.status_var.set(f"Sent calibrated positions: {[round(val, 1) for val in msg.data]}")
        self.get_logger().info(f'Published calibrated servo positions to /calibrated_servo_input')
    
    def reset_raw_sliders(self):
        """Reset all raw sliders to 0 degrees"""
        self.updating = True
        
        for i in range(len(self.raw_values)):
            self.raw_values[i].set(0.0)
            
            # Calculate the calibrated value for zero raw input
            channel_key = f'channel_{i}'
            gain = self.calibration[channel_key]['gain']
            offset = self.calibration[channel_key]['offset']
            
            # At raw=0, calibrated = offset
            calibrated_value = round(offset, 1)
            self.calibrated_values[i].set(calibrated_value)
        
        self.updating = False
        
        # Send the reset position
        if self.auto_send.get():
            self.send_positions()
        
        self.status_var.set("Reset raw values to 0°")
    
    def reset_calibrated_sliders(self):
        """Reset all calibrated sliders to 0 degrees"""
        self.updating = True
        
        for i in range(len(self.calibrated_values)):
            self.calibrated_values[i].set(0.0)
            
            # Calculate the raw value for zero calibrated input
            channel_key = f'channel_{i}'
            gain = self.calibration[channel_key]['gain']
            offset = self.calibration[channel_key]['offset']
            
            # Calculate raw value: (calibrated - offset) / gain = (0 - offset) / gain
            if gain != 0:  # Prevent division by zero
                raw_value = round(-offset / gain, 1)
            else:
                raw_value = 0
                self.get_logger().warn(f'Gain for channel {i} is zero, cannot calculate raw value')
                
            self.raw_values[i].set(raw_value)
        
        self.updating = False
        
        # Send the reset position
        if self.auto_send.get():
            self.send_positions()
            
        self.status_var.set("Reset calibrated values to 0°")
    
    def on_closing(self):
        """Handle window close event"""
        self.get_logger().info('Shutting down GUI')
        self.root.destroy()
        rclpy.shutdown()
    
    def spin(self):
        """Run the GUI mainloop"""
        while rclpy.ok():
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    
    node = CalibratedServoControlGUI()
    
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    except tk.TclError:
        # Handle Tkinter errors when window is closed
        pass
    finally:
        # Ensure the node is properly destroyed
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()