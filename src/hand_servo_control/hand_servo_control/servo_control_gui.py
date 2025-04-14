import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
from threading import Thread
import numpy as np

class ServoControlGUI(Node):
    def __init__(self):
        super().__init__('servo_control_gui')
        
        # Create a publisher for the /hand_servo_input topic
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/hand_servo_input',
            10
        )
        
        # Create the GUI interface
        self.root = tk.Tk()
        self.root.title("Hand Servo Control GUI")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Store slider values
        self.sliders = []
        self.slider_values = []
        
        # Create a frame for the sliders
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
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
        
        # Create sliders for all 16 servos
        for i in range(16):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=5)
            
            label = ttk.Label(frame, text=joint_labels[i])
            label.pack(side=tk.LEFT, padx=(0, 10))
            
            value_var = tk.DoubleVar(value=0.0)
            self.slider_values.append(value_var)
            
            value_label = ttk.Label(frame, textvariable=value_var, width=5)
            value_label.pack(side=tk.RIGHT, padx=(10, 0))
            
            slider = ttk.Scale(
                frame, 
                from_=-90.0, 
                to=90.0,
                variable=value_var,
                orient=tk.HORIZONTAL,
                command=lambda v, i=i: self.update_value(i)
            )
            slider.pack(fill=tk.X, expand=True)
            self.sliders.append(slider)
        
        # Control buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=10)
        
        send_button = ttk.Button(control_frame, text="Send Position", command=self.send_positions)
        send_button.pack(side=tk.LEFT, padx=5)
        
        reset_button = ttk.Button(control_frame, text="Reset to 0°", command=self.reset_sliders)
        reset_button.pack(side=tk.LEFT, padx=5)
        
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
        
        self.get_logger().info('Servo Control GUI initialized')
    
    def update_value(self, index):
        """Called when a slider value changes"""
        # Round the value to one decimal place for display
        value = round(self.slider_values[index].get(), 1)
        self.slider_values[index].set(value)
        
        # If auto-send is enabled, send all positions
        if self.auto_send.get():
            self.send_positions()
    
    def send_positions(self):
        """Send the current slider positions to the servo topic"""
        msg = Float32MultiArray()
        msg.data = [float(slider_val.get()) for slider_val in self.slider_values]
        
        self.publisher.publish(msg)
        self.status_var.set(f"Sent positions: {[round(val, 1) for val in msg.data]}")
        self.get_logger().info(f'Published servo positions')
    
    def reset_sliders(self):
        """Reset all sliders to 90 degrees"""
        for i, val in enumerate(self.slider_values):
            val.set(0.0)
        
        # Send the reset position
        if self.auto_send.get():
            self.send_positions()
    
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
    
    node = ServoControlGUI()
    
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