#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, Empty
import time
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import os
from datetime import datetime
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import the custom message type for tag arrays
from hand_calibration_interfaces.srv import StringArray

class MotorCalibrator(Node):
    """
    ROS2 Node for motor calibration using AprilTag measurements.
    
    This node provides services for different calibration sequences
    and stores calibration results for each motor.
    
    Now supports tracking multiple AprilTags simultaneously.
    """
    
    def __init__(self):
        super().__init__('motor_calibrator')
        
        # Create callback groups for concurrent service calls
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.client_group = ReentrantCallbackGroup()
        
        # Common parameters
        self.declare_parameter('output_dir', 'calibration_results')
        self.declare_parameter('service_timeout', 5.0)
        self.declare_parameter('service_retry_count', 3)
        
        # Calibration-specific parameters
        # MCP Abduction parameters
        self.declare_parameter('mcp_abduction.motor_index', 4)
        self.declare_parameter('mcp_abduction.tag_frame', 'tag_10')
        self.declare_parameter('mcp_abduction.min_angle', -30.0)
        self.declare_parameter('mcp_abduction.max_angle', 30.0)
        self.declare_parameter('mcp_abduction.num_steps', 10)
        self.declare_parameter('mcp_abduction.settle_time', 5.0)
        
        # MCP Flexion parameters
        self.declare_parameter('mcp_flexion.motor_index', 9)
        self.declare_parameter('mcp_flexion.tag_frame', 'tag_10')
        self.declare_parameter('mcp_flexion.min_angle', 10.0)
        self.declare_parameter('mcp_flexion.max_angle', 70.0)
        self.declare_parameter('mcp_flexion.num_steps', 10)
        self.declare_parameter('mcp_flexion.settle_time', 5.0)
        self.declare_parameter('mcp_flexion.hysteresis_clearance', -20.0)
        
        # PIP Flexion parameters
        self.declare_parameter('pip_flexion.motor_index', 14)
        self.declare_parameter('pip_flexion.tag_frame', 'tag_0')
        self.declare_parameter('pip_flexion.min_angle', 0.0)
        self.declare_parameter('pip_flexion.max_angle', 60.0)
        self.declare_parameter('pip_flexion.num_steps', 10)
        self.declare_parameter('pip_flexion.settle_time', 5.0)
        self.declare_parameter('pip_flexion.hysteresis_clearance', -10.0)
        
        # MCP Flexion Multi parameters
        self.declare_parameter('mcp_flexion_multi.motor_index', 9)
        self.declare_parameter('mcp_flexion_multi.tag_frame', 'tag_10')
        self.declare_parameter('mcp_flexion_multi.additional_tags', ['tag_0', 'tag_1'])
        self.declare_parameter('mcp_flexion_multi.min_angle', 10.0)
        self.declare_parameter('mcp_flexion_multi.max_angle', 70.0)
        self.declare_parameter('mcp_flexion_multi.num_steps', 10)
        self.declare_parameter('mcp_flexion_multi.settle_time', 5.0)
        self.declare_parameter('mcp_flexion_multi.hysteresis_clearance', -20.0)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.service_timeout = self.get_parameter('service_timeout').value
        self.service_retry_count = self.get_parameter('service_retry_count').value
   
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create publisher for motor commands
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/hand_servo_input',
            10
        )
        
        # Create clients for AprilTag services with callback group
        self.collect_client = self.create_client(
            StringArray, 
            '/collect_data', 
            callback_group=self.client_group
        )
        
        self.compute_client = self.create_client(
            StringArray, 
            '/compute',
            callback_group=self.client_group
        )
        
        self.clear_client = self.create_client(
            StringArray,
            '/clear_data',
            callback_group=self.client_group
        )
        
        # Wait for services to be available with timeout
        service_wait_timeout = 5.0  # 5 seconds timeout for initial service check
        start_time = time.time()
        
        while not self.collect_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > service_wait_timeout:
                self.get_logger().error('Timed out waiting for /collect_data service')
                break
            self.get_logger().info('Waiting for /collect_data service...')
        
        start_time = time.time()
        while not self.compute_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > service_wait_timeout:
                self.get_logger().error('Timed out waiting for /compute service')
                break
            self.get_logger().info('Waiting for /compute service...')
        
        # Create services for different calibration sequences
        self.phalanx_mcp_abduction_srv = self.create_service(
            Empty,
            'calibrate_phalanx_mcp_abduction',
            self.calibrate_phalanx_mcp_abduction_callback,
            callback_group=self.service_group
        )
        
        self.phalanx_mcp_flexion_srv = self.create_service(
            Empty,
            'calibrate_phalanx_mcp_flexion',
            self.calibrate_phalanx_mcp_flexion_callback,
            callback_group=self.service_group
        )
        
        self.phalanx_pip_flexion_srv = self.create_service(
            Empty,
            'calibrate_phalanx_pip_flexion',
            self.calibrate_phalanx_pip_flexion_callback,
            callback_group=self.service_group
        )
        
        # Add a new service for multi-tag calibration
        self.phalanx_mcp_flexion_multi_srv = self.create_service(
            Empty,
            'calibrate_phalanx_mcp_flexion_multi',
            self.calibrate_phalanx_mcp_flexion_with_multi_tags_callback,
            callback_group=self.service_group
        )
        
        # Storage for calibration results
        self.calibration_results = {}

        self.get_logger().info('\033[1;32m[INIT] Motor calibrator initialized\033[0m')
        self.get_logger().info('\033[1;34m[INFO] Available calibration services:\033[0m')
        self.get_logger().info('\033[1;34m[INFO]  - calibrate_phalanx_mcp_abduction\033[0m')
        self.get_logger().info('\033[1;34m[INFO]  - calibrate_phalanx_mcp_flexion\033[0m')
        self.get_logger().info('\033[1;34m[INFO]  - calibrate_phalanx_pip_flexion\033[0m')
        self.get_logger().info('\033[1;34m[INFO]  - calibrate_phalanx_mcp_flexion_multi\033[0m')
    
    def calibrate_phalanx_mcp_abduction_callback(self, request, response):
        """Service callback for calibrating MCP abduction"""
        try:
            self.get_logger().info('\033[1;34m[INFO] Starting phalanx MCP abduction calibration...\033[0m')
            
            # Get parameters from ROS parameters
            motor_index = self.get_parameter('mcp_abduction.motor_index').value
            tag_frame = self.get_parameter('mcp_abduction.tag_frame').value
            min_angle = self.get_parameter('mcp_abduction.min_angle').value
            max_angle = self.get_parameter('mcp_abduction.max_angle').value
            num_steps = self.get_parameter('mcp_abduction.num_steps').value
            settle_time = self.get_parameter('mcp_abduction.settle_time').value
            
            additional_motors = {9: 30.0} 

            self.get_logger().info(f'\033[1;34m[INFO] Using motor_index={motor_index}, tag_frame={tag_frame}\033[0m')

            # Run the calibration sequence
            result = self.run_calibration_sequence(
                motor_index=motor_index,
                min_angle=min_angle,
                max_angle=max_angle,
                num_steps=num_steps,
                settle_time=settle_time,
                sequence_name='phalanx_mcp_abduction',
                tag_frame=tag_frame,
                additional_motors=additional_motors
            )
            
            if result:
                self.get_logger().info('\033[1;32m[SUCCESS] Phalanx MCP abduction calibration completed successfully\033[0m')
            else:
                self.get_logger().error('\033[1;31m[ERROR] Phalanx MCP abduction calibration failed\033[0m')
                
            return response
        
        except Exception as e:
            self.get_logger().error(f'\033[1;31m[ERROR] Error during calibration: {str(e)}\033[0m')
            return response
    
    def calibrate_phalanx_mcp_flexion_callback(self, request, response):
        """Service callback for calibrating MCP flexion"""
        try:
            self.get_logger().info('\033[1;34m[INFO] Starting phalanx MCP flexion calibration...\033[0m')
            
            # Get parameters from ROS parameters
            motor_index = self.get_parameter('mcp_flexion.motor_index').value
            tag_frame = self.get_parameter('mcp_flexion.tag_frame').value
            min_angle = self.get_parameter('mcp_flexion.min_angle').value
            max_angle = self.get_parameter('mcp_flexion.max_angle').value
            num_steps = self.get_parameter('mcp_flexion.num_steps').value
            settle_time = self.get_parameter('mcp_flexion.settle_time').value
            hysteresis_clearance = self.get_parameter('mcp_flexion.hysteresis_clearance').value
            
            # Define additional motor positions to maintain during calibration
            additional_motors = {14: 30.0, 4: 0.0}  # Keep motor 14 at 30 degrees

            self.get_logger().info(f'\033[1;34m[INFO] Using motor_index={motor_index}, tag_frame={tag_frame}\033[0m')
            
            # Run the calibration sequence with hysteresis reduction
            result = self.run_calibration_sequence(
                motor_index=motor_index,
                min_angle=min_angle,
                max_angle=max_angle,
                num_steps=num_steps,
                settle_time=settle_time,
                sequence_name='phalanx_mcp_flexion',
                tag_frame=tag_frame,
                additional_motors=additional_motors,
                hysteresis_clearance=hysteresis_clearance
            )
            
            if result:
                self.get_logger().info('\033[1;32m[SUCCESS] Phalanx MCP flexion calibration completed successfully\033[0m')
            else:
                self.get_logger().error('\033[1;31m[ERROR] Phalanx MCP flexion calibration failed\033[0m')
                
            return response
        
        except Exception as e:
            self.get_logger().error(f'\033[1;31m[ERROR] Error during calibration: {str(e)}\033[0m')
            return response
    
    def calibrate_phalanx_pip_flexion_callback(self, request, response):
        """Service callback for calibrating PIP flexion"""
        try:
            self.get_logger().info('\033[1;34m[INFO] Starting phalanx PIP flexion calibration...\033[0m')
            
            # Get parameters from ROS parameters
            motor_index = self.get_parameter('pip_flexion.motor_index').value
            tag_frame = self.get_parameter('pip_flexion.tag_frame').value
            min_angle = self.get_parameter('pip_flexion.min_angle').value
            max_angle = self.get_parameter('pip_flexion.max_angle').value
            num_steps = self.get_parameter('pip_flexion.num_steps').value
            settle_time = self.get_parameter('pip_flexion.settle_time').value
            hysteresis_clearance = self.get_parameter('pip_flexion.hysteresis_clearance').value
            
            additional_motors = {4: 0.0, 9: 0.0}  # Keep motor 9 at 30 degrees

            self.get_logger().info(f'\033[1;34m[INFO] Using motor_index={motor_index}, tag_frame={tag_frame}\033[0m')
            
            # Run the calibration sequence
            result = self.run_calibration_sequence(
                motor_index=motor_index,
                min_angle=min_angle,
                max_angle=max_angle,
                num_steps=num_steps,
                settle_time=settle_time,
                sequence_name='phalanx_pip_flexion',
                tag_frame=tag_frame,
                hysteresis_clearance=hysteresis_clearance,
                additional_motors=additional_motors
            )
            
            if result:
                self.get_logger().info('\033[1;32m[SUCCESS] Phalanx PIP flexion calibration completed successfully\033[0m')
            else:
                self.get_logger().error('\033[1;31m[ERROR] Phalanx PIP flexion calibration failed\033[0m')
                
            return response
        
        except Exception as e:
            self.get_logger().error(f'\033[1;31m[ERROR] Error during calibration: {str(e)}\033[0m')
            return response
    
    def calibrate_phalanx_mcp_flexion_with_multi_tags_callback(self, request, response):
        """Service callback for calibrating MCP flexion with multiple tag tracking"""
        try:
            self.get_logger().info('\033[1;34m[INFO] Starting phalanx MCP flexion calibration with multi-tag tracking...\033[0m')
            
            # Get parameters from ROS parameters
            motor_index = self.get_parameter('mcp_flexion_multi.motor_index').value
            primary_tag_frame = self.get_parameter('mcp_flexion_multi.tag_frame').value
            additional_tags = self.get_parameter('mcp_flexion_multi.additional_tags').value
            min_angle = self.get_parameter('mcp_flexion_multi.min_angle').value
            max_angle = self.get_parameter('mcp_flexion_multi.max_angle').value
            num_steps = self.get_parameter('mcp_flexion_multi.num_steps').value
            settle_time = self.get_parameter('mcp_flexion_multi.settle_time').value
            hysteresis_clearance = self.get_parameter('mcp_flexion_multi.hysteresis_clearance').value
            
            # Define additional motor positions to maintain during calibration
            additional_motors = {14: 30.0, 4: 0.0}  # Keep motor 14 at 30 degrees

            self.get_logger().info(f'\033[1;34m[INFO] Using motor_index={motor_index}, tag_frame={primary_tag_frame}\033[0m')
            self.get_logger().info(f'\033[1;34m[INFO] Additional tags: {additional_tags}\033[0m')
            
            # Run the calibration sequence with hysteresis reduction and multi-tag tracking
            result = self.run_calibration_sequence(
                motor_index=motor_index,
                min_angle=min_angle,
                max_angle=max_angle,
                num_steps=num_steps,
                settle_time=settle_time,
                sequence_name='phalanx_mcp_flexion_multi',
                tag_frame=primary_tag_frame,
                additional_motors=additional_motors,
                hysteresis_clearance=hysteresis_clearance,
                additional_tags=additional_tags
            )
            
            if result:
                self.get_logger().info('\033[1;32m[SUCCESS] Phalanx MCP flexion calibration with multi-tag tracking completed successfully\033[0m')
            else:
                self.get_logger().error('\033[1;31m[ERROR] Phalanx MCP flexion calibration with multi-tag tracking failed\033[0m')
                
            return response
        
        except Exception as e:
            self.get_logger().error(f'\033[1;31m[ERROR] Error during calibration: {str(e)}\033[0m')
            return response
    
    def run_calibration_sequence(self, motor_index, min_angle, max_angle, num_steps, 
                                settle_time, sequence_name, tag_frame, 
                                additional_motors=None, hysteresis_clearance=None,
                                additional_tags=None):
        """
        Run a calibration sequence with the specified parameters
        
        Parameters:
        -----------
        motor_index : int
            Index of the motor to calibrate
        min_angle : float
            Minimum angle for the sweep (degrees)
        max_angle : float
            Maximum angle for the sweep (degrees)
        num_steps : int
            Number of points in the sweep
        settle_time : float
            Time to wait after moving to a position (seconds)
        sequence_name : str
            Name of the calibration sequence (for logging and file naming)
        tag_frame : str
            Primary AprilTag frame to track for calibration
        additional_motors : dict, optional
            Dictionary of motor indices and positions to maintain during calibration
        hysteresis_clearance : float, optional
            Angle to move to before each target position to reduce hysteresis effects
        additional_tags : list, optional
            Additional tag frames to track simultaneously
            
        Returns:
        --------
        bool
            True if calibration was successful, False otherwise
        """
        # Create list of all tags to track
        all_tags = [tag_frame]
        if additional_tags:
            all_tags.extend(additional_tags)
        
        # Clear any existing data for these tags
        self._call_service_with_timeout(self.clear_client, 'clear_data', all_tags)
        
        # Storage for calibration data
        commanded_angles = []
        measured_angles = []
        
        # Generate angle sweep
        angles = np.linspace(min_angle, max_angle, num_steps)
        
        # Move to each angle, collect data
        for i, angle in enumerate(angles):
            self.get_logger().info(f'Step {i+1}/{num_steps}: Moving to {angle:.2f}°')
            
            # If hysteresis clearance is enabled, move to clearance position first
            if hysteresis_clearance is not None:
                self.get_logger().info(f'Moving to hysteresis clearance position: {hysteresis_clearance:.2f}°')
                self._set_motor_angle(motor_index, hysteresis_clearance, additional_motors)
                time.sleep(1.0) # Allow time to settle
            
            # Send command to motor
            self._set_motor_angle(motor_index, angle, additional_motors)
            
            # Allow motor to reach position
            self.get_logger().info(f'Waiting {settle_time}s for motor to settle...')
            time.sleep(settle_time)
            
            # Collect data for all tags with timeout handling and retries
            self.get_logger().info(f'Capturing position for tags: {", ".join(all_tags)}...')
            
            success = False
            for retry in range(self.service_retry_count):
                response = self._call_service_with_timeout(self.collect_client, 'collect_data', all_tags)
                
                if response is not None and response.success:
                    commanded_angles.append(angle)
                    self.get_logger().info(f'Recorded command: {angle:.2f}° for tags: {", ".join(all_tags)}')
                    success = True
                    break
                else:
                    error_msg = response.message if response else "Service call timed out or failed"
                    if retry < self.service_retry_count - 1:
                        self.get_logger().warn(f'Retry {retry+1}/{self.service_retry_count}: Failed to collect data - {error_msg}')
                        time.sleep(0.5)  # Short delay before retry
                    else:
                        self.get_logger().error(f'Failed to collect data after {self.service_retry_count} attempts - {error_msg}')
            
            if not success:
                self.get_logger().warn(f'Skipping angle {angle:.2f}° due to collection failure')
        
        # Check if we have enough data points
        if len(commanded_angles) < 2:
            self.get_logger().error('Not enough data points collected for calibration')
            return False
        
        # Compute results for the primary tag
        self.get_logger().info(f'Collecting rotation analysis for primary tag {tag_frame}...')
        
        # Retry compute service call if needed
        compute_response = None
        for retry in range(self.service_retry_count):
            compute_response = self._call_service_with_timeout(self.compute_client, 'compute', tag_frame)
            
            if compute_response is not None and compute_response.success:
                self.get_logger().info('Successfully computed rotation')
                break
            else:
                error_msg = compute_response.message if compute_response else "Service call timed out or failed"
                if retry < self.service_retry_count - 1:
                    self.get_logger().warn(f'Retry {retry+1}/{self.service_retry_count}: Failed to compute - {error_msg}')
                    time.sleep(0.5)  # Short delay before retry
                else:
                    self.get_logger().error(f'Failed to compute after {self.service_retry_count} attempts - {error_msg}')
        
        if compute_response is None or not compute_response.success:
            self.get_logger().error('Failed to compute rotation results')
            return False
        
        # If additional tags were tracked, compute their results too (for logging purposes)
        if additional_tags:
            self.get_logger().info(f'Computing rotation for additional tags: {", ".join(additional_tags)}')
            additional_response = self._call_service_with_timeout(self.compute_client, 'compute', additional_tags)
            if additional_response and additional_response.success:
                self.get_logger().info(f'Additional tag results: {additional_response.message}')
        
        # Parse the angles from the response message for the primary tag
        measured_angles = self._extract_angles(compute_response.message)
        
        if measured_angles and len(measured_angles) > 0:
            # Compute calibration parameters
            result = self._compute_calibration(
                commanded_angles, 
                measured_angles, 
                motor_index, 
                sequence_name
            )
            return result
        else:
            self.get_logger().error('Failed to extract angle measurements')
            return False
    
    def _call_service_with_timeout(self, client, service_name, tag_frames=None):
        """
        Call a service with timeout handling
        
        Parameters:
        -----------
        client : rclpy.client.Client
            The service client to call
        service_name : str
            Name of the service (for logging)
        tag_frames : list or str, optional
            Tag frame(s) to use (for collect_data, compute, and clear_data services)
            
        Returns:
        --------
        response or None
            Service response, or None if the call failed
        """
        if not client.service_is_ready():
            self.get_logger().error(f'/{service_name} service is not available')
            return None
        
        try:
            # Create the appropriate request type based on service
            if service_name in ['collect_data', 'compute', 'clear_data']:
                # For these services, we use our custom StringArray service
                request = client.srv_type.Request()
                
                # Convert single tag_frame to list if needed
                if tag_frames:
                    if isinstance(tag_frames, str):
                        request.tags = [tag_frames]
                    else:
                        request.tags = tag_frames
                else:
                    request.tags = []
                    
                # Note: Remove any reference to 'header' as it's not in your service definition
            else:
                request = Trigger.Request()
            
            future = client.call_async(request)
            
            # Wait for response with timeout
            timeout_reached = False
            start_time = time.time()
            
            # Keep spinning until we get a response or timeout
            while not future.done() and not timeout_reached:
                # Spin for a short time
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Check for timeout
                if time.time() - start_time > self.service_timeout:
                    self.get_logger().error(f'Service call to /{service_name} timed out after {self.service_timeout}s')
                    timeout_reached = True
            
            # If we didn't timeout, return the result
            if not timeout_reached:
                return future.result()
            return None
        
        except Exception as e:
            self.get_logger().error(f'Error calling /{service_name} service: {str(e)}')
            return None
    
    def _set_motor_angle(self, motor_index, angle, additional_motors=None):
        """
        Set the specified motor to a specific angle in degrees
        
        Parameters:
        -----------
        motor_index : int
            Index of the motor to control
        angle : float
            Target angle in degrees
        additional_motors : dict, optional
            Dictionary of other motor indices and positions to set
        """
        try:
            # Create message with 16 zeros
            msg = Float32MultiArray()
            msg.data = [0.0] * 16
            
            # Set our specific motor index
            msg.data[motor_index] = angle
            
            # Set additional motors if specified
            if additional_motors:
                for idx, pos in additional_motors.items():
                    msg.data[idx] = pos
            
            # Publish command
            self.servo_pub.publish(msg)
            self.get_logger().debug(f'Published motor command: motor {motor_index} = {angle}°')
            
        except Exception as e:
            self.get_logger().error(f'Failed to set motor angle: {str(e)}')
    
    def _extract_angles(self, message):
        """
        Extract measured angles from the compute service response
        
        Returns:
        --------
        list
            List of measured angles, or None if parsing failed
        """
        try:
            # The new format may have multiple tag results
            # Find the specific tag's information
            if "angles=" not in message:
                self.get_logger().error(f'Unable to find angles in message: {message}')
                return None
                
            # Extract the angles section from the tag-specific part
            # Format: Tag tag_10: axis=[...], angles=0.00°, 2.69°, 7.05°, ...
            message_lines = message.split('\n')
            
            # Look for the line containing our tag and angles
            angles_part = None
            for line in message_lines:
                if "angles=" in line:
                    angles_part = line.split("angles=")[1].strip()
                    break
                    
            if not angles_part:
                self.get_logger().error('Unable to extract angles from message')
                return None
                
            # Extract the individual angle values
            angle_strings = angles_part.replace('°', '').split(', ')
            
            # Convert to float
            measured_angles = [float(angle) for angle in angle_strings]
            
            self.get_logger().info(f'Extracted {len(measured_angles)} angle measurements')
            return measured_angles
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse angles: {str(e)}')
            return None
    
    def _create_output_folder(self, sequence_name, motor_index, timestamp):
        """
        Create a properly structured output folder for the calibration results
        
        Parameters:
        -----------
        sequence_name : str
            Name of the calibration sequence
        motor_index : int
            Index of the motor being calibrated
        timestamp : str
            Timestamp string in format YYYYMMDD_HHMMSS
            
        Returns:
        --------
        str
            The full path to the created output folder
        """
        # Create folder name based on calibration type and date
        folder_name = f"{sequence_name}_motor_{motor_index}_{timestamp}"
        
        # Create the full path
        output_folder = os.path.join(self.output_dir, folder_name)
        
        # Ensure the folder exists
        os.makedirs(output_folder, exist_ok=True)
        
        self.get_logger().info(f'Created output folder: {output_folder}')
        return output_folder

    def _compute_calibration(self, commanded_angles, measured_angles, motor_index, sequence_name):
        """
        Compute and store calibration parameters
        """
        # Check if we have the expected number of measurements
        if len(measured_angles) != len(commanded_angles) - 1:
            self.get_logger().warn(f'Expected {len(commanded_angles)-1} measurements, got {len(measured_angles)}')
            
            # If we have more measurements than commanded angles, trim the measurements
            if len(measured_angles) > len(commanded_angles) - 1:
                self.get_logger().warn(f'Trimming measured angles to match commanded angles')
                measured_angles = measured_angles[:len(commanded_angles)-1]
            # If we have more commanded angles than measurements, trim the commanded angles
            elif len(commanded_angles) > len(measured_angles) + 1:
                self.get_logger().warn(f'Trimming commanded angles to match measurements')
                commanded_angles = commanded_angles[:len(measured_angles) + 1]
        
        # Check if we have enough data points for calibration
        if len(commanded_angles) < 2 or len(measured_angles) < 1:
            self.get_logger().error('Not enough data points for calibration')
            return False
        
        # Calculate commanded angles relative to the first commanded angle
        commanded_relative = [commanded_angles[i+1] - commanded_angles[0] 
                            for i in range(len(commanded_angles)-1)]
        
        self.get_logger().info(f'Commanded relative to first: {commanded_relative}')
        self.get_logger().info(f'Measured angles (relative to first): {measured_angles}')
        
        # Linear function for curve fitting
        def linear_model(x, gain, offset):
            return gain * x + offset
        
        try:
            # Ensure arrays are the same length
            if len(commanded_relative) != len(measured_angles):
                self.get_logger().warn('Array length mismatch after processing, adjusting to the shorter length')
                min_length = min(len(commanded_relative), len(measured_angles))
                commanded_relative = commanded_relative[:min_length]
                measured_angles = measured_angles[:min_length]
            
            self.get_logger().info(f'Final array lengths for fitting: commanded={len(commanded_relative)}, measured={len(measured_angles)}')
            
            # Perform curve fitting
            params, covariance = curve_fit(linear_model, commanded_relative, measured_angles)
            gain, offset = params
            
            # Calculate coefficient of determination (R^2)
            residuals = measured_angles - linear_model(np.array(commanded_relative), gain, offset)
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((measured_angles - np.mean(measured_angles))**2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
            
            # Calculate RMSE
            rmse = np.sqrt(np.mean(residuals**2))
            
            # Calculate inverse parameters (to convert from measured to commanded)
            inverse_gain = 1.0 / gain
            inverse_offset = -offset / gain
            
            # Get timestamp for this calibration
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            # Store the results
            calibration_data = {
                'gain': gain,
                'offset': offset,
                'inverse_gain': inverse_gain,
                'inverse_offset': inverse_offset,
                'r_squared': r_squared,
                'rmse': rmse,
                'commanded_angles': commanded_angles,
                'measured_angles': measured_angles,
                'commanded_relative': commanded_relative,
                'timestamp': timestamp
            }
            
            # Store in the results dictionary
            key = f'motor_{motor_index}_{sequence_name}'
            self.calibration_results[key] = calibration_data
            
            # Print results
            self.get_logger().info("==========================================")
            self.get_logger().info(f"Calibration Results for Motor {motor_index} ({sequence_name}):")
            self.get_logger().info(f"Gain (measured/commanded): {gain:.6f}")
            self.get_logger().info(f"Offset: {offset:.6f}")
            self.get_logger().info(f"R²: {r_squared:.6f}")
            self.get_logger().info(f"RMSE: {rmse:.6f} degrees")
            self.get_logger().info("==========================================")
            
            self.get_logger().info("Inverse Calibration (for control):")
            self.get_logger().info(f"Control Gain (commanded/measured): {inverse_gain:.6f}")
            self.get_logger().info(f"Control Offset: {inverse_offset:.6f}")
            self.get_logger().info("==========================================")
            
            # Create output folder with proper structure
            output_folder = self._create_output_folder(sequence_name, motor_index, timestamp)
            
            # Plot the results
            self._plot_calibration(
                gain, offset, inverse_gain, inverse_offset,
                commanded_angles, commanded_relative, measured_angles,
                motor_index, sequence_name, output_folder
            )
            
            # Save calibration parameters to file
            self._save_calibration_data(calibration_data, motor_index, sequence_name, output_folder)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to compute calibration: {str(e)}')
            return False

    def _plot_calibration(self, gain, offset, inverse_gain, inverse_offset,
                        commanded_angles, commanded_relative, measured_angles,
                        motor_index, sequence_name, output_folder):
        """Create calibration plots"""
        try:
            # Create figure for the calibration curve
            plt.figure(figsize=(10, 6))
            
            # Plot data points
            plt.scatter(commanded_relative, measured_angles, 
                        color='blue', label='Calibration Points')
            
            # Plot fitted line
            x_range = np.linspace(min(commanded_relative), max(commanded_relative), 100)
            y_fit = gain * x_range + offset
            plt.plot(x_range, y_fit, 'r-', label=f'Fitted: y = {gain:.4f}x + {offset:.4f}')
            
            # Plot identity line (y=x) for reference
            plt.plot(x_range, x_range, 'g--', label='Identity (y=x)')
            
            # Labels and title
            plt.xlabel('Commanded Angle Relative to First (degrees)')
            plt.ylabel('Measured Angle Relative to First (degrees)')
            plt.title(f'Motor {motor_index} Calibration Results ({sequence_name})')
            plt.grid(True)
            plt.legend()
            
            # Save the figure
            filename = os.path.join(output_folder, "calibration_curve.png")
            plt.savefig(filename)
            self.get_logger().info(f'Saved calibration plot to {filename}')
            
            # Create absolute angle plot
            plt.figure(figsize=(10, 6))
            
            # Add first point (0,0) to both datasets for complete visualization
            cmd_with_first = [0] + commanded_relative
            meas_with_first = [0] + measured_angles
            
            # Plot commanded vs measured absolute angles
            plt.plot(cmd_with_first, cmd_with_first, 'g--', label='Ideal (y=x)')
            plt.plot(cmd_with_first, meas_with_first, 'bo-', label='Measured')
            
            # Create calibrated line
            calibrated_x = cmd_with_first
            calibrated_y = [(y - offset) / gain for y in meas_with_first]
            plt.plot(calibrated_x, calibrated_y, 'r--', label='Calibrated')
            
            # Labels and title
            plt.xlabel('Commanded Angle (degrees)')
            plt.ylabel('Angle (degrees)')
            plt.title(f'Motor {motor_index} Angle Comparison ({sequence_name})')
            plt.grid(True)
            plt.legend()
            
            # Save the figure
            filename = os.path.join(output_folder, "angle_comparison.png")
            plt.savefig(filename)
            self.get_logger().info(f'Saved angle comparison to {filename}')
            
            # Close all figures to free resources
            plt.close('all')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create plot: {str(e)}')

    def _save_calibration_data(self, calibration_data, motor_index, sequence_name, output_folder):
        """Save calibration data to files"""
        try:
            # Save in CSV format
            csv_file = os.path.join(output_folder, "calibration_data.csv")
            with open(csv_file, 'w') as f:
                # Write header
                f.write("# Motor Calibration Results\n")
                f.write(f"# Motor: {motor_index}\n")
                f.write(f"# Sequence: {sequence_name}\n")
                f.write(f"# Date: {calibration_data['timestamp']}\n")
                f.write(f"# Gain: {calibration_data['gain']}\n")
                f.write(f"# Offset: {calibration_data['offset']}\n")
                f.write(f"# Inverse Gain: {calibration_data['inverse_gain']}\n")
                f.write(f"# Inverse Offset: {calibration_data['inverse_offset']}\n")
                f.write(f"# R²: {calibration_data['r_squared']}\n")
                f.write(f"# RMSE: {calibration_data['rmse']}\n")
                f.write("# Data:\n")
                f.write("commanded_angle,measured_angle,commanded_relative\n")
                
                # Write data rows
                for i in range(len(calibration_data['commanded_relative'])):
                    f.write(f"{calibration_data['commanded_angles'][i+1]},{calibration_data['measured_angles'][i]},{calibration_data['commanded_relative'][i]}\n")
            
            self.get_logger().info(f'Saved calibration data to {csv_file}')
            
            # Also save in a format easy to import into ROS parameter files
            yaml_file = os.path.join(output_folder, "calibration_params.yaml")
            with open(yaml_file, 'w') as f:
                f.write(f"motor_calibration_{motor_index}_{sequence_name}:\n")
                f.write(f"  gain: {calibration_data['gain']}\n")
                f.write(f"  offset: {calibration_data['offset']}\n")
                f.write(f"  inverse_gain: {calibration_data['inverse_gain']}\n")
                f.write(f"  inverse_offset: {calibration_data['inverse_offset']}\n")
                f.write(f"  r_squared: {calibration_data['r_squared']}\n")
                f.write(f"  rmse: {calibration_data['rmse']}\n")
            
            self.get_logger().info(f'Saved calibration parameters to {yaml_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    calibrator = MotorCalibrator()
    
    # Use a multithreaded executor to allow concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(calibrator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()