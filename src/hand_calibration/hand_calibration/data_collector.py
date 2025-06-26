#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger
from hand_calibration_interfaces.srv import StringArray
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import tf_transformations
from scipy.spatial.transform import Rotation as R

class AprilTagRotationAnalyzer(Node):
    """
    ROS2 Node to analyze rotation of multiple AprilTags simultaneously.
    Takes measurement when /collect_data service is called and computes the
    results when /compute service is called.
    """
    
    def __init__(self):
        super().__init__('apriltag_rotation_analyzer')
        
        # Parameters
        self.declare_parameter('base_frame', 'wrist_color_optical_frame')
        self.base_frame = self.get_parameter('base_frame').value
        
        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Storage for collected poses, organized by tag_frame
        self.collected_data = {}
        
        # Create service for collecting data with tag frame arguments
        self.collect_srv = self.create_service(
            StringArray, 
            '/collect_data', 
            self.collect_data_callback
        )
        
        # Create service for computing rotation with tag frame arguments
        self.compute_srv = self.create_service(
            StringArray,
            '/compute',
            self.compute_callback
        )
        
        # Create service for clearing data with tag frame arguments
        self.clear_srv = self.create_service(
            StringArray,
            '/clear_data',
            self.clear_data_callback
        )
        
        self.get_logger().info('\033[1;32m[INIT] AprilTag Rotation Analyzer node initialized\033[0m')
        self.get_logger().info(f'\033[1;34m[INFO] Base frame: {self.base_frame}\033[0m')
        self.get_logger().info('\033[1;34m[INFO] Call /collect_data with tag_frames to record tag poses\033[0m')
        self.get_logger().info('\033[1;34m[INFO] Call /compute with tag_frames to analyze the data\033[0m')

    def collect_data_callback(self, request, response):
        """Record the current pose of the specified AprilTags"""
        tag_frames = request.tags
        
        if not tag_frames:
            response.success = False
            response.message = "No tag_frames specified"
            return response
        
        success_tags = []
        fail_tags = []
        
        for tag_frame in tag_frames:
            try:
                # Get the transform from base_frame to tag_frame
                trans = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    tag_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5)  # 0.5 second timeout for transform lookup
                )
                
                # Extract position and orientation
                position = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]
                
                orientation = [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ]
                
                # Initialize data structure for this tag if it doesn't exist
                if tag_frame not in self.collected_data:
                    self.collected_data[tag_frame] = {
                        'poses': [],
                        'rotation_axis': None,
                        'rotation_angles': [],
                        'rotation_origin': None
                    }
                
                # Store the pose with timestamp
                self.collected_data[tag_frame]['poses'].append({
                    'position': position,
                    'orientation': orientation,
                    'timestamp': self.get_clock().now().to_msg()
                })
                
                count = len(self.collected_data[tag_frame]['poses'])
                self.get_logger().info(f"Recorded pose #{count} for tag {tag_frame}")
                success_tags.append(tag_frame)
                
            except Exception as e:
                self.get_logger().error(f"Failed to record pose for tag {tag_frame}: {str(e)}")
                fail_tags.append(tag_frame)
        
        # Prepare response message
        if success_tags:
            response.success = True
            response_msg = f"Successfully recorded poses for tags: {', '.join(success_tags)}"
            if fail_tags:
                response_msg += f". Failed for tags: {', '.join(fail_tags)}"
            response.message = response_msg
        else:
            response.success = False
            response.message = f"Failed to record poses for all tags: {', '.join(fail_tags)}"
            
        return response
    
    def compute_callback(self, request, response):
        """Compute rotation axis and angles for the specified tags"""
        tag_frames = request.tags
        
        # If no tags specified, use all tags with data
        if not tag_frames:
            tag_frames = list(self.collected_data.keys())
            
        if not tag_frames:
            response.success = False
            response.message = "No tag data available"
            return response
        
        success_tags = []
        fail_tags = []
        results = {}
        
        for tag_frame in tag_frames:
            if tag_frame not in self.collected_data:
                fail_tags.append(tag_frame)
                continue
                
            tag_data = self.collected_data[tag_frame]
            
            if len(tag_data['poses']) < 2:
                fail_tags.append(tag_frame)
                self.get_logger().error(f"Need at least 2 poses for tag {tag_frame} to compute rotation")
                continue
                
            try:
                # Calculate the rotation axis and angles
                self._compute_rotation(tag_frame)
                
                # Format the results for display
                axis = tag_data['rotation_axis']
                axis_str = f"[{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]"
                
                # Add the initial 0.0 angle for the first pose
                if len(tag_data['rotation_angles']) > 0 and tag_data['rotation_angles'][0] != 0.0:
                    tag_data['rotation_angles'].insert(0, 0.0)
                    
                # Convert angles to degrees for easier interpretation
                angles_deg = [angle * 180.0 / np.pi for angle in tag_data['rotation_angles']]
                angles_str = ", ".join([f"{angle:.2f}°" for angle in angles_deg])
                
                # Store results for this tag
                results[tag_frame] = {
                    'axis': axis_str,
                    'angles': angles_deg,
                    'angles_str': angles_str
                }
                
                # Print the results
                self.get_logger().info("==========================================")
                self.get_logger().info(f"Rotation Analysis Results for tag {tag_frame}:")
                self.get_logger().info(f"Number of poses used: {len(tag_data['poses'])}")
                self.get_logger().info(f"Rotation axis (unit vector): {axis_str}")
                self.get_logger().info(f"Rotation angles: {angles_str}")
                self.get_logger().info("==========================================")
                
                success_tags.append(tag_frame)
                
            except Exception as e:
                fail_tags.append(tag_frame)
                self.get_logger().error(f"Failed to compute rotation for tag {tag_frame}: {str(e)}")
        
        # Prepare response message
        if success_tags:
            response.success = True
            
            # Format detailed results for each tag
            result_messages = []
            for tag in success_tags:
                result_messages.append(f"Tag {tag}: axis={results[tag]['axis']}, angles={results[tag]['angles_str']}")
            
            response_msg = "Computed rotation for tags: " + ", ".join(success_tags)
            if fail_tags:
                response_msg += f". Failed for tags: {', '.join(fail_tags)}"
            
            # Add detailed results
            if result_messages:
                response_msg += "\n" + "\n".join(result_messages)
                
            response.message = response_msg
        else:
            response.success = False
            response.message = f"Failed to compute rotation for all tags: {', '.join(fail_tags)}"
            
        return response
    
    def clear_data_callback(self, request, response):
        """Clear data for specified tags"""
        tag_frames = request.tags
        
        # If no tags specified, clear all data
        if not tag_frames:
            self.collected_data = {}
            response.success = True
            response.message = "Cleared all tag data"
            return response
        
        for tag_frame in tag_frames:
            if tag_frame in self.collected_data:
                del self.collected_data[tag_frame]
                
        response.success = True
        response.message = f"Cleared data for tags: {', '.join(tag_frames)}"
        return response
    
    def _compute_rotation(self, tag_frame):
        """
        Compute the rotation axis and angles from collected poses for a specific tag
        
        The first pose is considered the reference pose.
        """
        tag_data = self.collected_data[tag_frame]
        
        if len(tag_data['poses']) < 2:
            raise ValueError(f"Need at least 2 poses for tag {tag_frame} to compute rotation")
            
        # Reference pose (first pose)
        ref_pose = tag_data['poses'][0]
        ref_pos = np.array(ref_pose['position'])
        ref_quat = np.array(ref_pose['orientation'])
        
        # Convert quaternion to rotation matrix
        ref_rot_mat = tf_transformations.quaternion_matrix(ref_quat)[:3, :3]
        
        # Initialize results
        tag_data['rotation_angles'] = []
        combined_axis = np.zeros(3)
        num_valid = 0
        
        # For each pose after the reference pose
        for pose in tag_data['poses'][1:]:
            pos = np.array(pose['position'])
            quat = np.array(pose['orientation'])
            
            # Compute relative transformation
            rot_mat = tf_transformations.quaternion_matrix(quat)[:3, :3]
            
            # Relative rotation: R_rel = R_current * R_ref^T
            rel_rot_mat = np.dot(rot_mat, ref_rot_mat.T)
            
            # Convert to axis-angle representation
            rel_rot = R.from_matrix(rel_rot_mat)
            axis_angle = rel_rot.as_rotvec()
            
            # Angle is the magnitude of the rotation vector
            angle = np.linalg.norm(axis_angle)
            
            # Axis is the normalized rotation vector
            if angle > 1e-6:  # Avoid division by zero for very small rotations
                axis = axis_angle / angle
                combined_axis += axis
                num_valid += 1
            else:
                # Skip poses with minimal rotation
                continue
                
            tag_data['rotation_angles'].append(float(angle))
        
        # Average rotation axis
        if num_valid > 0:
            tag_data['rotation_axis'] = combined_axis / num_valid
            # Normalize the axis
            tag_data['rotation_axis'] = tag_data['rotation_axis'] / np.linalg.norm(tag_data['rotation_axis'])
        else:
            raise ValueError(f"Could not compute valid rotation axis for tag {tag_frame}")
            
        # Attempt to find the origin of rotation (screw axis)
        if len(tag_data['poses']) >= 3:
            self._estimate_rotation_origin(tag_frame)
        else:
            self.get_logger().warn(f"At least 3 poses are recommended for accurate origin estimation for tag {tag_frame}")
    
    def _estimate_rotation_origin(self, tag_frame):
        """
        Estimate the origin of rotation (point on the screw axis)
        using the perpendicular vectors from each pose to the axis
        for a specific tag
        """
        tag_data = self.collected_data[tag_frame]
        
        # Use first pose as reference
        ref_pos = np.array(tag_data['poses'][0]['position'])
        
        # For each pose, find the closest point on the axis
        closest_points = []
        
        for pose in tag_data['poses']:
            pos = np.array(pose['position'])
            
            # Vector from reference to current position
            v = pos - ref_pos
            
            # Project v onto the axis
            projection = np.dot(v, tag_data['rotation_axis']) * tag_data['rotation_axis']
            
            # Perpendicular component
            perp = v - projection
            
            # Closest point on the axis to this position
            closest = ref_pos + projection
            closest_points.append(closest)
        
        # Average the closest points to estimate the origin
        if closest_points:
            tag_data['rotation_origin'] = np.mean(closest_points, axis=0)
        else:
            tag_data['rotation_origin'] = None

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagRotationAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()