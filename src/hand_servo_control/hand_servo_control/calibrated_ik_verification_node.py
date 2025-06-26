import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray
from hand_kinematics.srv import InverseKinematics
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Point
import asyncio

# ANSI color codes
RESET = "\033[0m"
RED = "\033[91m"
GREEN = "\033[92m"
BLUE = "\033[94m"

class CalibratedIKVerificationNode(Node):
    def __init__(self) -> None:
        super().__init__('calibrated_ik_verification_node')

        self.target_frame: str = "hand_base_link"
        self.source_tag_frame: str = "piano" # This is the child_frame_id from atag_calibration.yaml for tag 29
        self.pinky_chain_name: str = "pinky"

        # Create reentrant callback group to allow concurrent execution
        self.reentrant_group = ReentrantCallbackGroup()

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # IK service client with reentrant callback group
        from rclpy.qos import qos_profile_services_default
        self.ik_client = self.create_client(
            InverseKinematics, 
            'hand/inverse_kinematics',
            qos_profile=qos_profile_services_default,
            callback_group=self.reentrant_group
        )
        
        # Wait for service to become available
        service_timeout = 10.0
        if not self.ik_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error(f'{RED}IK service not available after {service_timeout}s{RESET}')
            raise RuntimeError("IK service not available")
        
        self.get_logger().info(f'{GREEN}IK service is available{RESET}')

        # Publisher for calibrated servo input
        # QoS Profile to match typical sensor/command data: reliable, volatile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.servo_publisher = self.create_publisher(
            Float32MultiArray,
            '/calibrated_servo_input',
            qos_profile
        )

        # Create the 'reach' service with reentrant callback group
        self.reach_service = self.create_service(
            Trigger,
            'ik_reach_tag',
            self.handle_reach_service,
            callback_group=self.reentrant_group
        )

        self.get_logger().info(f'{GREEN}Calibrated IK Verification Node initialized.{RESET}')
        self.get_logger().info(f'{BLUE}Targeting tag "{self.source_tag_frame}" in "{self.target_frame}" frame for "{self.pinky_chain_name}" finger.{RESET}')

    async def call_ik_service_async(self, ik_request: InverseKinematics.Request) -> InverseKinematics.Response:
        """Async wrapper for IK service call"""
        self.get_logger().info(f'{BLUE}Making async IK service call...{RESET}')
        
        if not self.ik_client.service_is_ready():
            raise RuntimeError("IK service is not ready")
        
        # Create future for the service call
        future = self.ik_client.call_async(ik_request)
        
        # Convert ROS2 future to asyncio future
        loop = asyncio.get_event_loop()
        
        # Create an asyncio Event to wait for completion
        done_event = asyncio.Event()
        result_holder = {'result': None, 'exception': None}
        
        def on_done(ros_future):
            try:
                result_holder['result'] = ros_future.result()
            except Exception as e:
                result_holder['exception'] = e
            loop.call_soon_threadsafe(done_event.set)
        
        future.add_done_callback(on_done)
        
        # Wait for completion with timeout
        try:
            await asyncio.wait_for(done_event.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            self.get_logger().error(f'{RED}Async IK service call timed out{RESET}')
            if not future.done():
                future.cancel()
            raise RuntimeError("IK service call timed out")
        
        if result_holder['exception']:
            raise result_holder['exception']
        
        return result_holder['result']

    async def handle_reach_service_async(self, request: Trigger.Request) -> Trigger.Response:
        """Async implementation of the reach service handler"""
        self.get_logger().info(f'{BLUE}Received "reach" service call (async handler).{RESET}')
        
        response = Trigger.Response()

        try:
            # 1. Lookup transform from source_tag_frame to target_frame (hand_base_link)
            now = rclpy.time.Time()
            self.get_logger().info(f'{BLUE}Looking up transform from "{self.target_frame}" to "{self.source_tag_frame}"{RESET}')
            
            # We want the pose of the 'piano' tag *in* the 'hand_base_link' frame.
            # So, target_frame is 'hand_base_link', source_frame is 'piano'.
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_tag_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info(f'{BLUE}Transform "{self.source_tag_frame}" to "{self.target_frame}" found.{RESET}')
            
            target_pos_in_hand_base = Point()
            target_pos_in_hand_base.x = trans.transform.translation.x
            target_pos_in_hand_base.y = trans.transform.translation.y
            target_pos_in_hand_base.z = trans.transform.translation.z

            self.get_logger().info(f'{BLUE}Target position for IK (in {self.target_frame}): '
                                   f'x={target_pos_in_hand_base.x:.4f}, y={target_pos_in_hand_base.y:.4f}, z={target_pos_in_hand_base.z:.4f}{RESET}')

        except TransformException as ex:
            self.get_logger().error(f'{RED}Could not transform {self.source_tag_frame} to {self.target_frame}: {ex}{RESET}')
            response.success = False
            response.message = f'TransformException: {ex}'
            return response

        # 2. Prepare and call IK service request
        ik_request = InverseKinematics.Request()
        ik_request.chain_name = self.pinky_chain_name
        ik_request.target_position = target_pos_in_hand_base
        ik_request.initial_guess_deg = [0.0, 0.0, 0.0] # For 3-DOF pinky

        try:
            # Use async service call
            ik_response = await self.call_ik_service_async(ik_request)
            
            if ik_response is None:
                self.get_logger().error(f'{RED}IK service call returned None.{RESET}')
                response.success = False
                response.message = 'IK service call returned None.'
                return response

        except Exception as e:
            self.get_logger().error(f'{RED}Exception during async IK service call: {e}{RESET}')
            response.success = False
            response.message = f'Exception during IK service call: {e}'
            return response

        if ik_response.success:
            self.get_logger().info(f'{BLUE}IK service succeeded. Joint angles (deg): {ik_response.joint_angles_deg}{RESET}')
            
            # 3. Construct Float32MultiArray message for /calibrated_servo_input
            # Initialize all 16 joint angles to 0.0 (or a neutral pose)
            servo_positions = [0.0] * 16 
            
            pinky_angles_deg = ik_response.joint_angles_deg
            if len(pinky_angles_deg) == 3:
                # Mapping based on FrankaPianoPlayer:
                # Pinky PMA: channel 4
                # Pinky PMF: channel 9
                # Pinky PPF/PDF: channel 14
                servo_positions[4] = pinky_angles_deg[0]  # Pinky joint 1 (pma)
                servo_positions[9] = pinky_angles_deg[1]  # Pinky joint 2 (pmf)
                servo_positions[14] = pinky_angles_deg[2] # Pinky joint 3 (ppf/distal)
                
                # 4. Publish the Float32MultiArray message
                servo_msg = Float32MultiArray()
                servo_msg.data = [float(angle) for angle in servo_positions] # Ensure they are float
                
                self.servo_publisher.publish(servo_msg)
                self.get_logger().info(f'{BLUE}Published to /calibrated_servo_input: {servo_msg.data}{RESET}')
                
                response.success = True
                response.message = 'Successfully computed IK and sent servo commands for pinky.'
            else:
                self.get_logger().error(f'{RED}IK service returned {len(pinky_angles_deg)} angles for pinky, expected 3.{RESET}')
                response.success = False
                response.message = 'IK service returned incorrect number of angles for pinky.'
        else:
            self.get_logger().error(f'{RED}IK service failed: {ik_response.message}{RESET}')
            response.success = False
            response.message = f'IK service failed: {ik_response.message}'
            
        return response

    def handle_reach_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Synchronous wrapper that calls the async handler"""
        # Create an event loop for this thread if one doesn't exist
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        # Run the async handler
        try:
            return loop.run_until_complete(self.handle_reach_service_async(request))
        except Exception as e:
            self.get_logger().error(f'{RED}Error in async handler: {e}{RESET}')
            response.success = False
            response.message = f'Error in async handler: {e}'
            return response

def main(args: list = None) -> None:
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor to support reentrant callback groups
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    
    node = CalibratedIKVerificationNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()