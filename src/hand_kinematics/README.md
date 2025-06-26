# Hand Kinematics C++ Node

A ROS2 C++ implementation of forward and inverse kinematics for BiDexHand with mechanical coupling between joints.

## Features

- Forward kinematics computation for all fingers (thumb, index, middle, ring, pinky)
- Inverse kinematics using Ceres optimization library
- Unified mechanical coupling between joints (PIP to DIP) using antiparallelogram model
- URDF-based robot description parsing
- ROS2 service interface for easy integration

## Dependencies

- ROS2 (tested with Jazzy)
- Eigen3
- Ceres Solver
- TinyXML2
- yaml-cpp

### Ubuntu Dependencies

```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-geometry-msgs ros-jazzy-std-msgs
sudo apt install libeigen3-dev libceres-dev libtinyxml2-dev libyaml-cpp-dev
```

## Building

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or copy this package
# Copy your package files here

# Build
cd ~/ros2_ws
colcon build --packages-select hand_kinematics

# Source the workspace
source install/setup.bash
```

## Usage

### Running the Node

```bash
ros2 launch hand_kinematics hand_kinematics.launch.py urdf_file:=/path/to/your/hand.urdf
```

### ROS2 Services

#### Services Provided
- `/hand/forward_kinematics` (hand_kinematics/srv/ForwardKinematics) - Compute tip position from joint angles
- `/hand/inverse_kinematics` (hand_kinematics/srv/InverseKinematics) - Compute joint angles to reach target position

### Testing

Run the test client to see the node in action:

```bash
ros2 run hand_kinematics hand_kinematics_test_client
```

The test client will:
1. Call forward kinematics service with sample joint angles
2. Call inverse kinematics service with target positions
3. Display the results including success status and computed values

### Example Usage in Code

#### Using Forward Kinematics Service
```cpp
auto client = node->create_client<hand_kinematics::srv::ForwardKinematics>("hand/forward_kinematics");
auto request = std::make_shared<hand_kinematics::srv::ForwardKinematics::Request>();
request->chain_name = "index";
request->joint_angles_deg = {10.0, 20.0, 30.0};  // degrees

auto future = client->async_send_request(request);
```

#### Using Inverse Kinematics Service
```cpp
auto client = node->create_client<hand_kinematics::srv::InverseKinematics>("hand/inverse_kinematics");
auto request = std::make_shared<hand_kinematics::srv::InverseKinematics::Request>();
request->chain_name = "thumb";
request->target_position.x = 0.08;
request->target_position.y = 0.02;
request->target_position.z = 0.05;
request->initial_guess_deg = {0.0, 10.0, 5.0, 15.0};  // optional

auto future = client->async_send_request(request);
```

#### Using Command Line
```bash
# Forward Kinematics
ros2 service call /hand/forward_kinematics hand_kinematics/srv/ForwardKinematics "chain_name: 'index'
joint_angles_deg: [10.0, 20.0, 30.0]"

# Inverse Kinematics
ros2 service call /hand/inverse_kinematics hand_kinematics/srv/InverseKinematics "chain_name: 'thumb'
target_position:
  x: 0.08
  y: 0.02
  z: 0.05
initial_guess_deg: [0.0, 10.0, 5.0, 15.0]"
```

## Architecture

The main class `HandKinematics` handles:

1. **URDF Parsing**: Reads robot description from URDF file
2. **Kinematic Tree Building**: Constructs the kinematic tree
3. **Forward Kinematics**: Computes end-effector poses from joint angles
4. **Inverse Kinematics**: Solves for joint angles given target poses
5. **Mechanical Coupling**: Applies unified coupling between PIP and DIP joints

### Mechanical Coupling

The system implements unified coupling between:
- PIP (Proximal Interphalangeal) joints and DIP (Distal Interphalangeal) joints for all fingers including the thumb
- Uses the antiparallelgram model as described in linkage_analysis/antiparallelgram_model.pdf

## Configuration

### Finger Chains

The node supports the following finger chains:
- `index`: Index finger (ima_joint, imf_joint, ipf_joint) with tip joint i_tip_joint
- `middle`: Middle finger (mma_joint, mmf_joint, mpf_joint) with tip joint m_tip_joint
- `ring`: Ring finger (rma_joint, rmf_joint, rpf_joint) with tip joint r_tip_joint
- `pinky`: Pinky finger (pma_joint, pmf_joint, ppf_joint) with tip joint p_tip_joint
- `thumb`: Thumb (tcf_joint, tca_joint, tma_joint, tmf_joint) with tip joint t_tip_joint

### URDF Requirements

Your URDF should define:
- All joint names as specified in the finger chains
- Joint limits for proper IK constraints
- Proper parent-child relationships

## Troubleshooting

1. **IK doesn't converge**: Check that your target position is reachable and within joint limits
2. **Node crashes on startup**: Verify that the URDF path is correct and the file is valid
3. **Unexpected results**: Ensure that the joint names in the URDF match the expected names in the code

## License

MIT License

## Contributing

Feel free to submit issues and enhancement requests!
ros2 service call /hand/forward_kinematics hand_kinematics_cpp/srv/ForwardKinematics "chain_name: 'index'
joint_angles_deg: [10.0, 20.0, 30.0]"

# Inverse Kinematics
ros2 service call /hand/inverse_kinematics hand_kinematics_cpp/srv/InverseKinematics "chain_name: 'thumb'
target_position:
  x: 0.08
  y: 0.02
  z: 0.05
initial_guess_deg: [0.0, 10.0, 5.0, 15.0]"


## Architecture

The main class `HandKinematics` handles:

1. **URDF Parsing**: Reads robot description from URDF file
2. **Kinematic Tree Building**: Constructs the kinematic tree
3. **Forward Kinematics**: Computes end-effector poses from joint angles
4. **Inverse Kinematics**: Solves for joint angles given target poses
5. **Mechanical Coupling**: Applies coupling between PIP and DIP joints

### Mechanical Coupling

The system implements unified coupling between:
- PIP (Proximal Interphalangeal) joints and DIP (Distal Interphalangeal) joints for all fingers including the thumb
- Uses the antiparallelogram model as described in linkage_analysis/antiparallelgram_model.pdf

## Configuration

### Finger Chains

The node supports the following finger chains:
- `index`: Index finger (ima_joint, imf_joint, ipf_joint)
- `middle`: Middle finger (mma_joint, mmf_joint, mpf_joint)
- `ring`: Ring finger (rma_joint, rmf_joint, rpf_joint)
- `pinky`: Pinky finger (pma_joint, pmf_joint, ppf_joint)
- `thumb`: Thumb (tcf_joint, tca_joint, tma_joint, tmf_joint)

### URDF Requirements

Your URDF should define:
- All joint names as specified in the finger chains
- Joint limits for proper IK constraints
- Proper parent-child relationships

## Troubleshooting

1. **IK doesn't converge**: Check that your target position is reachable and within joint limits
2. **Node crashes on startup**: Verify that the URDF path is correct and the file is valid
3. **Unexpected results**: Ensure that the joint names in the URDF match the expected names in the code

## License

MIT License
