#include <rclcpp/rclcpp.hpp>
#include <hand_kinematics/srv/forward_kinematics.hpp>
#include <hand_kinematics/srv/inverse_kinematics.hpp>
#include <chrono>
#include <cmath>

class HandKinematicsTestClient : public rclcpp::Node {
public:
    HandKinematicsTestClient() : Node("hand_kinematics_test_client") {
        // Create service clients
        fk_client_ = this->create_client<hand_kinematics::srv::ForwardKinematics>("hand/forward_kinematics");
        ik_client_ = this->create_client<hand_kinematics::srv::InverseKinematics>("hand/inverse_kinematics");
        
        // Create timer for demonstrations
        timer_ = this->create_wall_timer(
            std::chrono::seconds(8),  // Increased time to see results
            std::bind(&HandKinematicsTestClient::timerCallback, this));
        
        demo_step_ = 0;
        last_fk_result_.reset();  // Initialize as null
        
        // Wait for services to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for services...");
        while (!fk_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Still waiting for FK service...");
        }
        
        while (!ik_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Still waiting for IK service...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Services are ready! Starting tests...");
    }

private:
    rclcpp::Client<hand_kinematics::srv::ForwardKinematics>::SharedPtr fk_client_;
    rclcpp::Client<hand_kinematics::srv::InverseKinematics>::SharedPtr ik_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int demo_step_;
    std::shared_ptr<hand_kinematics::srv::ForwardKinematics::Response> last_fk_result_;
    std::string current_test_chain_;
    
    void timerCallback() {
        RCLCPP_INFO(this->get_logger(), "\n========== Test Step %d ==========", demo_step_);
        
        switch (demo_step_ % 3) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Test: FK->IK verification for index finger");
                runFKtoIKTest("index", {15.0, 25.0, 35.0});
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Test: FK->IK verification for middle finger");
                runFKtoIKTest("middle", {20.0, 30.0, 40.0});
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Test: FK->IK verification for thumb");
                runFKtoIKTest("thumb", {10.0, 15.0, 20.0, 25.0});
                break;
        }
        demo_step_++;
    }
    
    void runFKtoIKTest(const std::string& chain_name, const std::vector<double>& test_angles) {
        current_test_chain_ = chain_name;
        
        // Step 1: Run FK with test angles
        RCLCPP_INFO(this->get_logger(), "\n--- Step 1: Forward Kinematics ---");
        RCLCPP_INFO(this->get_logger(), "Testing %s finger with joint angles:", chain_name.c_str());
        for (size_t i = 0; i < test_angles.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.1f degrees", i, test_angles[i]);
        }
        
        auto fk_request = std::make_shared<hand_kinematics::srv::ForwardKinematics::Request>();
        fk_request->chain_name = chain_name;
        fk_request->joint_angles_deg = test_angles;
        
        // Use lambda to capture the test flow
        auto future = fk_client_->async_send_request(
            fk_request,
            [this, test_angles](rclcpp::Client<hand_kinematics::srv::ForwardKinematics>::SharedFuture future) {
                this->handleFKResponse(future, test_angles);
            });
    }
    
    void handleFKResponse(rclcpp::Client<hand_kinematics::srv::ForwardKinematics>::SharedFuture future,
                         const std::vector<double>& original_angles) {
        try {
            last_fk_result_ = future.get();
            
            if (last_fk_result_->success) {
                RCLCPP_INFO(this->get_logger(), "FK Result:");
                RCLCPP_INFO(this->get_logger(), "  Tip Position: [%.4f, %.4f, %.4f] meters",
                           last_fk_result_->tip_position.x, 
                           last_fk_result_->tip_position.y, 
                           last_fk_result_->tip_position.z);
                
                // Step 2: Use FK result as IK target
                RCLCPP_INFO(this->get_logger(), "\n--- Step 2: Inverse Kinematics ---");
                RCLCPP_INFO(this->get_logger(), "Using FK result as IK target position");
                
                auto ik_request = std::make_shared<hand_kinematics::srv::InverseKinematics::Request>();
                ik_request->chain_name = current_test_chain_;
                ik_request->target_position = last_fk_result_->tip_position;
                
                // Use a different initial guess to test convergence
                if (original_angles.size() == 3) {
                    ik_request->initial_guess_deg = {0.0, 0.0, 0.0};
                } else {  // thumb with 4 angles
                    ik_request->initial_guess_deg = {0.0, 0.0, 0.0, 0.0};
                }
                
                // Call IK
                auto ik_future = ik_client_->async_send_request(
                    ik_request,
                    [this, original_angles](rclcpp::Client<hand_kinematics::srv::InverseKinematics>::SharedFuture future) {
                        this->handleIKResponse(future, original_angles);
                    });
            } else {
                RCLCPP_ERROR(this->get_logger(), "FK failed: %s", last_fk_result_->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "FK service call failed: %s", e.what());
        }
    }
    
    void handleIKResponse(rclcpp::Client<hand_kinematics::srv::InverseKinematics>::SharedFuture future,
                         const std::vector<double>& original_angles) {
        try {
            auto ik_response = future.get();
            
            if (ik_response->success) {
                RCLCPP_INFO(this->get_logger(), "IK Result:");
                RCLCPP_INFO(this->get_logger(), "  Chain: %s", current_test_chain_.c_str());
                RCLCPP_INFO(this->get_logger(), "  Method: %s", ik_response->method.c_str());
                
                // Display calculated joint angles
                RCLCPP_INFO(this->get_logger(), "  Computed joint angles:");
                for (size_t i = 0; i < ik_response->joint_angles_deg.size(); i++) {
                    RCLCPP_INFO(this->get_logger(), "    Joint %zu: %.2f degrees", i, ik_response->joint_angles_deg[i]);
                }
                
                // Display actual achieved position
                RCLCPP_INFO(this->get_logger(), "  Actual position: [%.4f, %.4f, %.4f] meters",
                           ik_response->actual_position.x, 
                           ik_response->actual_position.y, 
                           ik_response->actual_position.z);
                
                // Step 3: Compare original FK result with IK achieved position
                RCLCPP_INFO(this->get_logger(), "\n--- Step 3: Verification ---");
                double position_error = calculatePositionError(last_fk_result_->tip_position, 
                                                             ik_response->actual_position);
                RCLCPP_INFO(this->get_logger(), "Position error (||FK - IK||): %.6f meters", position_error);
                
                if (position_error < 0.001) {  // 1mm tolerance
                    RCLCPP_INFO(this->get_logger(), "SUCCESS: FK and IK are consistent!");
                } else if (position_error < 0.01) {  // 1cm tolerance
                    RCLCPP_WARN(this->get_logger(), "WARNING: Small inconsistency between FK and IK");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "ERROR: Large inconsistency between FK and IK");
                }
                
                // Compare joint angles
                RCLCPP_INFO(this->get_logger(), "\nJoint angle comparison:");
                for (size_t i = 0; i < original_angles.size(); i++) {
                    double angle_diff = std::abs(original_angles[i] - ik_response->joint_angles_deg[i]);
                    RCLCPP_INFO(this->get_logger(), "  Joint %zu: Original=%.2f°, IK=%.2f°, Diff=%.2f°", 
                               i, original_angles[i], ik_response->joint_angles_deg[i], angle_diff);
                }
                
            } else {
                RCLCPP_ERROR(this->get_logger(), "IK failed: %s", ik_response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IK service call failed: %s", e.what());
        }
        
        RCLCPP_INFO(this->get_logger(), "\n========== Test Complete ==========\n");
    }
    
    double calculatePositionError(const geometry_msgs::msg::Point& p1, 
                                 const geometry_msgs::msg::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HandKinematicsTestClient>();
    
    // Create executor and add node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Spin the executor
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}