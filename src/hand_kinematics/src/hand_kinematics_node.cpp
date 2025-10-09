#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <hand_kinematics/srv/forward_kinematics.hpp>
#include <hand_kinematics/srv/inverse_kinematics.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <ceres/ceres.h>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

namespace hand_kinematics {

struct JointInfo {
    std::string type;
    std::string parent;
    std::string child;
    Eigen::Vector3d xyz;
    Eigen::Vector3d rpy;
    Eigen::Matrix4d transform;
    double lower_limit;
    double upper_limit;
    Eigen::Vector3d axis;
    bool movable;
};

struct ChainInfo {
    std::vector<std::string> in_joints;
    std::string df_joint;
    std::string tip_joint;
};

class HandKinematics : public rclcpp::Node {
public:
    HandKinematics() : Node("hand_kinematics") {
        // Define finger chains
        finger_chains_["index"] = ChainInfo{
            {"ima_joint", "imf_joint", "ipf_joint"}, 
            "idf_joint", 
            "i_tip_joint"
        };
        finger_chains_["middle"] = ChainInfo{
            {"mma_joint", "mmf_joint", "mpf_joint"}, 
            "mdf_joint", 
            "m_tip_joint"
        };
        finger_chains_["ring"] = ChainInfo{
            {"rma_joint", "rmf_joint", "rpf_joint"}, 
            "rdf_joint", 
            "r_tip_joint"
        };
        finger_chains_["pinky"] = ChainInfo{
            {"pma_joint", "pmf_joint", "ppf_joint"}, 
            "pdf_joint", 
            "p_tip_joint"
        };
        
        // Define thumb chain
        thumb_chain_ = ChainInfo{
            {"tcf_joint", "tca_joint", "tma_joint", "tmf_joint"},
            "tdf_joint",
            "t_tip_joint"
        };
        
        // Parameters
        this->declare_parameter("urdf_path", "");
        std::string urdf_path = this->get_parameter("urdf_path").as_string();
        
        if (urdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "URDF path not specified!");
            return;
        }
        
        // Parse URDF
        if (!parseURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return;
        }
        
        buildKinematicTree();
        
        // Create services
        fk_service_ = this->create_service<hand_kinematics::srv::ForwardKinematics>(
            "hand/forward_kinematics",
            std::bind(&HandKinematics::forwardKinematicsCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        ik_service_ = this->create_service<hand_kinematics::srv::InverseKinematics>(
            "hand/inverse_kinematics",
            std::bind(&HandKinematics::inverseKinematicsCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Member variables
    std::map<std::string, ChainInfo> finger_chains_;
    ChainInfo thumb_chain_;
    std::map<std::string, JointInfo> joints_;
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> tree_;
    std::string root_link_;
    
    // Services
    rclcpp::Service<hand_kinematics::srv::ForwardKinematics>::SharedPtr fk_service_;
    rclcpp::Service<hand_kinematics::srv::InverseKinematics>::SharedPtr ik_service_;
    
    bool parseURDF(const std::string& urdf_path) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file: %s", urdf_path.c_str());
            return false;
        }
        
        tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
        if (!robot) {
            RCLCPP_ERROR(this->get_logger(), "No robot element found in URDF");
            return false;
        }
        
        // Parse joints
        for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint");
             joint != nullptr; joint = joint->NextSiblingElement("joint")) {
            
            JointInfo info;
            info.type = joint->Attribute("type");
            
            tinyxml2::XMLElement* parent = joint->FirstChildElement("parent");
            tinyxml2::XMLElement* child = joint->FirstChildElement("child");
            
            if (!parent || !child) continue;
            
            info.parent = parent->Attribute("link");
            info.child = child->Attribute("link");
            
            // Parse origin
            tinyxml2::XMLElement* origin = joint->FirstChildElement("origin");
            info.xyz = parseXYZ(origin);
            info.rpy = parseRPY(origin);
            info.transform = createTransformMatrix(info.xyz, info.rpy);
            
            // Parse limits
            tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
            if (limit) {
                info.lower_limit = std::stod(limit->Attribute("lower") ? limit->Attribute("lower") : "0.0");
                info.upper_limit = std::stod(limit->Attribute("upper") ? limit->Attribute("upper") : "0.0");
            } else {
                info.lower_limit = 0.0;
                info.upper_limit = 0.0;
            }
            
            // Parse axis
            tinyxml2::XMLElement* axis = joint->FirstChildElement("axis");
            if (axis && axis->Attribute("xyz")) {
                info.axis = parseXYZ(axis);
                info.axis.normalize();
            } else {
                info.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
            }
            
            info.movable = (info.type != "fixed");
            
            joints_[joint->Attribute("name")] = info;
        }
        
        return true;
    }
    
    Eigen::Vector3d parseXYZ(tinyxml2::XMLElement* element) {
        if (!element || !element->Attribute("xyz")) {
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        
        std::string xyz_str = element->Attribute("xyz");
        std::istringstream iss(xyz_str);
        double x, y, z;
        iss >> x >> y >> z;
        return Eigen::Vector3d(x, y, z);
    }
    
    Eigen::Vector3d parseRPY(tinyxml2::XMLElement* element) {
        if (!element || !element->Attribute("rpy")) {
            return Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        
        std::string rpy_str = element->Attribute("rpy");
        std::istringstream iss(rpy_str);
        double r, p, y;
        iss >> r >> p >> y;
        return Eigen::Vector3d(r, p, y);
    }
    
    Eigen::Matrix4d createTransformMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // Create rotation matrix from RPY
        Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());
        
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) = xyz;
        
        return T;
    }
    
    void buildKinematicTree() {
        std::set<std::string> all_links;
        std::set<std::string> child_links;
        
        // Build the tree structure
        for (const auto& [joint_name, joint_info] : joints_) {
            all_links.insert(joint_info.parent);
            all_links.insert(joint_info.child);
            child_links.insert(joint_info.child);
            
            tree_[joint_info.parent].push_back({joint_info.child, joint_name});
        }
        
        // Find root link (not a child of any joint)
        for (const auto& link : all_links) {
            if (child_links.find(link) == child_links.end()) {
                root_link_ = link;
                break;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Root link: %s", root_link_.c_str());
    }
    
    static double computeDFFromPF(double pf_deg) {
        if (std::abs(pf_deg + 65.705) < 1e-6) {
            return 0.0;
        }
        
        double pf_rad = pf_deg * M_PI / 180.0;
        double result = 2.0 * std::atan((1.0/3.0) / std::tan((114.295 * M_PI / 180.0 - pf_rad) / 2.0));
        return result * 180.0 / M_PI - 24.295;
    }
    
    static double computeTDFFromTMF(double tmf_deg) {
        if (std::abs(tmf_deg + 64.841) < 1e-6) {
            return 0.0;
        }
        
        double tmf_rad = tmf_deg * M_PI / 180.0;
        double result = 2.0 * std::atan((13.0/37.0) / std::tan((115.159 * M_PI / 180.0 - tmf_rad) / 2.0));
        return result * 180.0 / M_PI - 25.159;
    }
    
    ChainInfo getChain(const std::string& name) {
        if (name == "thumb") {
            return thumb_chain_;
        }
        
        auto it = finger_chains_.find(name);
        if (it != finger_chains_.end()) {
            return it->second;
        }
        
        throw std::runtime_error("Unknown chain: " + name);
    }
    
    std::map<std::string, double> computeAngleMap(const std::string& chain_name, 
                                                  const std::vector<double>& angles_deg) {
        ChainInfo chain = getChain(chain_name);
        
        if (angles_deg.size() != chain.in_joints.size()) {
            throw std::runtime_error("Angle count mismatch for chain " + chain_name);
        }
        
        std::map<std::string, double> amap;
        
        // Convert degrees to radians for in_joints
        for (size_t i = 0; i < chain.in_joints.size(); i++) {
            amap[chain.in_joints[i]] = angles_deg[i] * M_PI / 180.0;
        }
        
        // Apply coupling - using the unified function for all fingers including thumb
        if (!chain.df_joint.empty()) {
            double last_angle_rad = angles_deg.back() * M_PI / 180.0;
            double df_rad = computeDFFromPF(last_angle_rad);
            amap[chain.df_joint] = df_rad;
        }
        
        return amap;
    }
    
    std::map<std::string, Eigen::Matrix4d> calculateLinkTransforms(
            const std::map<std::string, double>& joint_angles) {
        
        std::map<std::string, Eigen::Matrix4d> link_transforms;
        link_transforms[root_link_] = Eigen::Matrix4d::Identity();
        
        // Traverse the tree and compute transforms
        std::function<void(const std::string&, const Eigen::Matrix4d&)> traverseTree;
        traverseTree = [&](const std::string& link, const Eigen::Matrix4d& parent_transform) {
            auto tree_it = tree_.find(link);
            if (tree_it == tree_.end()) return;
            
            for (const auto& [child_link, joint_name] : tree_it->second) {
                const JointInfo& joint_info = joints_[joint_name];
                
                // Start with fixed transform
                Eigen::Matrix4d joint_transform = joint_info.transform;
                
                // Apply joint angle if movable
                if (joint_info.movable) {
                    auto angle_it = joint_angles.find(joint_name);
                    if (angle_it != joint_angles.end()) {
                        double angle = angle_it->second;
                        
                        if (joint_info.type == "revolute" || joint_info.type == "continuous") {
                            Eigen::AngleAxisd rotation(angle, joint_info.axis);
                            joint_transform.block<3,3>(0,0) *= rotation.toRotationMatrix();
                        } else if (joint_info.type == "prismatic") {
                            joint_transform.block<3,1>(0,3) += joint_info.axis * angle;
                        }
                    }
                }
                
                // Compute absolute transform
                Eigen::Matrix4d abs_transform = parent_transform * joint_transform;
                link_transforms[child_link] = abs_transform;
                
                // Recurse
                traverseTree(child_link, abs_transform);
            }
        };
        
        traverseTree(root_link_, link_transforms[root_link_]);
        return link_transforms;
    }
    
    Eigen::Vector3d computeTipPosition(const std::string& chain_name, 
                                       const std::vector<double>& angles_deg) {
        auto angle_map = computeAngleMap(chain_name, angles_deg);
        auto transforms = calculateLinkTransforms(angle_map);
        
        ChainInfo chain = getChain(chain_name);
        const JointInfo& tip_joint = joints_[chain.tip_joint];
        
        return transforms[tip_joint.child].block<3,1>(0,3);
    }
    
    // Ceres cost function for IK
    class IKCostFunction : public ceres::CostFunction {
    public:
        IKCostFunction(HandKinematics* hand, const std::string& chain_name, 
                       const Eigen::Vector3d& target) 
            : hand_(hand), chain_name_(chain_name), target_(target) {
            
            // Get number of parameters (in_joints)
            ChainInfo chain = hand_->getChain(chain_name);
            int n_params = chain.in_joints.size();
            
            set_num_residuals(3);  // x, y, z
            mutable_parameter_block_sizes()->push_back(n_params);
        }
        
        bool Evaluate(double const* const* parameters,
                      double* residuals,
                      double** jacobians) const override {
            
            // Convert parameters to vector
            ChainInfo chain = hand_->getChain(chain_name_);
            int n_params = chain.in_joints.size();
            std::vector<double> angles_deg(n_params);
            for (int i = 0; i < n_params; i++) {
                angles_deg[i] = parameters[0][i];
            }
            
            // Compute tip position
            Eigen::Vector3d tip_pos = hand_->computeTipPosition(chain_name_, angles_deg);
            
            // Compute residuals
            Eigen::Vector3d error = tip_pos - target_;
            residuals[0] = error[0];
            residuals[1] = error[1];
            residuals[2] = error[2];
            
            // Numerical jacobian if requested
            if (jacobians && jacobians[0]) {
                const double h = 1e-6;
                for (int j = 0; j < n_params; j++) {
                    std::vector<double> angles_perturbed = angles_deg;
                    angles_perturbed[j] += h;
                    
                    Eigen::Vector3d tip_pos_perturbed = hand_->computeTipPosition(chain_name_, angles_perturbed);
                    Eigen::Vector3d error_perturbed = tip_pos_perturbed - target_;
                    
                    // Finite difference approximation
                    for (int i = 0; i < 3; i++) {
                        jacobians[0][i * n_params + j] = (error_perturbed[i] - error[i]) / h;
                    }
                }
            }
            
            return true;
        }
        
    private:
        HandKinematics* hand_;
        std::string chain_name_;
        Eigen::Vector3d target_;
    };
    
    bool solveIK(const std::string& chain_name, const Eigen::Vector3d& target_pos,
                 std::vector<double>& solution, const std::vector<double>* initial_guess = nullptr) {
        
        ChainInfo chain = getChain(chain_name);
        int n_vars = chain.in_joints.size();
        
        // Initialize solution
        solution.resize(n_vars);
        if (initial_guess && static_cast<int>(initial_guess->size()) == n_vars) {
            solution = *initial_guess;
        } else {
            std::fill(solution.begin(), solution.end(), 0.0);
        }
        
        // Set up Ceres problem
        ceres::Problem problem;
        
        // Add cost function
        ceres::CostFunction* cost_function = new IKCostFunction(this, chain_name, target_pos);
        problem.AddResidualBlock(cost_function, nullptr, solution.data());
        
        // Add joint limits
        for (int i = 0; i < n_vars; i++) {
            const JointInfo& joint_info = joints_[chain.in_joints[i]];
            
            problem.SetParameterLowerBound(solution.data(), i, joint_info.lower_limit * 180.0 / M_PI);
            problem.SetParameterUpperBound(solution.data(), i, joint_info.upper_limit * 180.0 / M_PI);
        }
        
        // Configure solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 100;
        options.function_tolerance = 1e-8;
        
        // Solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
        return summary.IsSolutionUsable();
    }
    
    void forwardKinematicsCallback(
        const std::shared_ptr<hand_kinematics::srv::ForwardKinematics::Request> request,
        std::shared_ptr<hand_kinematics::srv::ForwardKinematics::Response> response) {
        
        try {
            // Validate chain name
            ChainInfo chain = getChain(request->chain_name);
            
            // Validate joint angles count
            if (request->joint_angles_deg.size() != chain.in_joints.size()) {
                response->success = false;
                response->message = "Invalid number of joint angles for chain " + request->chain_name + 
                                  ". Expected " + std::to_string(chain.in_joints.size()) +
                                  ", got " + std::to_string(request->joint_angles_deg.size());
                return;
            }
            
            // Compute FK
            Eigen::Vector3d tip_pos = computeTipPosition(request->chain_name, request->joint_angles_deg);
            
            // Populate response
            response->success = true;
            response->tip_position.x = tip_pos[0];
            response->tip_position.y = tip_pos[1];
            response->tip_position.z = tip_pos[2];
            response->message = "Forward kinematics computed successfully";
            
            RCLCPP_INFO(this->get_logger(), "FK for %s: [%.3f, %.3f, %.3f]", 
                       request->chain_name.c_str(), tip_pos[0], tip_pos[1], tip_pos[2]);
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "FK error: %s", e.what());
        }
    }
    
    void inverseKinematicsCallback(
        const std::shared_ptr<hand_kinematics::srv::InverseKinematics::Request> request,
        std::shared_ptr<hand_kinematics::srv::InverseKinematics::Response> response) {
        
        try {
            // Validate chain name
            ChainInfo chain = getChain(request->chain_name);
            
            // Prepare target position
            Eigen::Vector3d target(request->target_position.x, request->target_position.y, request->target_position.z);
            
            // Prepare initial guess if provided
            std::vector<double> initial_guess;
            if (!request->initial_guess_deg.empty()) {
                if (request->initial_guess_deg.size() != chain.in_joints.size()) {
                    response->success = false;
                    response->message = "Invalid number of initial guess angles for chain " + request->chain_name;
                    return;
                }
                initial_guess = request->initial_guess_deg;
            }
            
            // Solve IK
            std::vector<double> solution;
            bool success = solveIK(request->chain_name, target, solution, 
                                 initial_guess.empty() ? nullptr : &initial_guess);
            
            // Populate response
            response->success = success;
            if (success) {
                response->joint_angles_deg = solution;
                response->method = "ceres";  // We're using Ceres for optimization
                
                // Verify by computing FK
                Eigen::Vector3d actual_pos = computeTipPosition(request->chain_name, solution);
                response->actual_position.x = actual_pos[0];
                response->actual_position.y = actual_pos[1];
                response->actual_position.z = actual_pos[2];
                
                response->message = "IK converged successfully";
                
                RCLCPP_INFO(this->get_logger(), "IK solution for %s:", request->chain_name.c_str());
                for (size_t i = 0; i < solution.size(); i++) {
                    RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.2f deg", i, solution[i]);
                }
                RCLCPP_INFO(this->get_logger(), "  Actual position: [%.3f, %.3f, %.3f]", 
                           actual_pos[0], actual_pos[1], actual_pos[2]);
            } else {
                response->message = "IK failed to converge";
                RCLCPP_WARN(this->get_logger(), "IK failed for %s", request->chain_name.c_str());
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "IK error: %s", e.what());
        }
    }
    

};

} // namespace hand_kinematics

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hand_kinematics::HandKinematics>());
    rclcpp::shutdown();
    return 0;
}