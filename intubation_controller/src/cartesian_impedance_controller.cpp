// Copyright (c) 2023 Franka Robotics GmbH
// Copyright (c) 2026 Konstantinos Gourgiotis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <intubation_controller/cartesian_impedance_controller.hpp>
#include <intubation_controller/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace intubation_controller {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  for (const auto& franka_robot_state_name : franka_robot_state_->get_state_interface_names()) {
    config.names.push_back(franka_robot_state_name);
  }

  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
    
  if (initialization_flag_) {
    equilibriumPose();
    initialization_flag_ = false;
  }
  
  franka_msgs::msg::FrankaRobotState robot_state;
  franka_robot_state_->get_values_as_message(robot_state);

  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array = 
    franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  
  Eigen::Map<Vector7d> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());    
  Eigen::Map<Vector7d> q(robot_state.measured_joint_state.position.data());
  Eigen::Map<Vector7d> dq(robot_state.measured_joint_state.velocity.data());
  Eigen::Map<Vector7d> tau_J_d(robot_state.desired_joint_state.effort.data());
  
  std::array<double, 16> pose_array = franka_cartesian_pose_->getCurrentPoseMatrix();
  Eigen::Affine3d transform(Eigen::Map<const Eigen::Matrix4d>(pose_array.data()));
  
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  std::tie(orientation, position) =
    franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;
  
  // clip position error
  for (int i = 0; i < 3; i++) {
    error(i) = std::min(std::max(error(i), translational_clip_min_(i)), translational_clip_max_(i));
  }
  
  // orientation error
  // Flip quaternion if needed to maintain shortest-path rotation
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);
  
  // clip orientation error
  for (int i = 0; i < 3; i++) {
    error(i+3) = std::min(std::max(error(i+3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }
  
  error_i.head(3) << (error_i.head(3) + error.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i.tail(3) << (error_i.tail(3) + error.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);
  
  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // Low-pass filter joint velocities to reduce sensor noise
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq;
    
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse  
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  
  // Cartesian PD control
  Eigen::Matrix<double, 6, 1> wrench;
  wrench = - (cartesian_stiffness_ * error) - (cartesian_damping_ * (jacobian * dq_filtered_)) - (cartesian_integral_gains_ * error_i);
  tau_task << jacobian.transpose() * wrench;
  // nullspace PD control
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq_filtered_);
  // Desired torque
  //tau_d << tau_task + tau_nullspace + coriolis;
  tau_d << tau_task + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  msg.header.stamp = get_node()->now();
  msg.wrench.force.x  = wrench(0);
  msg.wrench.force.y  = wrench(1);
  msg.wrench.force.z  = wrench(2);
  msg.wrench.torque.x = wrench(3);
  msg.wrench.torque.y = wrench(4);
  msg.wrench.torque.z = wrench(5);

  wrench_pub_->tryPublish(msg);
   
  const auto* pose_cmd = equilibrium_buffer_.readFromRT();
  const auto* param_cmd = compliance_buffer_.readFromRT();
  
  if (pose_cmd && pose_cmd->seq != last_pose_seq) 
  {
    last_pose_seq = pose_cmd->seq;
    error_i.setZero();
    position_d_target_ = pose_cmd->position;
    orientation_d_target_ = pose_cmd->orientation;
  }

  if (param_cmd && param_cmd->seq != last_param_seq)
  {
    last_param_seq = param_cmd->seq;
    cartesian_stiffness_target_ = param_cmd->k_gains.asDiagonal();
    cartesian_damping_target_ = param_cmd->d_ratio.asDiagonal() * 2.0 * cartesian_stiffness_target_.cwiseSqrt();
    nullspace_stiffness_target_ = param_cmd->nullspace_stiffness;
    std::cout << "Cartesian stiffness:\n"
            << cartesian_stiffness_target_ << std::endl;
    std::cout << "Cartesian damping:\n"
            << cartesian_damping_target_ << std::endl;
    std::cout << "Nullspace stiffness: "
            << nullspace_stiffness_target_ << std::endl;
  }

  cartesian_stiffness_ =
    filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
    filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
    filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  return controller_interface::return_type::OK;
}

Vector7d CartesianImpedanceController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_J_d) {
  Vector7d tau_d_saturated{};
  for (int i = 0; i < num_joints; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_ratio", {});
    auto_declare<std::vector<double>>("ki_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));
          
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_ratio = get_node()->get_parameter("d_ratio").as_double_array();
  auto ki_gains = get_node()->get_parameter("ki_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_ratio.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_ratio parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (ki_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "ki_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(6)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size 6 but is of size %ld",
                 k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_ratio.size() != static_cast<uint>(6)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_ratio should be of size 6 but is of size %ld",
                 d_ratio.size());
    return CallbackReturn::FAILURE;
  }
  if (ki_gains.size() != static_cast<uint>(6)) {
    RCLCPP_FATAL(get_node()->get_logger(), "ki_gains should be of size 6 but is of size %ld",
                 ki_gains.size());
    return CallbackReturn::FAILURE;
  }
  
  for (int i = 0; i < 6; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_ratio_(i) = d_ratio.at(i);
    ki_gains_(i) = ki_gains.at(i);
  }
  
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_integral_gains_.setZero();
  cartesian_stiffness_ = k_gains_.asDiagonal();
  cartesian_damping_ = d_ratio_.asDiagonal() * 2.0 * cartesian_stiffness_.cwiseSqrt();
  cartesian_integral_gains_ = ki_gains_.asDiagonal();
  cartesian_stiffness_target_ = cartesian_stiffness_;
  cartesian_damping_target_ = cartesian_damping_;
  nullspace_stiffness_target_ = nullspace_stiffness_;
  
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
        
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));      
  
  franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
      arm_id_ + "/" + k_robot_state_interface_name, robot_description_);
  
  s_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/reactive_wrench", rclcpp::QoS(1).best_effort());
  
  wrench_pub_ = 
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(s_publisher_);

  msg.header.frame_id = "fr3_link0";
  
  equilibrium_sub_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/equilibrium_pose",
      rclcpp::QoS(1).reliable(),
      std::bind(
        &CartesianImpedanceController::equilibriumPoseCallback,
        this,
        std::placeholders::_1));
  
  compliance_sub_ =
    get_node()->create_subscription<intubation_interfaces::msg::ComplianceParam>(
      "/compliance_param",
      rclcpp::QoS(1).reliable(),
      std::bind(
        &CartesianImpedanceController::complianceParamCallback,
        this,
        std::placeholders::_1));
   
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  initial_q_.setZero();
  initial_dq_.setZero();
  q_d_nullspace_.setZero();
  dq_filtered_.setZero();
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  error_i.setZero();

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::equilibriumPose() {
  franka_msgs::msg::FrankaRobotState initial_state;
  franka_robot_state_->get_values_as_message(initial_state);
  initial_q_ = Eigen::Map<const Vector7d>(initial_state.measured_joint_state.position.data());
  initial_dq_ = Eigen::Map<const Vector7d>(initial_state.measured_joint_state.velocity.data());
  q_d_nullspace_ = initial_q_;
  dq_filtered_ = initial_dq_;
  std::tie(orientation_d_, position_d_) =
    franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;
}

void CartesianImpedanceController::equilibriumPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  EquilibriumPoseCommand cmd;
  cmd.position << msg->pose.position.x, msg->pose.position.y,
                  msg->pose.position.z;
  cmd.orientation.x() = msg->pose.orientation.x;
  cmd.orientation.y() = msg->pose.orientation.y;
  cmd.orientation.z() = msg->pose.orientation.z;
  cmd.orientation.w() = msg->pose.orientation.w;
  
  const auto* last_cmd = equilibrium_buffer_.readFromNonRT();
  if (last_cmd && last_cmd->orientation.coeffs().dot(cmd.orientation.coeffs()) < 0.0)
  {
    cmd.orientation.coeffs() << -cmd.orientation.coeffs();
  }
  
  cmd.seq = ++seq_count;
  equilibrium_buffer_.writeFromNonRT(cmd);
}

void CartesianImpedanceController::complianceParamCallback(
  const intubation_interfaces::msg::ComplianceParam::SharedPtr msg)
{
  ComplianceCommand cmd;
  cmd.k_gains << msg->translational_stiffness[0], msg->translational_stiffness[1],
    msg->translational_stiffness[2], msg->rotational_stiffness[0],
    msg->rotational_stiffness[1], msg->rotational_stiffness[2];
  
  cmd.d_ratio << msg->translational_damping[0], msg->translational_damping[1],
    msg->translational_damping[2], msg->rotational_damping[0],
    msg->rotational_damping[1], msg->rotational_damping[2];

  cmd.nullspace_stiffness = msg->nullspace_stiffness;

  cmd.seq = ++seq_count;
  compliance_buffer_.writeFromNonRT(cmd);
}

}  // namespace intubation_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(intubation_controller::CartesianImpedanceController,
                       controller_interface::ControllerInterface)