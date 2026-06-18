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

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include "intubation_interfaces/msg/compliance_param.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace intubation_controller {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;
  bool initialization_flag_{true};
  uint64_t seq_count = 0;
  uint64_t last_pose_seq = 0;
  uint64_t last_param_seq = 0;
  const bool k_elbow_activated_{false};
  const double kAlpha = 0.99;
  const double filter_params_ = 0.0005;
  Vector7d initial_q_;
  Vector7d initial_dq_;
  Vector7d q_d_nullspace_;
  Vector7d dq_filtered_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Matrix<double, 6, 1> error_i;
  Eigen::Matrix<double, 6, 1> k_gains_;
  Eigen::Matrix<double, 6, 1> d_ratio_;
  Eigen::Matrix<double, 6, 1> ki_gains_;
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_;
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_integral_gains_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  const Eigen::Vector3d translational_clip_min_{-0.01, -0.01, -0.01};
  const Eigen::Vector3d translational_clip_max_{0.01, 0.01, 0.01};
  const Eigen::Vector3d rotational_clip_min_{-0.05, -0.05, -0.05};
  const Eigen::Vector3d rotational_clip_max_{0.05, 0.05, 0.05};
  
  void equilibriumPose();

  Vector7d saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_J_d);

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;
  
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> wrench_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> s_publisher_;
  geometry_msgs::msg::WrenchStamped msg;
  
  void equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void complianceParamCallback(
    const intubation_interfaces::msg::ComplianceParam::SharedPtr msg);
 
  struct EquilibriumPoseCommand
  {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    uint64_t seq = 0;
  };

  struct ComplianceCommand
  {
    Eigen::Matrix<double, 6, 1> k_gains;
    Eigen::Matrix<double, 6, 1> d_ratio;
    double nullspace_stiffness;
    uint64_t seq = 0;
  };
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_sub_;
  realtime_tools::RealtimeBuffer<EquilibriumPoseCommand> equilibrium_buffer_;
  rclcpp::Subscription<intubation_interfaces::msg::ComplianceParam>::SharedPtr compliance_sub_;
  realtime_tools::RealtimeBuffer<ComplianceCommand> compliance_buffer_;
};

}  // namespace intubation_controller