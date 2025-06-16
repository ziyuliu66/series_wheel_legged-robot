// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef LQR_EFFORT_CONTROLLER__LQR_EFFORT_CONTROLLER_HPP_
#define LQR_EFFORT_CONTROLLER__LQR_EFFORT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <utility>

#include "controller_interface/controller_interface.hpp"
#include "lqr_effort_controller_parameters.hpp"
#include "lqr_effort_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


namespace lqr_effort_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

static constexpr double PI = 3.14159265358979323846;  

static const float thigh_link_length = 0.3;

static const float shank_link_length = 0.3;

static const float rad2angle = 180/PI;

static const float wheel_radius = 0.1;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

// 机器人状态结构体
struct RobotState {
    // 欧拉角（单位：弧度）
    float euler_roll = 0.0;    // 绕X轴的旋转角（横滚角）
    float euler_pitch = 0.0;   // 绕Y轴的旋转角（俯仰角）
    float euler_yaw = 0.0;     // 绕Z轴的旋转角（偏航角）

    // 欧拉角速度（单位：弧度/秒）
    float euler_roll_velocity = 0.0;   // 横滚角速度
    float euler_pitch_velocity = 0.0;  // 俯仰角速度
    float euler_yaw_velocity = 0.0;    // 偏航角速度

    // 三轴加速度（单位：m/s²）
    float x_acceleration = 0.0;  // X轴加速度（前向）
    float y_acceleration = 0.0;  // Y轴加速度（侧向）
    float z_acceleration = 0.0;  // Z轴加速度（垂直）

    // 轮子线速度（单位：m/s）
    float left_wheel_velocity = 0.0;   // 左轮线速度
    float right_wheel_velocity = 0.0;  // 右轮线速度

    //各关节角度（单位：弧度/秒）
    float left_hip_angle = 0.0, left_knee_angle = 0.0;
    float right_hip_angle = 0.0, right_knee_angle = 0.0;

    //theta0为髋关节计算角度，theta1为膝关节计算角度
    float left_theta0 = 0.0, left_theta1 = 0.0; 
    float right_theta0 = 0.0, right_theta1 = 0.0;

    //l0为虚拟摆杆腿长，phi0为虚拟摆杆倾角
    float left_l0 = 0.0, left_phi0 = 0.0, left_d_phi0 = 0.0;
    float right_l0 = 0.0, right_phi0 = 0.0, right_d_phi0 = 0.0;

    //F0为虚拟摆杆推力，Tp为虚拟摆杆转矩
    float left_F0 = 0.0, left_Tp = 0.0;
    float right_F0 = 0.0, right_Tp = 0.0;

    float left_T0 = 0.0, left_T1 = 0.0;
    float right_T0 = 0.0, right_T1 = 0.0;

    float car_mean_velocity = 0.0;
    float car_mean_displacement = 0.0;
};

class LqrEffortController : public controller_interface::ControllerInterface
{
public:
  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  LqrEffortController();

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  LQR_EFFORT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<lqr_effort_controller::ParamListener> param_listener_;
  lqr_effort_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  LQR_EFFORT_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);
  void joint_states_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
  void L0_PHI0(const float theta0, const float theta1, float& L0, float& phi0);
  void VMC_calc(float F0, float Tp, float theta0, float theta1, float& T0, float& T1);

  RobotState robotstate_;
};

}  // namespace lqr_effort_controller

#endif  // LQR_EFFORT_CONTROLLER__LQR_EFFORT_CONTROLLER_HPP_
