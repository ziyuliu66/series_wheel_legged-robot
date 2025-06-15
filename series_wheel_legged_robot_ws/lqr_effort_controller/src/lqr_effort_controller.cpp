// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "lqr_effort_controller/lqr_effort_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = lqr_effort_controller::LqrEffortController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace lqr_effort_controller
{
LqrEffortController::LqrEffortController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn LqrEffortController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<lqr_effort_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LqrEffortController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&LqrEffortController::reference_callback, this, std::placeholders::_1));

  imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
    "imu", subscribers_qos,
    std::bind(&LqrEffortController::imu_callback, this, std::placeholders::_1));

  joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", subscribers_qos,
    std::bind(&LqrEffortController::joint_states_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void LqrEffortController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
}

void LqrEffortController::imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
  // 提取四元数
  tf2::Quaternion quat(msg->orientation.x , msg->orientation.y , msg->orientation.z , msg->orientation.w);
  // 转换为欧拉角 (弧度)
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  robotstate_.euler_roll = roll;
  robotstate_.euler_pitch = pitch;
  robotstate_.euler_yaw = yaw;

  robotstate_.euler_roll_velocity = msg->angular_velocity.x;
  robotstate_.euler_pitch_velocity = msg->angular_velocity.y;
  robotstate_.euler_yaw_velocity = msg->angular_velocity.z;

  robotstate_.x_acceleration = msg->linear_acceleration.x;
  robotstate_.y_acceleration = msg->linear_acceleration.y;
  robotstate_.z_acceleration = msg->linear_acceleration.z;
}

void LqrEffortController::joint_states_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
  robotstate_.left_wheel_velocity = msg->velocity[0];
  robotstate_.right_wheel_velocity = msg->velocity[1];
  robotstate_.left_hip_angle = msg->position[2];
  robotstate_.right_hip_angle = msg->position[3];
  robotstate_.left_knee_angle = msg->position[4];
  robotstate_.right_knee_angle = msg->position[5];

  robotstate_.left_theta0 =  PI - robotstate_.left_hip_angle;
  robotstate_.right_theta0 =  PI - robotstate_.right_hip_angle;
  robotstate_.left_theta1 = robotstate_.left_theta0 - robotstate_.left_knee_angle;
  robotstate_.right_theta1 = robotstate_.right_theta0 - robotstate_.right_knee_angle;
  // left_wheel_effort_= msg->effort[0];
  // right_wheel_effort_ = msg->effort[1];
}

controller_interface::InterfaceConfiguration LqrEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration LqrEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size());
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn LqrEffortController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LqrEffortController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LqrEffortController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto current_ref = input_ref_.readFromRT();

  L0_PHI0(robotstate_.left_theta0, robotstate_.left_theta1,robotstate_.left_l0,robotstate_.left_phi0);
  L0_PHI0(robotstate_.right_theta0, robotstate_.right_theta1,robotstate_.right_l0,robotstate_.right_phi0);
  VMC_calc(robotstate_.left_F0,robotstate_.left_Tp,robotstate_.left_theta0,robotstate_.left_theta1,robotstate_.left_T0,robotstate_.left_T1);
  VMC_calc(robotstate_.right_F0,robotstate_.right_Tp,robotstate_.right_theta0,robotstate_.right_theta1,robotstate_.right_T0,robotstate_.right_T1);

  double set_effort[6] = {0.0,0.0,1.0,1.0,1.0,1.0};

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    // if (!std::isnan((*current_ref)->displacements[i]))
    // {
    //   if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
    //   {
    //     (*current_ref)->displacements[i] /= 2;
    //   }
      command_interfaces_[i].set_value(set_effort[i]/*(*current_ref)->displacements[i]*/);

    //   (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    // }
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void L0_PHI0(const float theta0, const float theta1, float L0, float phi0) {
   
    // 计算末端坐标
    const float xc = thigh_link_length * std::cos(theta0) + shank_link_length * std::cos(theta1);
    const float yc = thigh_link_length * std::sin(theta0) + shank_link_length * std::sin(theta1);
    
    // 计算末端到原点的距离
    L0 = std::sqrt(xc * xc + yc * yc);
    
    // 计算末端方向角 (范围: [-π, π])
    phi0 = std::atan2(yc, xc);
 
}

void VMC_calc(float F0, float Tp, float theta0, float theta1, float T0, float T1) {

    // 计算角度差值
    const float theta_diff = theta0 - theta1;
    
    // 计算雅可比矩阵的分母公共项
    const float denom1 = thigh_link_length * thigh_link_length + 
                          2 * thigh_link_length * shank_link_length * std::cos(theta_diff) + 
                          shank_link_length * shank_link_length;
    const float denom2 = std::sqrt(denom1);
    
    // 计算雅可比矩阵元素
    const float sin_diff = std::sin(theta_diff);
    const float j11 = -(thigh_link_length * shank_link_length * sin_diff) / denom2;
    const float j12 = -j11;  // 因为j12 = -j11
    const float j21 = (thigh_link_length * 
                       (thigh_link_length + shank_link_length * std::cos(theta_diff))) / denom1;
    const float j22 = (shank_link_length * 
                       (shank_link_length + thigh_link_length * std::cos(theta_diff))) / denom1;
    
    // 计算关节力矩 (τ = JᵀF)
    T0 = j11 * F0 + j21 * Tp;  // 大腿关节力矩
    T1 = j12 * F0 + j22 * Tp;  // 小腿关节力矩

}
    
}  // namespace lqr_effort_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lqr_effort_controller::LqrEffortController, controller_interface::ControllerInterface)
