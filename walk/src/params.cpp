// Copyright 2023 Kenji Brameld
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

#include <vector>
#include "params.hpp"

namespace walk
{

using rclcpp::ParameterValue;

Params::Params(rclcpp::Node & node)
: node_{node}
{
  double max_forward = node_.declare_parameter("max_forward", 0.1);
  double max_left = node_.declare_parameter("max_left", 0.05);
  double max_turn = node_.declare_parameter("max_turn", 0.5);
  double speed_multiplier = node_.declare_parameter("speed_multiplier", 0.8);
  double foot_lift_amp = node_.declare_parameter("foot_lift_amp", 0.002);
  double period = node_.declare_parameter("period", 0.3);
  double dt = node_.declare_parameter("dt", 0.01);
  double sole_x = node_.declare_parameter("sole_x", -0.022);
  double sole_y = node_.declare_parameter("sole_y", 0.05);
  double sole_z = node_.declare_parameter("sole_z", -0.315);
  double max_forward_change = node_.declare_parameter("max_forward_change", 0.04);
  double max_left_change = node_.declare_parameter("max_left_change", 0.04);
  double max_turn_change = node_.declare_parameter("max_turn_change", 0.6);
  double footh_forward_multiplier = node_.declare_parameter("footh_forward_multiplier", 0.1);
  double footh_left_multiplier = node_.declare_parameter("footh_left_multiplier", 0.15);

  double arm_base_position_left = node_.declare_parameter("arm_base_position_left", 1.7);
  double arm_base_position_right = node_.declare_parameter("arm_base_position_right", 1.7);
  double arm_swing_amplitude = node_.declare_parameter("arm_swing_amplitude", 0.1);
  double arm_step_size = node_.declare_parameter("arm_step_size", 0.015);
  double arm_min_twist_to_activate = node_.declare_parameter("arm_min_twist_to_activate", 0.05);

  
  RCLCPP_DEBUG(logger, "Parameters: ");
  RCLCPP_DEBUG(logger, "  max_forward : %f", max_forward);
  RCLCPP_DEBUG(logger, "  max_left : %f", max_left);
  RCLCPP_DEBUG(logger, "  max_turn : %f", max_turn);
  RCLCPP_DEBUG(logger, "  speed_multiplier : %f", speed_multiplier);
  RCLCPP_DEBUG(logger, "  foot_lift_amp : %f", foot_lift_amp);
  RCLCPP_DEBUG(logger, "  period : %f", period);
  RCLCPP_DEBUG(logger, "  dt : %f", dt);
  RCLCPP_DEBUG(logger, "  sole_x : %f", sole_x);
  RCLCPP_DEBUG(logger, "  sole_y : %f", sole_y);
  RCLCPP_DEBUG(logger, "  sole_z : %f", sole_z);
  RCLCPP_DEBUG(logger, "  max_forward_change : %f", max_forward_change);
  RCLCPP_DEBUG(logger, "  max_left_change : %f", max_left_change);
  RCLCPP_DEBUG(logger, "  max_turn_change : %f", max_turn_change);
  RCLCPP_DEBUG(logger, "  footh_forward_multiplier : %f", footh_forward_multiplier);
  RCLCPP_DEBUG(logger, "  footh_left_multiplier : %f", footh_left_multiplier);
  RCLCPP_DEBUG(logger, "  arm_base_position_left : %f", arm_base_position_left_);
  RCLCPP_DEBUG(logger, "  arm_base_position_right : %f", arm_base_position_right_);
  RCLCPP_DEBUG(logger, "  arm_swing_amplitude : %f", arm_swing_amplitude_);
  RCLCPP_DEBUG(logger, "  arm_step_size : %f", arm_step_size_);
  RCLCPP_DEBUG(logger, "  arm_min_twist_to_activate : %f", arm_min_twist_to_activate_);

  feet_trajectory_ = feet_trajectory::Params(
    foot_lift_amp, period, dt, footh_forward_multiplier, footh_left_multiplier);
  sole_pose_ = sole_pose::Params(sole_x, sole_y, sole_z);
  target_gait_calculator_ = target_gait_calculator::Params(period);
  twist_change_limiter_ = twist_change_limiter::Params(
    max_forward_change, max_left_change, max_turn_change);
  twist_limiter_ = twist_limiter::Params(
    max_forward, max_left, max_turn, speed_multiplier);

  arm_base_position_left_ = arm_base_position_left;
  arm_base_position_right_ = arm_base_position_right;
  arm_swing_amplitude_ = arm_swing_amplitude;
  arm_step_size_ = arm_step_size;
  arm_min_twist_to_activate_ = arm_min_twist_to_activate;


  // Register parameter change callback
  on_set_parameters_callback_handle_ = node_.add_on_set_parameters_callback(
    std::bind(&Params::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Params::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(logger, "Parameter updated:");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & param : parameters) {
    RCLCPP_DEBUG_STREAM(
      logger,
      "- " << param.get_name() << " (" << param.get_type_name() << ")" << " = " <<
        param.value_to_string());

    auto name = param.get_name();
    if (name == "max_forward") {
      twist_limiter_.max_forward_ = param.as_double();
    } else if (name == "max_left") {
      twist_limiter_.max_left_ = param.as_double();
    } else if (name == "max_turn") {
      twist_limiter_.max_turn_ = param.as_double();
    } else if (name == "speed_multiplier") {
      twist_limiter_.speed_multiplier_ = param.as_double();
    } else if (name == "foot_lift_amp") {
      feet_trajectory_.foot_lift_amp_ = param.as_double();
    } else if (name == "period") {
      feet_trajectory_.period_ = param.as_double();
    } else if (name == "dt") {
      feet_trajectory_.dt_ = param.as_double();
    } else if (name == "sole_x") {
      sole_pose_.sole_x_ = param.as_double();
    } else if (name == "sole_y") {
      sole_pose_.sole_y_ = param.as_double();
    } else if (name == "sole_z") {
      sole_pose_.sole_z_ = param.as_double();
    } else if (name == "max_forward_change") {
      twist_change_limiter_.max_forward_change_ = param.as_double();
    } else if (name == "max_left_change") {
      twist_change_limiter_.max_left_change_ = param.as_double();
    } else if (name == "max_turn_change") {
      twist_change_limiter_.max_turn_change_ = param.as_double();
    } else if (name == "footh_forward_multiplier") {
      feet_trajectory_.footh_forward_multiplier_ = param.as_double();
    } else if (name == "footh_left_multiplier") {
      feet_trajectory_.footh_left_multiplier_ = param.as_double();
    } else if (name == "arm_base_position_left") {
      arm_base_position_left_ = param.as_double();
    } else if (name == "arm_base_position_right") {
      arm_base_position_right_ = param.as_double();
    } else if (name == "arm_swing_amplitude") {
      arm_swing_amplitude_ = param.as_double();
    } else if (name == "arm_step_size") {
      arm_step_size_ = param.as_double();
    } else if (name == "arm_min_twist_to_activate") {
      arm_min_twist_to_activate_ = param.as_double();
    }
    
  }

  return result;
}


}  // namespace walk
