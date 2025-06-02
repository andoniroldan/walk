#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include <cmath>


#include "walk/walk.hpp"
#include "twist_limiter.hpp"
#include "twist_change_limiter.hpp"
#include "maths_functions.hpp"
#include "walk_interfaces/msg/feet_trajectory_point.hpp"
#include "walk_interfaces/msg/step.hpp"
#include "step_state.hpp"
#include "target_gait_calculator.hpp"
#include "sole_pose.hpp"
#include "feet_trajectory.hpp"
#include "params.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace walk
{

Walk::Walk(const rclcpp::NodeOptions & options)
: Node("Walk", options), walking_enabled_(false)  // Initialy, the walking is disabled
{
  params_ = std::make_unique<Params>(*this);

  generate_command_timer_ = create_wall_timer(
    std::chrono::duration<float>(params_->feet_trajectory_.dt_),
    std::bind(&Walk::generateCommand, this));

  sub_phase_ = create_subscription<biped_interfaces::msg::Phase>(
    "phase", 10, std::bind(&Walk::notifyPhase, this, std::placeholders::_1));

  sub_target_ = create_subscription<geometry_msgs::msg::Twist>(
    "target", 10, std::bind(&Walk::walk, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&Walk::imuCallback, this, std::placeholders::_1));

  // New subscriber to enable or disable walking
  sub_walk_control_ = create_subscription<std_msgs::msg::Bool>(
    "/walk_control", 10, std::bind(&Walk::walkControlCallback, this, std::placeholders::_1));

  sub_action_status_ = create_subscription<std_msgs::msg::String>(
    "/nao_pos_action/status", 10, std::bind(&Walk::actionStatusCallback, this, std::placeholders::_1));

  sub_fsr_ = create_subscription<nao_lola_sensor_msgs::msg::FSR>(
    "/sensors/fsr", 10, std::bind(&Walk::fsrCallback, this, std::placeholders::_1));
  
  pub_sole_poses_ = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);
  pub_current_twist_ = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step_ = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  pub_gait_ = create_publisher<walk_interfaces::msg::Gait>("walk/gait", 1);
  pub_step_ = create_publisher<walk_interfaces::msg::Step>("walk/step", 1);

  pub_getup_action_ = create_publisher<std_msgs::msg::String>("action_req_legs", 10);

  pub_arm_positions_ = create_publisher<nao_lola_command_msgs::msg::JointPositions>(
    "/effectors/joint_positions", 10);

  pub_general_stiffness_ = create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
    "/effectors/joint_stiffnesses", 10);
  
  security_fall_ = false;
  RCLCPP_INFO(get_logger(), "Walk node has been started.");

  is_standing_ = false;

  walking_enabled_last_time_ = false;
  
  start_moving_time_ = this->get_clock()->now();
  has_started_moving_ = false;

  fsr_emergency_stop_ = false;
  last_fsr_emergency_stop_ = false;

  accel_x = 0.0;

  current_left_shoulder_pos_ = 1.5;
  current_right_shoulder_pos_ = 1.5;

  target_left_shoulder_pos_ = 1.5;
  target_right_shoulder_pos_ = 1.5;

  delay_completed_ = false;

  // Start a timer for the 20-second delay so the robot can start
  delay_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20000), [this]() {
      delay_completed_ = true;
      RCLCPP_INFO(this->get_logger(), "20-second delay completed. Ready to start walk");
      delay_timer_->cancel(); // Stop the delay timer after it completes
    });
}

Walk::~Walk() {}

void Walk::walkControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  walking_enabled_ = msg->data;  // Activate or disable walking 

  if (walking_enabled_) {
    RCLCPP_INFO(get_logger(), "🟢 Walk activated.");
  } else {
    RCLCPP_INFO(get_logger(), "🔴 Walk disabled.");
    has_started_moving_ = false;
    // Reset the walking state
    restartWalk();
  }
}

void Walk::generateCommand()
{

  if(!delay_completed_){
    return;
  }

  if (fsr_emergency_stop_) {
    RCLCPP_DEBUG(get_logger(), "🚫 FSR emergency stop detected, stoping movement.");
    return;
  }

  if (security_fall_) {
    RCLCPP_DEBUG(get_logger(), "🚫 Security fall detected, stoping movement.");
    return;
  }

  if (!walking_enabled_) {
    RCLCPP_DEBUG(get_logger(), "🚫 Walk desabled, stoping movement.");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "generateCommand()");

  if (!step_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,  // ms
      "No step calculated yet, can't generate command!");
    return;
  }

  if (!step_state_->done()) {
    RCLCPP_DEBUG(get_logger(), "sending sole poses");
    pub_sole_poses_->publish(
      sole_pose::generate(params_->sole_pose_, step_state_->next(), phase_, filtered_gyro_y_));
  }

  updateArms();

  pub_current_twist_->publish(curr_twist_);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = step_state_->done();
  pub_ready_to_step_->publish(ready_to_step);
}

void Walk::walk(const geometry_msgs::msg::Twist & commanded_twist)
{

    RCLCPP_DEBUG(get_logger(), "arm_min_twist_to_activate_: %.3f", params_->arm_min_twist_to_activate_);


  if(!delay_completed_){
    return;
  }

  if(fsr_emergency_stop_) {
    RCLCPP_DEBUG(get_logger(), "🚫 FSR emergency stop detected, ignoring movement commands.");
    return;
  }

  if (!walking_enabled_) {
    RCLCPP_DEBUG(get_logger(), "🚫 Walk disabled, ignoring movement commands.");
    return;
  }

  // Calculate the time since the robot started moving
  rclcpp::Time now = this->get_clock()->now();
  double elapsed_time = (now - start_moving_time_).seconds();

  geometry_msgs::msg::Twist adjusted_twist = commanded_twist;

  if (has_started_moving_ && elapsed_time < IGNORE_TWIST_VELOCITY_TIME) {
    // If the robot has started moving, but less than 2 seconds have passed,
    // ignore the twist messages and keep the robot still
    RCLCPP_INFO(get_logger(), "⏳ Less than 2 seconds have passed since the robot started moving. Ignoring twist messages.");
    adjusted_twist.linear.x = 0.0;
    adjusted_twist.linear.y = 0.0;
    adjusted_twist.linear.z = 0.0;
    adjusted_twist.angular.x = 0.0;
    adjusted_twist.angular.y = 0.0;
    adjusted_twist.angular.z = 0.0;
  }

  RCLCPP_DEBUG(
    get_logger(), "walk() called with commanded_twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    commanded_twist.linear.x, commanded_twist.linear.y, commanded_twist.linear.z,
    commanded_twist.angular.x, commanded_twist.angular.y, commanded_twist.angular.z);

  RCLCPP_DEBUG(
    get_logger(), "walk() adjusted twist: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    adjusted_twist.linear.x, adjusted_twist.linear.y, adjusted_twist.linear.z,
    adjusted_twist.angular.x, adjusted_twist.angular.y, adjusted_twist.angular.z);

  target_twist_ = twist_limiter::limit(params_->twist_limiter_, adjusted_twist);
}

void Walk::notifyPhase(const biped_interfaces::msg::Phase & phase)
{
  if(!delay_completed_){
    return;
  }

  if (fsr_emergency_stop_) {
    RCLCPP_DEBUG(get_logger(), "🚫 FSR emergency stop detected, ignoring fase changes.");
    return;
  }

  if (!walking_enabled_) {
    RCLCPP_DEBUG(get_logger(), "🚫 Walk disabled, ignoring fase changes.");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "notifyPhase called");

  if (phase.phase == phase_.phase) {
    RCLCPP_DEBUG(get_logger(), "Notified of a phase, but no change has taken place. Ignoring.");
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Calculating new step!");

  // If the robot has started moving, log the time so when 2 seconds have passed
  // it can actually follow the twist messages
  if (!has_started_moving_) {
    has_started_moving_ = true;
    start_moving_time_ = this->get_clock()->now();
    RCLCPP_INFO(get_logger(), "☑️ The robot has started moving. Starting zero speed timer.");
  }

  phase_ = phase;

  curr_twist_ = twist_change_limiter::limit(
    params_->twist_change_limiter_, curr_twist_, target_twist_);

  auto gait = target_gait_calculator::calculate(curr_twist_, params_->target_gait_calculator_);
  pub_gait_->publish(gait);

  auto ftp_next = walk_interfaces::msg::FeetTrajectoryPoint(
    (phase.phase == phase.LEFT_STANCE) ? gait.left_stance_phase_aim : gait.right_stance_phase_aim);

  RCLCPP_DEBUG(
    get_logger(), "Using %s",
    (phase.phase == phase.LEFT_STANCE) ? "LSP (Left Stance Phase)" : "RSP (Right Stance Phase)");

  step_ = std::make_unique<walk_interfaces::msg::Step>(
    feet_trajectory::generate(
      params_->feet_trajectory_, phase, ftp_current_, ftp_next));
  step_state_ = std::make_unique<StepState>(*step_);
  pub_step_->publish(*step_);


  // print param arm_min_twist_to_activate_
  RCLCPP_INFO(get_logger(), "arm_min_twist_to_activate_: %.3f", params_->arm_min_twist_to_activate_);

  if (curr_twist_.linear.x > params_->arm_min_twist_to_activate_) {

    RCLCPP_INFO(get_logger(), "Activating arms 💪");
    double amplitude = params_->arm_swing_amplitude_;
    double base_position_left = params_->arm_base_position_left_;
    double base_position_right = params_->arm_base_position_right_;

    if (phase.phase == biped_interfaces::msg::Phase::RIGHT_SWING) {
      target_left_shoulder_pos_ = base_position_left - amplitude;   // left arm forward
      target_right_shoulder_pos_ = base_position_right + amplitude;  // right arm backward
    } else if (phase.phase == biped_interfaces::msg::Phase::LEFT_SWING) {
      target_left_shoulder_pos_ = base_position_left + amplitude;   // left arm backward
      target_right_shoulder_pos_ = base_position_right - amplitude;  // right arm forward
    }
  } else {
    target_left_shoulder_pos_ = params_->arm_base_position_left_;;  // arms down
    target_right_shoulder_pos_ = params_->arm_base_position_right_;;
  }

  ftp_current_ = std::move(ftp_next);
}

void Walk::imuCallback(const sensor_msgs::msg::Imu & imu)
{

  if(!delay_completed_){
    return;
  }
  // Thresholds for detecting a fall
  const double FALL_Z_THRESHOLD = -5.0;       // Detects if the robot is lying down
  const double RECOVERY_Z_THRESHOLD = -9.0;  // Close to -9.8 when standing
  const double FALL_TIME_THRESHOLD = 5.0;    // Time in seconds before confirming the fall
  const double FALL_FREQUENCY_THRESHOLD = 8; // Number of detected possible falls in short time
  const double FALL_FREQUENCY_TIME_WINDOW = 1.0; // Time window in seconds to count frequent falls

  static bool fallen = false;
  static bool counting_fall_time = false;
  static std::vector<rclcpp::Time> fall_timestamps; // Stores timestamps of recent falls
  static rclcpp::Time fall_start_time; // Start time when fall is detected

  accel_x = imu.linear_acceleration.x;
  double accel_z = imu.linear_acceleration.z;
  
  rclcpp::Time now = this->get_clock()->now();

  // Remove timestamps older than FALL_FREQUENCY_TIME_WINDOW
  fall_timestamps.erase(
    std::remove_if(fall_timestamps.begin(), fall_timestamps.end(),
                   [now, FALL_FREQUENCY_TIME_WINDOW](rclcpp::Time t) { 
                       return (now - t).seconds() > FALL_FREQUENCY_TIME_WINDOW; 
                   }),
    fall_timestamps.end());

  // If acceleration indicates a fall, start or continue counting time
  if (accel_z > FALL_Z_THRESHOLD) {  
    if (!counting_fall_time) {  
      fall_start_time = now;
      counting_fall_time = true;  
      fall_timestamps.push_back(now); // Log this fall attempt
      RCLCPP_WARN(get_logger(), "⚠️ Possible fall detected, starting timer...");
    } 
    else if ((now - fall_start_time).seconds() >= FALL_TIME_THRESHOLD && !fallen) {  
      fallen = true;
      // Set last walking state to resume after recovery
      if (!security_fall_){
        walking_enabled_last_time_ = walking_enabled_;
        RCLCPP_WARN(get_logger(), "🔍 walking_enabled_ before fall: %s", walking_enabled_ ? "true" : "false");
      }
      security_fall_ = true;
      
      RCLCPP_ERROR(get_logger(), "❌ Fall confirmed! Stopping movement.");

      // Stop walking
      walking_enabled_ = false;
      has_started_moving_ = false;

      // Reset walk node to initial state
      restartWalk();

      // Check if swing was actually running
      RCLCPP_INFO(get_logger(), "Sending get-up command immediately.");
      sendGetupCommand();
    }
  } 
  else {  
    // If acceleration goes back to normal, stop counting fall time
    if (counting_fall_time) {  
      counting_fall_time = false;  
      RCLCPP_INFO(get_logger(), "✅ False alarm! Acceleration normalized.");
    }
  }
  if (fall_timestamps.size() >= FALL_FREQUENCY_THRESHOLD  && !security_fall_) {
    // If too many falls detected in a short period (probably nao is having spams
    // because is trying to walk lying on the floor), force a fall
    fallen = true;

    // Set last walking state to resume after recovery
    if (!security_fall_){
      walking_enabled_last_time_ = walking_enabled_;
      RCLCPP_WARN(get_logger(), "🔍 walking_enabled_ before fall: %s", walking_enabled_ ? "true" : "false");
    }
    security_fall_ = true;

    RCLCPP_ERROR(get_logger(), "🚨 Too many falls detected in short time! Triggering fall recovery.");
    
    // Set last walking state to resume after recovery
    walking_enabled_last_time_ = walking_enabled_;
    RCLCPP_WARN(get_logger(), "🔍 walking_enabled_ before fall: %s", walking_enabled_ ? "true" : "false");


    // Stop walking
    walking_enabled_ = false;
    has_started_moving_ = false;

    // Reset walk node to initial state
    restartWalk();
    
    // Send get-up command immediately
    sendGetupCommand();
    
    // Clear the fall timestamps to reset the detection window
    fall_timestamps.clear();
  }

  // If the robot returns to a stable position, reset the fall detection
  if (fallen && accel_z < RECOVERY_Z_THRESHOLD) {  
    fallen = false;
    counting_fall_time = false;
    fall_start_time = rclcpp::Time();  
    RCLCPP_INFO(get_logger(), "✅ Robot has recovered stability, resuming movement.");
  }
}

void Walk::actionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(!delay_completed_){
    return;
  }
  RCLCPP_INFO(get_logger(), "Received action status: %s", msg->data.c_str());

  if (security_fall_) {
    if (msg->data.find("succeeded") != std::string::npos) {
      if (msg->data.find("only_legs_fast") != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Swing completed...");

        // Now that swing is done, send get-up command
        sendGetupCommand();
      } 
      else if (msg->data.find("stand") != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Stand completed...");
        is_standing_ = true;

        // Now that the robot is standing, send get-up command
        // Is less probable that is trying to make a stand on the floor
        // but not impossible
        sendGetupCommand();
      }

      // Resume walking if walking was enabled
      if (walking_enabled_last_time_) {
        RCLCPP_INFO(get_logger(), "✅ Recovery complete, walking can resume.");
        
        walking_enabled_ = true;
        has_started_moving_ = false;

        // Publis stifness to 1 in all joints
        nao_lola_command_msgs::msg::JointStiffnesses msg_stiffness;
        msg_stiffness.indexes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                                  12, 13, 14, 15, 16, 17, 18};
        msg_stiffness.stiffnesses = {1.0, 1.0, 1.0, 1.0, 1.0,
                                      1.0, 1.0, 1.0, 1.0, 1.0,
                                      1.0, 1.0, 1.0, 1.0, 1.0,
                                      1.0, 1.0, 1.0, 1.0};

        pub_general_stiffness_->publish(msg_stiffness);



      } else{
        RCLCPP_INFO(get_logger(), "✅ Recovery complete, walking is disabled.");
      }

      // Recovery confirmed, allow walking again
      security_fall_ = false;
    }
  }
}

void Walk::sendGetupCommand()
{
  const double FALL_X_THRESHOLD = 2.0;
  
  // Determine whether the fall was forward or backward
  std::string action = (accel_x > FALL_X_THRESHOLD) ? "getupFront" : "getupBack";
  RCLCPP_INFO(get_logger(), "Executing get-up action: %s", action.c_str());

  // Publish the action to get up
  std_msgs::msg::String getup_action_msg;
  getup_action_msg.data = action;
  pub_getup_action_->publish(getup_action_msg);
}

/* Restart the walk as if it was the first time */
void Walk::restartWalk()
{
  RCLCPP_WARN(get_logger(), "🔄 Resetting Walk node to initial state...");

  // Reset phase and trajectory
  phase_ = biped_interfaces::msg::Phase();
  curr_twist_ = geometry_msgs::msg::Twist();
  target_twist_ = geometry_msgs::msg::Twist();
  
  // Reset current step
  step_.reset();
  step_state_.reset();

  ftp_current_ = walk_interfaces::msg::FeetTrajectoryPoint();

  RCLCPP_INFO(get_logger(), "✅ Walk node state has been reset.");
}

void Walk::fsrCallback(const nao_lola_sensor_msgs::msg::FSR::SharedPtr msg)
{
  if(!delay_completed_){
    return;
  }
  
  std::vector<double> values = {
    msg->l_foot_front_left, msg->l_foot_front_right,
    msg->l_foot_back_left, msg->l_foot_back_right,
    msg->r_foot_front_left, msg->r_foot_front_right,
    msg->r_foot_back_left, msg->r_foot_back_right
  };

  bool all_low = std::all_of(values.begin(), values.end(), [](double val) {
    return val < 0.2;
  });

  if (all_low) {
    if (!last_fsr_emergency_stop_) {
      RCLCPP_WARN(get_logger(), "🟥 All FSR sensors below threshold") ;
    }
    last_fsr_emergency_stop_ = true;
    fsr_emergency_stop_ = true;
  } 
  else if (!all_low) {
    if (last_fsr_emergency_stop_) {
      RCLCPP_INFO(get_logger(), "🟩 Pressure detected again");
    }
    last_fsr_emergency_stop_ = false;
    fsr_emergency_stop_ = false;
  }
}

void Walk::updateArms()
{
  double step_size = params_->arm_step_size_;
  current_left_shoulder_pos_ = stepTowards(current_left_shoulder_pos_, target_left_shoulder_pos_, step_size);
  current_right_shoulder_pos_ = stepTowards(current_right_shoulder_pos_, target_right_shoulder_pos_, step_size);

  nao_lola_command_msgs::msg::JointPositions msg;
  msg.indexes = {2, 18, 3, 19, 4, 5, 6, 20, 21, 22};
  msg.positions = {current_left_shoulder_pos_, current_right_shoulder_pos_, 0.2, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  pub_arm_positions_->publish(msg);
}

double Walk::stepTowards(double current, double target, double step)
{
  if (std::fabs(target - current) < step) {
    return target;
  }
  return current + (target > current ? step : -step);
}


}  // namespace walk

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk::Walk)
