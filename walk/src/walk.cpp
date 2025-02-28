#include <memory>
#include <utility>

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

  pub_sole_poses_ = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);
  pub_current_twist_ = create_publisher<geometry_msgs::msg::Twist>("walk/current_twist", 1);
  pub_ready_to_step_ = create_publisher<std_msgs::msg::Bool>("walk/ready_to_step", 1);

  pub_gait_ = create_publisher<walk_interfaces::msg::Gait>("walk/gait", 1);
  pub_step_ = create_publisher<walk_interfaces::msg::Step>("walk/step", 1);
}

Walk::~Walk() {}

void Walk::walkControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  walking_enabled_ = msg->data;  // Activate or disable walking 

  if (walking_enabled_) {
    RCLCPP_INFO(get_logger(), "🟢 Walk activated.");
  } else {
    RCLCPP_INFO(get_logger(), "🔴 Walk disabled.");
  }
}

void Walk::generateCommand()
{
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

  pub_current_twist_->publish(curr_twist_);

  std_msgs::msg::Bool ready_to_step;
  ready_to_step.data = step_state_->done();
  pub_ready_to_step_->publish(ready_to_step);
}

void Walk::walk(const geometry_msgs::msg::Twist & commanded_twist)
{
  if (!walking_enabled_) {
    RCLCPP_DEBUG(get_logger(), "🚫 Walk disabled, ignoring movement commands.");
    return;
  }

  RCLCPP_DEBUG(
    get_logger(), "walk() called with commanded_twist:  %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    commanded_twist.linear.x, commanded_twist.linear.y, commanded_twist.linear.z,
    commanded_twist.angular.x, commanded_twist.angular.y, commanded_twist.angular.z);

  target_twist_ = twist_limiter::limit(params_->twist_limiter_, commanded_twist);
}

void Walk::notifyPhase(const biped_interfaces::msg::Phase & phase)
{
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

  ftp_current_ = std::move(ftp_next);
}

void Walk::imuCallback(const sensor_msgs::msg::Imu & imu)
{
  filtered_gyro_y_ = 0.8 * filtered_gyro_y_ + 0.2 * imu.angular_velocity.y;
}

}  // namespace walk

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(walk::Walk)
