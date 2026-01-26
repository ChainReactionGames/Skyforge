#include "lqr_arm_controller/lqr_arm_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"   // 🔴 ADDED: debug message
#include <pluginlib/class_list_macros.hpp>

namespace lqr_arm_controller
{

/* =========================
 * Initialization
 * ========================= */
controller_interface::CallbackReturn
LQRArmController::on_init()
{
  // UNCHANGED
  auto_declare<std::vector<double>>("K", std::vector<double>(18, 0.0));
  return controller_interface::CallbackReturn::SUCCESS;
}

/* =========================
 * Interface configuration
 * ========================= */
controller_interface::InterfaceConfiguration
LQRArmController::command_interface_configuration() const
{
  // UNCHANGED
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joint1/effort", "joint2/effort", "joint3/effort"}
  };
}

controller_interface::InterfaceConfiguration
LQRArmController::state_interface_configuration() const
{
  // UNCHANGED
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "joint1/position", "joint1/velocity", "joint1/effort",
      "joint2/position", "joint2/velocity", "joint2/effort",
      "joint3/position", "joint3/velocity", "joint3/effort"
    }
  };
}

/* =========================
 * Activation
 * ========================= */
controller_interface::CallbackReturn
LQRArmController::on_activate(const rclcpp_lifecycle::State &)
{
  // UNCHANGED safety checks
  if (command_interfaces_.size() != 3 || state_interfaces_.size() != 9)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Interface count mismatch (cmd=%zu, state=%zu)",
      command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 🔴 ADDED: always-on debug publisher (NO lazy publishing)
  torque_pub_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

/* =========================
 * Configuration
 * ========================= */
controller_interface::CallbackReturn
LQRArmController::on_configure(const rclcpp_lifecycle::State &)
{
  // UNCHANGED
  auto K_vec = get_node()->get_parameter("K").as_double_array();

  if (K_vec.size() != 18)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "K must have exactly 18 elements (3x6). Got %zu", K_vec.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // UNCHANGED mapping
  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 6; ++j)
      K_(i, j) = K_vec[i * 6 + j];

  RCLCPP_INFO(get_node()->get_logger(), "LQR gain matrix loaded");
  return controller_interface::CallbackReturn::SUCCESS;
}

/* =========================
 * Update loop
 * ========================= */
controller_interface::return_type
LQRArmController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // UNCHANGED state reader
  auto read_state = [&](size_t idx) -> double {
    auto val = state_interfaces_[idx].get_optional();
    if (!val) throw std::runtime_error("State interface unavailable");
    return *val;
  };

  Eigen::Matrix<double, 6, 1> x;

  try
  {
    // UNCHANGED state construction
    x << read_state(0) - 1.07079632679,
         read_state(3) - 1.0,
         read_state(6) + 0.5,
         read_state(1),
         read_state(4),
         read_state(7);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::return_type::ERROR;
  }

  // UNCHANGED control law
  Eigen::Matrix<double, 3, 1> tau = -K_ * x;

  // UNCHANGED command interface writes
  command_interfaces_[0].set_value(tau(0));
  command_interfaces_[1].set_value(tau(1));
  command_interfaces_[2].set_value(tau(2));

  // 🔴 ADDED: explicit torque publication (NO echo-triggered behavior)
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {tau(0), tau(1), tau(2)};
  torque_pub_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace lqr_arm_controller

PLUGINLIB_EXPORT_CLASS(
  lqr_arm_controller::LQRArmController,
  controller_interface::ControllerInterface)
