#include "external_torque_controller/external_torque_controller.hpp"

#include <algorithm>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace external_torque_controller
{

controller_interface::CallbackReturn ExternalTorqueController::on_init()
{
  // Defaults match your LQR controller’s interface names
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3"});
  auto_declare<std::string>("topic", "/tau_cmd");
  auto_declare<double>("timeout_sec", 0.05);

  // Per-joint absolute torque limits. Keep conservative at first.
  auto_declare<std::vector<double>>("tau_limits", {50.0, 50.0, 50.0});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ExternalTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Map joints -> "<joint>/effort"
  cfg.names.reserve(3);
  for (const auto & j : joint_names_) {
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
ExternalTorqueController::state_interface_configuration() const
{
  // We don't *need* state interfaces to apply external torques.
  // Return NONE so ros2_control doesn't try to claim any.
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::NONE;
  return cfg;
}

controller_interface::CallbackReturn
ExternalTorqueController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  topic_name_  = get_node()->get_parameter("topic").as_string();
  timeout_sec_ = get_node()->get_parameter("timeout_sec").as_double();
  tau_limits_  = get_node()->get_parameter("tau_limits").as_double_array();

  if (joint_names_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected exactly 3 joints, got %zu", joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (tau_limits_.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected tau_limits size 3, got %zu", tau_limits_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "timeout_sec must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  // QoS: for control commands, keep depth small. Best effort avoids blocking.
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  qos.durability_volatile();

  sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_name_, qos,
    std::bind(&ExternalTorqueController::tau_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "ExternalTorqueController configured. Topic: %s, timeout: %.3f s",
    topic_name_.c_str(), timeout_sec_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ExternalTorqueController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 3 command interfaces, got %zu", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // On activation, command zero torque for safety
  for (auto & ci : command_interfaces_) {
    bool zeroComplete = ci.set_value(0.0);
    if (!zeroComplete) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set zero torque on activate");
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_tau_ = {0.0, 0.0, 0.0};
    have_cmd_ = false;
    last_rx_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  }

  RCLCPP_INFO(get_node()->get_logger(), "ExternalTorqueController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ExternalTorqueController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Command zero torque on deactivate
  for (auto & ci : command_interfaces_) {
    bool zeroComplete = ci.set_value(0.0);
    if (!zeroComplete) {
        RCLCPP_WARN(get_node()->get_logger(), "Failed to set zero torque on deactivate");
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "ExternalTorqueController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

void ExternalTorqueController::tau_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) return;
  if (msg->data.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Received tau_cmd with data.size()=%zu (<3). Ignoring.", msg->data.size());
    return;
  }

  std::array<double, 3> tau;
  tau[0] = msg->data[0];
  tau[1] = msg->data[1];
  tau[2] = msg->data[2];

  // Apply saturation here (so update loop is just a fast write)
  for (size_t i = 0; i < 3; ++i) {
    const double lim = std::abs(tau_limits_[i]);
    tau[i] = std::clamp(tau[i], -lim, lim);
  }

  std::lock_guard<std::mutex> lock(mtx_);
  last_tau_ = tau;
  last_rx_time_ = get_node()->get_clock()->now();
  have_cmd_ = true;
}

controller_interface::return_type
ExternalTorqueController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, 3> tau_to_apply{0.0, 0.0, 0.0};
  bool valid = false;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (have_cmd_) {
      const rclcpp::Time now = get_node()->get_clock()->now();
      const double age = (now - last_rx_time_).seconds();
      if (age <= timeout_sec_) {
        tau_to_apply = last_tau_;
        valid = true;
      }
    }
  }

  if (!valid) {
    // Stale or missing command -> zero torque (safe default)
    tau_to_apply = {0.0, 0.0, 0.0};
  }

  // Write to effort command interfaces
  bool ok0 = command_interfaces_[0].set_value(tau_to_apply[0]);
  bool ok1 = command_interfaces_[1].set_value(tau_to_apply[1]);
  bool ok2 = command_interfaces_[2].set_value(tau_to_apply[2]);

  if (!(ok0 && ok1 && ok2)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set one or more effort commands");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace external_torque_controller

PLUGINLIB_EXPORT_CLASS(
  external_torque_controller::ExternalTorqueController,
  controller_interface::ControllerInterface)
