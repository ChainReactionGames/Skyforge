#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace external_torque_controller
{

class ExternalTorqueController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void tau_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Params
  std::vector<std::string> joint_names_;
  std::string topic_name_;
  double timeout_sec_{0.05};  // default 50 ms
  std::vector<double> tau_limits_; // per-joint abs limit; size 3

  // Latest command storage
  std::mutex mtx_;
  std::array<double, 3> last_tau_{0.0, 0.0, 0.0};
  rclcpp::Time last_rx_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
  bool have_cmd_{false};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

}  // namespace external_torque_controller
