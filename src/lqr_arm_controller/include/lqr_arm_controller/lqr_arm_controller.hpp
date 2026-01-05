#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "eigen3/Eigen/Dense"

namespace lqr_arm_controller
{
    class LQRArmController : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;
        
        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        controller_interface::CallbackReturn 
        on_init() override;
        controller_interface::CallbackReturn 
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn 
        on_activate(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::return_type
        update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        // LQR gain matrix
        Eigen::Matrix<double, 3, 6> K_;

    };
}