#include "lqr_arm_controller/lqr_arm_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace lqr_arm_controller
{

    controller_interface::CallbackReturn
    LQRArmController::on_init()
    {
        // Declare initialization parameters (K matrix for the LQR controller's gain)
        auto_declare_parameters<std::vector<double>>(
            "K", std::vector<double>{});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // command_interfaces_ vector: 0 - joint1/effort, 1 - joint2/effort, 2 - joint3/effort
    controller_interface::InterfaceConfiguration
    LQRArmController::command_interface_configuration() const
    {
        // Define command interfaces for 3 joints - the joint names MUST match those in the URDF
        return controller_interface::InterfaceConfiguration {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"joint1/effort", "joint2/effort", "joint3/effort"}
        }
    }

    // state_interfaces_ vector: 0 - joint1/position, 1 - joint1/velocity,
    //                           2 - joint2/position, 3 - joint2/velocity,
    //                           4 - joint3/position, 5 - joint3/velocity
    controller_interface::InterfaceConfiguration
    LQRArmController::state_interface_configuration() const
    {
        // Define state interfaces for 3 joints (position and velocity)
        return controller_interface::InterfaceConfiguration {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {
                "joint1/position", "joint1/velocity",
                "joint2/position", "joint2/velocity",
                "joint3/position", "joint3/velocity"
            }
        };
    }

    // Activation safety checks
    controller_interface::CallbackReturn
    LQRArmController::on_activate(const rclcpp_lifecycle::State & )
    {
        if (command_interfaces_.size() != 3)
        {
            RCLCPP_ERROR(
                get_logger(),
                "LQRArmController requires 3 command interfaces (effort for 3 joints).");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (state_interfaces_.size() != 6)
        {
            RCLCPP_ERROR(
                get_logger(),
                "LQRArmController requires 6 state interfaces (position and velocity for 3 joints).");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }
}