#include "lqr_arm_controller/lqr_arm_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace lqr_arm_controller
{
    /// MARK: - Initialization
    controller_interface::CallbackReturn
    LQRArmController::on_init()
    {
        // Declare initialization parameters (K matrix for the LQR controller's gain)
        // Expecting a flat vector of 18 elements (3x6 matrix)
        auto_declare<std::vector<double>>("K", std::vector<double>(18, 0.0));
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /// MARK: - Configuring command/state interfaces

    // command_interfaces_ vector: 0 - joint1/effort, 1 - joint2/effort, 2 - joint3/effort
    controller_interface::InterfaceConfiguration
    LQRArmController::command_interface_configuration() const
    {
        // Define command interfaces for 3 joints - the joint names MUST match those in the URDF
        return controller_interface::InterfaceConfiguration {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"joint1/effort", "joint2/effort", "joint3/effort"}
        };
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
                get_node()->get_logger(),
                "LQRArmController requires 3 command interfaces (effort for 3 joints).");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (state_interfaces_.size() != 6)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "LQRArmController requires 6 state interfaces (position and velocity for 3 joints).");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /// MARK: - Load LQR gain matrix
    controller_interface::CallbackReturn
    LQRArmController::on_configure(const rclcpp_lifecycle::State & )
    {
        // Retrieve and set the LQR gain matrix K from the given 18 element array parameter
        auto K_vec = get_node()->get_parameter("K").as_double_array();
        if (K_vec.size() != 18)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "LQR gain matrix K must have exactly 18 elements (3x6 matrix). Got %zu", K_vec.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        // Map flat vector to Eigen matrix
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 6; ++j)
            {
                K_(i, j) = K_vec[i * 6 + j];
            }
        }
        RCLCPP_INFO(get_node()->get_logger(), "LQR gain matrix K configured successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    LQRArmController::update(
        const rclcpp::Time &,
        const rclcpp::Duration & )
    {
        
        auto read_state = [&](size_t idx) -> double {
            auto val = state_interfaces_[idx].get_optional();
            if (!val) {
                throw std::runtime_error("State interface unavailable");
            }
            return *val;
        };

        // Construct state vector x = [q1, q2, q3, dq1, dq2, dq3]^T
        Eigen::Matrix<double, 6, 1> x;
        try {
            x(0) = read_state(0); // joint1 pos
            x(1) = read_state(2); // joint2 pos
            x(2) = read_state(4); // joint3 pos
            x(3) = read_state(1); // joint1 vel
            x(4) = read_state(3); // joint2 vel
            x(5) = read_state(5); // joint3 vel
            RCLCPP_INFO(get_node()->get_logger(), "States: %f %f, %f %f, %f %f", x(0), x(1), x(2), x(3), x(4), x(5));

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
            return controller_interface::return_type::ERROR;
        }

        //DEPRECATED WAY OF DOING THE SAME THING:
        // Eigen::Matrix<double, 6, 1> x;
        // x(0) = state_interfaces_[0].get_value(); // joint1 position
        // x(1) = state_interfaces_[2].get_value(); // joint2 position
        // x(2) = state_interfaces_[4].get_value(); // joint3 position
        // x(3) = state_interfaces_[1].get_value(); // joint1 velocity
        // x(4) = state_interfaces_[3].get_value(); // joint2 velocity
        // x(5) = state_interfaces_[5].get_value(); // joint3 velocity

        // Compute control input tau = -K * x
        Eigen::Matrix<double, 3, 1> tau = -K_ * x;
        //tau(0) = 1; // For testing purposes, set joint1 torque to 1
        RCLCPP_INFO(get_node()->get_logger(), "Computed torques: %f %f %f", tau(0), tau(1), tau(2));   
        // Apply control inputs to command interfaces

        // NEW SUPPOSEDLY SAFE WAY THAT DOESN'T SEEM TO WORK
        // auto set_command = [&](size_t idx, double value) -> bool {
        //     auto opt = command_interfaces_[idx].get_optional();
        //     if (!opt) {
        //         throw std::runtime_error("Command interface unavailable");
        //     }
        //     *opt = value;
        //     return true;
        // };
        // try {
        //     set_command(0, tau(0)); // joint1 effort
        //     set_command(1, tau(1)); // joint2 effort
        //     set_command(2, tau(2)); // joint3 effort
        // } catch (const std::exception & e) {
        //     RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
        //     return controller_interface::return_type::ERROR;
        // }

        //DEPRECATED WAY OF DOING THE SAME THING:
        bool ok0 = command_interfaces_[0].set_value(tau(0)); // joint1 effort
        bool ok1 = command_interfaces_[1].set_value(tau(1)); // joint2 effort
        bool ok2 = command_interfaces_[2].set_value(tau(2)); // joint3 effort
        if (!(ok0 && ok1 && ok2)) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to set one or more joint effort commands"
            );
        }
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lqr_arm_controller::LQRArmController, controller_interface::ControllerInterface)
