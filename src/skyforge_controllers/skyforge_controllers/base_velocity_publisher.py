#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float64MultiArray

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/base_velocity_controller/commands',
            10
        )

        # Acceleration parameters
        self.amax = 0.0625    # maximum acceleration
        self.vmax = 0.1       # maximum velocity
        self.T = 30.0         # total period
        self.L = 2.0          # total distance to travel in one period

        self.ta = self.vmax / self.amax                        # acceleration time
        self.tc = (self.L - self.amax * self.ta**2)/self.vmax  # constant velocity (coast) time


        self.stopping = False   # whether to stop after some time
        self.stop_time = 90.0  # time to stop the motion if self.stopping is True


        # Velocity publishing parameters

        self.dt = 0.01              # time step for velocity points (100 Hz matches controller update rate)

        # self.declare_parameter('use_sim_time', True)

        self.get_logger().info("Waiting for clock to start")
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Clock started, starting trajectory publisher")

        # Wait for controller to be active
        # self.wait_for_controller_active('lqr_arm_controller')
        self.wait_for_controller_active('base_velocity_controller')

        self.timer = self.create_timer(self.dt, self.tick)
        
        # State variables
        self.t0 = self.get_clock().now().nanoseconds / 1e9 # time at start
        self.v = 0.0       # current velocity

        self.get_logger().info("Velocity Publisher Node Started (aM based)")

    def wait_for_controller_active(self, controller_name: str, timeout: float = 30.0) -> bool:
        client = self.create_client(ListControllers, 'controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Service controller_manager/list_controllers not available')
            return False

        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            req = ListControllers.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                response = future.result()
                for controller in response.controller:
                    if controller.name == controller_name and controller.state == 'active':
                        self.get_logger().info(f'Controller {controller_name} is active')
                        return True
            else:
                self.get_logger().error('Failed to call service controller_manager/list_controllers')

            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - start_time > timeout:
                self.get_logger().error(f'Timeout waiting for controller {controller_name} to become active')
                return False

            # self.get_logger().info(f'Waiting for controller {controller_name} to become active...')
            rclpy.spin_once(self, timeout_sec=1.0)

        return False
    

    def aM(self, t: float) -> float:
        if self.stopping:
            if t >= self.stop_time:
                return 0.0
            
        tau = t % self.T

        if 0.0 <= tau < self.ta:
            return self.amax
        elif self.ta <= tau < (self.ta + self.tc):
            return 0.0
        elif (self.ta + self.tc) <= tau < (2 * self.ta + self.tc):
            return -self.amax
        else:
            return 0.0

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        acc = self.aM(now - self.t0)

        # Integrate acceleration → velocity
        self.v += acc * self.dt

        # Safety clamp
        self.v = max(min(self.v, self.vmax), 0.0)

        msg = Float64MultiArray()
        msg.data = [float(self.v)]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
