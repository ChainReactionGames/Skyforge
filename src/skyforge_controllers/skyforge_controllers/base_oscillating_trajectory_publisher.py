#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class OscillatingBasePublisher(Node):
    def __init__(self):
        super().__init__('oscillating_trajectory_publisher')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/base_trajectory_controller/joint_trajectory',
            10
        )

         # Oscillation parameters
        self.amplitude = 0.15       # maximum displacement
        self.frequency = 0.2       # Hz (cycles per second)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.send_trajectory)
        self.get_logger().info("Smooth Oscillating Base Node Started")


    def send_trajectory(self):
        # current_time = self.get_clock().now().nanoseconds / 1e9
        # t = current_time - self.start_time

        # position = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
        # velocity = (2 * math.pi * self.frequency) * self.amplitude * math.cos(2 * math.pi * self.frequency * t) 

        traj = JointTrajectory()
        traj.joint_names = ['world_to_base']

        oscillationResolution = 10  # Number of points in one oscillation cycle
        for i in range(oscillationResolution):
            j = i + 1
            dt = j * (1.0 / (self.frequency * oscillationResolution))
            pos = self.amplitude * math.cos(2 * math.pi * self.frequency * (dt))
            vel = (2 * math.pi * self.frequency) * self.amplitude * -1 * math.sin(2 * math.pi * self.frequency * (dt))

            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.velocities = [vel]
            point.time_from_start.sec = int(dt)
            point.time_from_start.nanosec = int((dt - int(dt)) * 1e9)
            traj.points.append(point)


        self.publisher_.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = OscillatingBasePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
