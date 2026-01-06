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
        self.amplitude = 1.0       # maximum displacement
        self.frequency = 0.2       # Hz (cycles per second)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Publish at 50 Hz for smooth motion
        freq = 50.0
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.send_trajectory)
        self.get_logger().info("Smooth Oscillating Base Node Started")


    def send_trajectory(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.start_time

        position = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
        velocity = (2 * math.pi * self.frequency) * self.amplitude * math.cos(2 * math.pi * self.frequency * t) 

        traj = JointTrajectory()
        traj.joint_names = ['world_to_base']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]  # Velocity of last point in array has to be zero :() 
        point.time_from_start.sec = 0
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
