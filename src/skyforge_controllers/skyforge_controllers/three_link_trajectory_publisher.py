#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.send_trajectory)
        self.get_logger().info("Trajectory Publisher Node Started")

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        point.effort = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info("Trajectory command published!")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
