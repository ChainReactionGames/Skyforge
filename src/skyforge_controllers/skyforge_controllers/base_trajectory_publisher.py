#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/base_trajectory_controller/joint_trajectory',
            1
        )

        timer_period = 3#10.0  # seconds
        self.timer = self.create_timer(timer_period, self.send_trajectory)
        self.get_logger().info("Trajectory Publisher Node Started")

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['world_to_base']

        # Point 1: Starting at zero
        point1 = JointTrajectoryPoint()
        point1.positions = [0]
        point1.velocities = [0]
        point1.time_from_start.sec = 0
        traj.points.append(point1)

        # Point 2: Moving at constant speed
        point2 = JointTrajectoryPoint()
        point2.positions = [1]#[2.0]
        point2.velocities = [0]
        point2.time_from_start.sec = 1#4
        traj.points.append(point2)

        # Point 3: Moving at constant speed
        point3 = JointTrajectoryPoint()
        point3.positions = [-1]#[4.0]
        point3.velocities = [0]#[0]
        point3.time_from_start.sec = 2#8
        traj.points.append(point3)

        self.publisher_.publish(traj)
        self.get_logger().info("Base trajectory command published!")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
