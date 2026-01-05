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
            10
        )

        timer_period = 10.0  # seconds
        self.timer = self.create_timer(timer_period, self.send_trajectory)
        self.get_logger().info("Trajectory Publisher Node Started")

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['world_to_base']

        def make_point(pos, vel, t_sec, t_nsec=0):
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.velocities = [vel]
            point.time_from_start.sec = t_sec
            point.time_from_start.nanosec = t_nsec
            return point

        # Sequence: start, stop, start, stop, start
        points = []

        # Point 1: Start at 0
        points.append(make_point(0.0, 0.0, 0))

        # Point 2: Move to 1.0 over 2s
        points.append(make_point(1.0, 0.3, 2))

        # Point 3: Stop for 1s at 1.0
        points.append(make_point(1.0, 0.0, 3))

        # Point 4: Move to 2.0 over 2s
        points.append(make_point(2.0, 0.3, 5))

        # Point 5: Stop for 1s at 2.0
        points.append(make_point(2.0, 0.0, 6))

        # Point 6: Move to 3.0 over 2s
        points.append(make_point(3.0, 0.3, 8))

        # Point 7: Stop for 1s at 3.0
        points.append(make_point(3.0, 0.0, 9))

        # Point 8: Move to 4.0 over 2s
        points.append(make_point(4.0, 0.3, 11))

        # Add points to trajectory
        traj.points.extend(points)

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
