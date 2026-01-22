#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
from controller_manager_msgs.srv import ListControllers

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/base_trajectory_controller/joint_trajectory',
            1
        )

        # Acceleration parameters
        self.amax = 0.0625    # maximum acceleration
        self.vmax = 0.1       # maximum velocity
        self.T = 30.0         # total period
        self.L = 2.0          # total distance to travel in one period

        self.ta = self.vmax / self.amax                        # acceleration time
        self.tc = (self.L - self.amax * self.ta**2)/self.vmax  # constant velocity (coast) time


        self.stopping = True   # whether to stop after some time
        self.stop_time = 90.0  # time to stop the motion if self.stopping is True


        # Trajectory publishing parameters

        self.dt = 0.02              # time step for trajectory points
        self.horizon = 2.0          # seconds of trajectory to publish each time
        self.publish_period = 0.1   # seconds between trajectory publications

        # State variables
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.last_t = 0.0  # time since start
        self.y = 0.0       # current position
        self.v = 0.0       # current velocity


        # self.declare_parameter('use_sim_time', True)

        self.get_logger().info("Waiting for clock to start")
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Clock started, starting trajectory publisher")

        # Wait for controller to be active
        self.wait_for_controller_active('lqr_arm_controller')
        self.wait_for_controller_active('base_trajectory_controller')

        self.t0 = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(self.publish_period, self.send_trajectory)
        self.get_logger().info("Trajectory Publisher Node Started (aM based)")

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

    def to_duration(self, t: float) -> Duration:
        sec = int(math.floor(t))
        nanosec = int((t - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def integrate_to(self, t_end: float):
        t = self.last_t
        y = self.y
        v = self.v
        while t < t_end - 1e-12:
            h = min(self.dt, t_end - t)
            a = self.aM(t)
            v += a * h
            y += v * h
            t += h

        self.last_t = t
        self.y = y
        self.v = v

    def sample_state_at(self, t_query: float):
        self.integrate_to(t_query)
        return self.y, self.v
    
    def send_trajectory(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t_now = now - self.t0

        self.sample_state_at(t_now)
        y0, v0 = self.y, self.v

        traj = JointTrajectory()
        traj.joint_names = ['world_to_base']
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        # traj.header.stamp = self.get_clock().now().to_msg()

        t_rel = 0.0
        while t_rel < self.horizon + 1e-12:
            t_abs = t_now + t_rel
            y, v = self.sample_state_at(t_abs)

            pt = JointTrajectoryPoint()
            pt.positions = [float(y)]
            # pt.velocities = [float(v)]
            pt.time_from_start = self.to_duration(t_rel)
            traj.points.append(pt)

            t_rel += self.dt

        self.publisher_.publish(traj)
        self.get_logger().info("Published base trajectory command from t={:.3f} s, pos {:.3f} m, vel {:.3f} m/s".format(t_now, y0, v0))

        # # Point 1: Starting at zero
        # point1 = JointTrajectoryPoint()
        # point1.positions = [0]
        # point1.velocities = [0]
        # point1.time_from_start.sec = 0
        # traj.points.append(point1)

        # # Point 2: Moving at constant speed
        # point2 = JointTrajectoryPoint()
        # point2.positions = [2.0]
        # point2.velocities = [0]
        # point2.time_from_start.sec = 4
        # traj.points.append(point2)

        # # Point 3: Moving at constant speed
        # point3 = JointTrajectoryPoint()
        # point3.positions = [4.0]
        # point3.velocities = [0]
        # point3.time_from_start.sec = 8
        # traj.points.append(point3)

        # self.publisher_.publish(traj)
        # self.get_logger().info("Base trajectory command published!")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
