#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import time


class JointMonitorPlotter(Node):
    def __init__(self, reference_positions=None, log_interval=2.0):
        super().__init__('joint_monitor_plotter')

        # Plot groups
        self.plot_groups = {
            "arm_joints": ["joint1", "joint2", "joint3"],
            "base_transform": ["world_to_base"]
        }

        self.reference_positions = reference_positions or {}
        self.log_interval = log_interval

        self.start_time = time.time()
        self.last_log_time = time.time()

        # Shared time history
        self.time_history = deque(maxlen=1000)

        # Per-joint data
        self.joint_history = {
            joint: deque(maxlen=1000)
            for joints in self.plot_groups.values()
            for joint in joints
        }

        # Plot handles
        self.figures = {}
        self.axes = {}
        self.lines = {}

        self._setup_plots()

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def _setup_plots(self):
        plt.ion()

        for group_name, joints in self.plot_groups.items():
            fig, ax = plt.subplots()
            self.figures[group_name] = fig
            self.axes[group_name] = ax
            self.lines[group_name] = {}

            for joint in joints:
                line, = ax.plot([], [], label=joint)
                self.lines[group_name][joint] = line

            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position")
            ax.set_title(group_name.replace("_", " ").title())
            ax.legend()

    def joint_callback(self, msg: JointState):
        t = time.time() - self.start_time
        self.time_history.append(t)

        # Store joint data
        for joint in self.joint_history.keys():
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.joint_history[joint].append(abs(msg.position[idx]))
            else:
                self.joint_history[joint].append(
                    self.joint_history[joint][-1] if self.joint_history[joint] else 0.0
                )

        self.update_plots()

        if time.time() - self.last_log_time > self.log_interval:
            metrics = self.compute_metrics()
            for joint, m in metrics.items():
                self.get_logger().info(f"[Metrics] {joint}: {m}")
            self.last_log_time = time.time()

    def update_plots(self):
        times = np.array(self.time_history)

        for group_name, joints in self.plot_groups.items():
            ax = self.axes[group_name]

            for joint in joints:
                positions = np.array(self.joint_history[joint])
                self.lines[group_name][joint].set_data(times, positions)

                if joint in self.reference_positions:
                    ref = self.reference_positions[joint]
                    ax.plot(times, np.ones_like(times) * ref, '--', color='gray')

            ax.relim()
            ax.autoscale_view()

        plt.pause(0.001)

    def compute_metrics(self):
        results = {}
        times = np.array(self.time_history)

        for joint, positions_deque in self.joint_history.items():
            positions = np.array(positions_deque)
            if len(positions) < 5:
                continue

            ref = self.reference_positions.get(joint, positions[-1])
            results[joint] = self._calculate_metrics(times, positions, ref)

        return results

    @staticmethod
    def _calculate_metrics(time, position, reference):
        error = reference - position
        steady_state_error = error[-1]

        try:
            t10 = time[np.where(position >= 0.1 * reference)[0][0]]
            t90 = time[np.where(position >= 0.9 * reference)[0][0]]
            rise_time = t90 - t10
        except IndexError:
            rise_time = None

        overshoot = ((np.max(position) - reference) / reference) * 100 if reference != 0 else 0.0

        tol = 0.02 * abs(reference)
        out_of_bounds = np.where(np.abs(position - reference) > tol)[0]
        settling_time = time[out_of_bounds[-1]] if len(out_of_bounds) > 0 else 0.0

        return {
            "rise_time": rise_time,
            "overshoot_%": overshoot,
            "settling_time": settling_time,
            "steady_state_error": steady_state_error
        }


def main(args=None):
    rclpy.init(args=args)

    reference_positions = {
        "joint1": 0.25,
        "joint2": 0.85,
        "joint3": 1.1,
    }

    node = JointMonitorPlotter(reference_positions=reference_positions)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        final_metrics = node.compute_metrics()
        for joint, m in final_metrics.items():
            print(f"[Final Metrics] {joint}: {m}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
