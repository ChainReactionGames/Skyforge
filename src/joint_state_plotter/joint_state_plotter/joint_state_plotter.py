#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque
import numpy as np
import matplotlib.pyplot as plt


class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        # ---------------- Config ----------------
        self.maxlen = 1000
        self.joint_names = []
        self.initialized = False

        # ---------------- State Histories ----------------
        self.time = deque(maxlen=self.maxlen)
        self.q = {}
        self.qd = {}
        self.qdd = {}
        self.prev_qd = {}

        # ---------------- Force / Torque ----------------
        self.Fx = deque(maxlen=self.maxlen)
        self.Fy = deque(maxlen=self.maxlen)
        self.Tau = deque(maxlen=self.maxlen)

        self.prev_time = None

        # ---------------- ROS ----------------
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # ---------------- Plot Setup ----------------
        plt.ion()
        self.plots_ready = False
        self.plot_timer = self.create_timer(0.05, self.update_plots)

        self.get_logger().info("joint_state_plotter running")

    # ==================================================
    # Joint State Callback (NO plotting here)
    # ==================================================
    def joint_callback(self, msg: JointState):

        if not msg.name:
            return

        # Initialize joints once
        if not self.initialized:
            self.joint_names = msg.name[:3]   # use first 3 joints
            for j in self.joint_names:
                self.q[j] = deque(maxlen=self.maxlen)
                self.qd[j] = deque(maxlen=self.maxlen)
                self.qdd[j] = deque(maxlen=self.maxlen)
                self.prev_qd[j] = 0.0

            self._setup_plots()
            self.initialized = True
            self.get_logger().info(f"Tracking joints: {self.joint_names}")
            return

        # ROS 2 Jazzy-safe time
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0.0:
            return

        self.time.append(now)

        # Update joint states
        for j in self.joint_names:
            idx = msg.name.index(j)
            q = abs(msg.position[idx])
            qd = msg.velocity[idx] if msg.velocity else 0.0
            qdd = (qd - self.prev_qd[j]) / dt

            self.q[j].append(q)
            self.qd[j].append(qd)
            self.qdd[j].append(qdd)
            self.prev_qd[j] = qd

        self.prev_time = now

        # Compute dynamics
        fx, fy, tau = self.compute_forces()
        self.Fx.append(fx)
        self.Fy.append(fy)
        self.Tau.append(tau)

    # ==================================================
    # Force & Moment Equations (UNCHANGED LOGIC)
    # ==================================================
    def compute_forces(self):

        q1, q2, q3 = [self.q[j][-1] for j in self.joint_names]
        v1, v2, v3 = [self.qd[j][-1] for j in self.joint_names]
        a1, a2, a3 = [self.qdd[j][-1] for j in self.joint_names]

        m1, m2, m3 = 3.07, 3.07, 41.218
        l1, l2 = 0.75, 0.75
        c1, c2, c3 = 0.207655, 0.207655, 0.966762
        aM = 0.0

        Fx = m3 * (
            -l1*v1**2*np.cos(q1)
            - l2*(v1+v2)**2*np.cos(q1+q2)
            - c3*(v1+v2+v3)**2*np.cos(q1+q2+q3)
            - a1*l1*np.sin(q1)
            - (a1+a2)*l2*np.sin(q1+q2)
            - (a1+a2+a3)*c3*np.sin(q1+q2+q3)
        )

        Fy = m3 * (
            aM
            + a1*l1*np.cos(q1)
            + (a1+a2)*l2*np.cos(q1+q2)
            + (a1+a2+a3)*c3*np.cos(q1+q2+q3)
            - l1*v1**2*np.sin(q1)
            - l2*(v1+v2)**2*np.sin(q1+q2)
            - c3*(v1+v2+v3)**2*np.sin(q1+q2+q3)
        )

        Tau = (
            m1*(-c1*v1**2*np.cos(q1) - a1*c1*np.sin(q1))
            + m2*(-l1*v1**2*np.cos(q1)
                  - c2*(v1+v2)**2*np.cos(q1+q2)
                  - a1*l1*np.sin(q1)
                  - (a1+a2)*c2*np.sin(q1+q2))
            + m3*(-l1*v1**2*np.cos(q1)
                  - l2*(v1+v2)**2*np.cos(q1+q2)
                  - c3*(v1+v2+v3)**2*np.cos(q1+q2+q3)
                  - a1*l1*np.sin(q1)
                  - (a1+a2)*l2*np.sin(q1+q2)
                  - (a1+a2+a3)*c3*np.sin(q1+q2+q3))
        )

        return Fx, Fy, Tau

    # ==================================================
    # Plot Setup
    # ==================================================
    def _setup_plots(self):
        plt.ion()

        # --- Force X ---
        self.fig_fx, self.ax_fx = plt.subplots()
        self.line_fx, = self.ax_fx.plot([], [], label="Fx (N)")
        self.ax_fx.set_title("Force in X")
        self.ax_fx.set_xlabel("Time (s)")
        self.ax_fx.set_ylabel("Force (N)")

        # --- Force Y ---
        self.fig_fy, self.ax_fy = plt.subplots()
        self.line_fy, = self.ax_fy.plot([], [], label="Fy (N)")
        self.ax_fy.set_title("Force in Y")
        self.ax_fy.set_xlabel("Time (s)")
        self.ax_fy.set_ylabel("Force (N)")

        # --- Torque ---
        self.fig_tau, self.ax_tau = plt.subplots()
        self.line_tau, = self.ax_tau.plot([], [], label="Torque (Nm)")
        self.ax_tau.set_title("End-Effector Torque")
        self.ax_tau.set_xlabel("Time (s)")
        self.ax_tau.set_ylabel("Torque (Nm)")

        # --- Joint Acceleration ---
        self.fig_acc, self.ax_acc = plt.subplots()
        self.acc_lines = {
            j: self.ax_acc.plot([], [], label=f"{j} accel")[0]
            for j in self.joint_names[:3]
        }
        self.ax_acc.set_title("Joint Accelerations")
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_ylabel("Angular Acceleration (rad/s²)")

        # --- Joint Position ---
        self.fig_q, self.ax_q = plt.subplots()
        self.q_lines = {
            j: self.ax_q.plot([], [], linewidth=2, label=f"{j} position")[0]
            for j in self.joint_names[:3]
        }
        self.ax_q.set_title("Joint Positions")
        self.ax_q.set_xlabel("Time (s)")
        self.ax_q.set_ylabel("Joint Position (rad)")

        # --- Common formatting ---
        for ax in [self.ax_fx, self.ax_fy, self.ax_tau, self.ax_acc, self.ax_q]:
            ax.legend()
            ax.grid(True)

        self.plots_ready = True


    # ==================================================
    # Plot Updates (MAIN THREAD ONLY)
    # ==================================================
# Plot Updates (MAIN THREAD ONLY)
# ==================================================
    def update_plots(self):
        if not self.plots_ready or len(self.time) < 2:
            return

        t = np.array(self.time) - self.time[0]

        # --- Forces / Torque ---
        self.line_fx.set_data(t, self.Fx)
        self.line_fy.set_data(t, self.Fy)
        self.line_tau.set_data(t, self.Tau)

        # --- Joint Accelerations ---
        for j in self.joint_names[:3]:
            self.acc_lines[j].set_data(t, self.qdd[j])

        # --- Joint Positions ---
        for j in self.joint_names[:3]:
            self.q_lines[j].set_data(t, self.q[j])

        # --- Rescale all axes ---
        for ax in [self.ax_fx, self.ax_fy, self.ax_tau, self.ax_acc, self.ax_q]:
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.001)



def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
