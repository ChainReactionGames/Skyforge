#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import sys
import traceback

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        self.maxlen = 1000
        self.joint_names = []
        self.initialized = False

        self.time = deque(maxlen=self.maxlen)
        self.q = {}
        self.qd = {}
        self.qdd = {}
        self.prev_qd = {}

        self.Fx = deque(maxlen=self.maxlen)
        self.Fy = deque(maxlen=self.maxlen)
        self.Tau = deque(maxlen=self.maxlen)

        # Base histories
        self.base_pos = deque(maxlen=self.maxlen)
        self.base_vel = deque(maxlen=self.maxlen)
        self.base_acc = deque(maxlen=self.maxlen)  # NEW: base acceleration
        self.prev_base_pos = None
        self.prev_time = None

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        try:
            plt.ion()
            self.plots_ready = False
            self.plot_timer = self.create_timer(0.05, self.update_plots)
        except Exception as e:
            self.get_logger().warn(f"Plotting setup failed: {e}")
            self.plots_ready = False

        self.get_logger().info("joint_state_plotter running")

    # ------------------ JointState callback ------------------
    def joint_callback(self, msg: JointState):
        if not msg.name:
            return

        if not self.initialized:
            # Track first 3 non-base joints
            self.joint_names = [n for n in msg.name if n != "world_to_base"][:3]
            for j in self.joint_names:
                self.q[j] = deque(maxlen=self.maxlen)
                self.qd[j] = deque(maxlen=self.maxlen)
                self.qdd[j] = deque(maxlen=self.maxlen)
                self.prev_qd[j] = 0.0

            try:
                self._setup_plots()
            except:
                self.get_logger().warn("Failed to setup plots")

            self.initialized = True
            self.get_logger().info(f"Tracking joints: {self.joint_names}")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0.0:
            return

        self.time.append(now)

        # --- Update arm joints ---
        for j in self.joint_names:
            if j in msg.name:
                idx = msg.name.index(j)
                q = msg.position[idx]
                qd = msg.velocity[idx] if msg.velocity else 0.0
                qdd = (qd - self.prev_qd[j]) / dt

                self.q[j].append(q)
                self.qd[j].append(qd)
                self.qdd[j].append(qdd)
                self.prev_qd[j] = qd

        # --- Update base from "world_to_base" joint ---
        if "world_to_base" in msg.name:
            idx = msg.name.index("world_to_base")
            base_p = msg.position[idx]
            self.base_pos.append(base_p)

            # Velocity: use msg if available, else finite difference
            if msg.velocity and len(msg.velocity) > idx:
                base_v = msg.velocity[idx]
            elif self.prev_base_pos is not None:
                base_v = (base_p - self.prev_base_pos) / dt
            else:
                base_v = 0.0
            self.base_vel.append(base_v)

            # Acceleration: finite difference of velocity
            if len(self.base_vel) >= 2:
                self.base_acc.append((self.base_vel[-1] - self.base_vel[-2]) / dt)
            else:
                self.base_acc.append(0.0)

            self.prev_base_pos = base_p

        self.prev_time = now

        # Compute forces/torque
        fx, fy, tau = self.compute_forces()
        self.Fx.append(fx)
        self.Fy.append(fy)
        self.Tau.append(tau)

    # ------------------ Forces computation ------------------
    def compute_forces(self):
        q1, q2, q3 = [self.q[j][-1] for j in self.joint_names]
        v1, v2, v3 = [self.qd[j][-1] for j in self.joint_names]
        a1, a2, a3 = [self.qdd[j][-1] for j in self.joint_names]

        m3 = 41.218
        l1, l2, l3 = 0.75, 0.75, 2
        c3 = 0.966762
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

        Tau = ((a1 + a2 + a3)*l3
            - c3*m3*np.sin(q1+q2+q3) * (
                -l1*v1**2*np.cos(q1)
                - l2*(v1+v2)**2*np.cos(q1+q2)
                - c3*(v1+v2+v3)**2*np.cos(q1+q2+q3)
                - a1*l1*np.sin(q1)
                - (a1+a2)*l2*np.sin(q1+q2)
                - (a1+a2+a3)*c3*np.sin(q1+q2+q3)
            )
            + c3*m3*np.cos(q1+q2+q3) * (
                aM
                + a1*l1*np.cos(q1)
                + (a1+a2)*l2*np.cos(q1+q2)
                + (a1+a2+a3)*c3*np.cos(q1+q2+q3)
                - l1*v1**2*np.sin(q1)
                - l2*(v1+v2)**2*np.sin(q1+q2)
                - c3*(v1+v2+v3)**2*np.sin(q1+q2+q3)
            )
        )

        return Fx, Fy, Tau

    # ------------------ Plot setup ------------------
    def _setup_plots(self):
        plt.ion()

        self.fig_fx, self.ax_fx = plt.subplots()
        self.line_fx, = self.ax_fx.plot([], [], label="Fx (N)")
        self.ax_fx.set_title("Force in X")

        self.fig_fy, self.ax_fy = plt.subplots()
        self.line_fy, = self.ax_fy.plot([], [], label="Fy (N)")
        self.ax_fy.set_title("Force in Y")

        self.fig_tau, self.ax_tau = plt.subplots()
        self.line_tau, = self.ax_tau.plot([], [], label="Torque")
        self.ax_tau.set_title("Torque")

        self.fig_acc, self.ax_acc = plt.subplots()
        self.acc_lines = {j: self.ax_acc.plot([], [], label=f"{j} accel")[0] for j in self.joint_names}
        self.ax_acc.set_title("Joint Accelerations")

        self.fig_q, self.ax_q = plt.subplots()
        self.q_lines = {j: self.ax_q.plot([], [], label=f"{j} pos")[0] for j in self.joint_names}
        self.ax_q.set_title("Joint Positions")

        # Base plots
        self.fig_bp, self.ax_bp = plt.subplots()
        self.line_bp, = self.ax_bp.plot([], [], label="Base Position")
        self.ax_bp.set_title("Base Position")

        self.fig_bv, self.ax_bv = plt.subplots()
        self.line_bv, = self.ax_bv.plot([], [], label="Base Velocity")
        self.ax_bv.set_title("Base Velocity")

        self.fig_ba, self.ax_ba = plt.subplots()  # NEW: Base acceleration
        self.line_ba, = self.ax_ba.plot([], [], label="Base Acceleration")
        self.ax_ba.set_title("Base Acceleration")

        for ax in [self.ax_fx, self.ax_fy, self.ax_tau, self.ax_acc, self.ax_q,
                   self.ax_bp, self.ax_bv, self.ax_ba]:
            ax.legend()
            ax.grid(True)

        self.plots_ready = True

    # ------------------ Plot update ------------------
    def update_plots(self):
        if not self.plots_ready or len(self.time) < 2:
            return
        try:
            t = np.array(self.time) - self.time[0]

            self.line_fx.set_data(t, self.Fx)
            self.line_fy.set_data(t, self.Fy)
            self.line_tau.set_data(t, self.Tau)

            for j in self.joint_names:
                self.acc_lines[j].set_data(t, self.qdd[j])
                self.q_lines[j].set_data(t, self.q[j])

            # Base plots
            self.line_bp.set_data(t, self.base_pos)
            self.line_bv.set_data(t, self.base_vel)
            self.line_ba.set_data(t, self.base_acc)

            for ax in [self.ax_fx, self.ax_fy, self.ax_tau, self.ax_acc, self.ax_q,
                       self.ax_bp, self.ax_bv, self.ax_ba]:
                ax.relim()
                ax.autoscale_view()

            plt.pause(0.001)
        except Exception:
            pass

    # ------------------ Print metrics ------------------
    def print_metrics(self):
        try:
            t = np.array(self.time) - self.time[0] if self.time else np.array([0])

            signals = {
                "Fx": self.Fx,
                "Fy": self.Fy,
                "Torque": self.Tau,
                "Base Pos": self.base_pos,
                "Base Vel": self.base_vel,
                "Base Acc": self.base_acc  # NEW
            }

            for j in self.joint_names:
                signals[f"{j} Pos"] = self.q[j]
                signals[f"{j} Accel"] = self.qdd[j]

            print("\n===== CONTROL PERFORMANCE METRICS =====")
            for name, data in signals.items():
                data = np.array(data)
                if len(data) < 2:
                    continue
                peak = np.max(data)
                peak_time = t[np.argmax(data)]
                rms = np.sqrt(np.mean(data**2))
                final = data[-1]
                os_percent = (peak - final)/abs(final)*100 if final != 0 else float('nan')
                print(f"{name:>12} | Peak: {peak:8.4f} at {peak_time:6.3f}s | RMS: {rms:8.4f} | %OS: {os_percent:6.2f}")
            print("======================================\n")
            sys.stdout.flush()
        except Exception:
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down...")
    finally:
        node.print_metrics()
        node.destroy_node()
        rclpy.shutdown()
