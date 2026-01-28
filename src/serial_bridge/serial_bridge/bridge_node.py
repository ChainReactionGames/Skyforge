#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.port = '/dev/ttyUSB0'
        self.baud = 115200

        # Connect to ESP32
        self.get_logger().info(f'Starting SerialBridge on {self.port} at {self.baud} baud')
        connected = False
        while not connected:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                connected = True
            except serial.SerialException:
                self.get_logger().warn(f'Port {self.port} busy. Waiting...')
                time.sleep(1)
        self.get_logger().info('Serial port opened successfully')

        # Handshake
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.write(b'HELLO\n')
        self.get_logger().info('Sent handshake: HELLO')

        # ROS subscriptions and publishers
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/tau_cmd', 10)

        self.latest_q = [1.984107747, -1.318508983, 0.905197563] # Whatever the home state is should be the default so the controller doesn't freak out if no data is being sent yet
        self.latest_dq = [0.0,0.0,0.0]

    def joint_callback(self, msg: JointState):
        # Store latest positions and velocities
        self.latest_q = list(msg.position[:3])
        self.latest_dq = list(msg.velocity[:3])

    def send_and_receive(self):
        # Send joint states to ESP32
        line = f"{self.latest_q[0]},{self.latest_q[1]},{self.latest_q[2]},"
        line += f"{self.latest_dq[0]},{self.latest_dq[1]},{self.latest_dq[2]}\n"
        self.get_logger().info(f'Sending joint states: {line}')

        self.ser.write(line.encode())

        # Read ESP32 response (torques)
        if self.ser.in_waiting:
            raw = self.ser.readline().decode().strip()
            if raw:
                self.get_logger().info(f'Received: {raw}')
                # Convert CSV to floats
                try:
                    if raw[0].isdigit() or raw[0] == '-':
                        tau_vals = [float(x) for x in raw.split(',')]
                        msg = Float64MultiArray(data=tau_vals)
                        self.pub.publish(msg)
                        self.get_logger().info(f'Torques: {tau_vals}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to parse: {raw} -> {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.send_and_receive()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down bridge')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
