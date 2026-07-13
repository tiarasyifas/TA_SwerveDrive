#!/usr/bin/env python3
"""
robotpadi_hardware_bridge.py
-------------------------------------------------------------------------
Bridges the real Arduino Mega (robotpadi_firmware.ino) to ROS 2, using the
SAME topics the Gazebo simulation used. Drop this file into
robotpadi_controller/robotpadi_controller/ alongside swerve_controller.py.

  Subscribes (already published by swerve_controller.py, unchanged):
    /wheel_controller/commands    (std_msgs/Float64MultiArray, 4x rad/s)
    /steering_controller/commands (std_msgs/Float64MultiArray, 4x rad)

  Publishes:
    /joint_states (sensor_msgs/JointState) -- same joint names as
    robotpadi.ros2control ("Revolute 1".."Revolute 8"), populated from the
    Arduino's encoder/step feedback instead of Gazebo. rviz, robot_state_publisher,
    and anything else that consumed /joint_states from the simulated
    joint_state_broadcaster keeps working unmodified against real hardware.

  Serial protocol (matches robotpadi_firmware.ino):
    TX -> "V,w0,w1,w2,w3,s0,s1,s2,s3\n"   (wheel rad/s, steering rad)
    RX <- "F,p0..p3,v0..v3,a0..a3\n"       (wheel rad, wheel rad/s, steer rad)
-------------------------------------------------------------------------
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import serial

# Joint order must match robotpadi.ros2control / controllers.yaml
STEER_JOINTS = ["Revolute 1", "Revolute 2", "Revolute 3", "Revolute 4"]
WHEEL_JOINTS = ["Revolute 5", "Revolute 6", "Revolute 7", "Revolute 8"]

SERIAL_PORT = "/dev/ttyUSB0"   # adjust to your board (e.g. /dev/ttyUSB0, COM5)
SERIAL_BAUD = 115200
CMD_RESEND_HZ = 20.0           # re-send last command periodically as a heartbeat


class RobotpadiHardwareBridge(Node):
    def __init__(self):
        super().__init__('robotpadi_hardware_bridge')

        self.declare_parameter('serial_port', SERIAL_PORT)
        self.declare_parameter('serial_baud', SERIAL_BAUD)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value

        self.get_logger().info(f"Opening serial port {port} @ {baud}")
        self.ser = serial.Serial(port, baud, timeout=0.05)

        self.wheel_cmd = [0.0, 0.0, 0.0, 0.0]
        self.steer_cmd = [0.0, 0.0, 0.0, 0.0]
        self._lock = threading.Lock()

        self.create_subscription(
            Float64MultiArray, '/wheel_controller/commands', self._on_wheel_cmd, 10)
        self.create_subscription(
            Float64MultiArray, '/steering_controller/commands', self._on_steer_cmd, 10)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Periodically push the current command to the Arduino (also acts as
        # the heartbeat the firmware's CMD_TIMEOUT_MS safety check expects).
        self.create_timer(1.0 / CMD_RESEND_HZ, self._send_command)

        # Dedicated thread to read serial without blocking the ROS executor.
        self._stop = False
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info("robotpadi_hardware_bridge ready")

    # ---- ROS callbacks ----

    def _on_wheel_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"/wheel_controller/commands expected 4 values, got {len(msg.data)}")
            return
        with self._lock:
            self.wheel_cmd = list(msg.data)

    def _on_steer_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"/steering_controller/commands expected 4 values, got {len(msg.data)}")
            return
        with self._lock:
            self.steer_cmd = list(msg.data)

    # ---- Serial TX ----

    def _send_command(self):
        with self._lock:
            w = self.wheel_cmd
            s = self.steer_cmd
        line = "V," + ",".join(f"{v:.4f}" for v in (w + s)) + "\n"
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ---- Serial RX (background thread) ----

    def _read_loop(self):
        buf = b""
        while not self._stop and rclpy.ok():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read failed: {e}")
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_feedback_line(line.decode('ascii', errors='ignore').strip())

    def _handle_feedback_line(self, line: str):
        if not line.startswith("F,"):
            return
        parts = line.split(",")[1:]
        if len(parts) != 12:
            return
        try:
            vals = [float(x) for x in parts]
        except ValueError:
            return

        wheel_pos = vals[0:4]
        wheel_vel = vals[4:8]
        steer_pos = vals[8:12]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = STEER_JOINTS + WHEEL_JOINTS
        msg.position = steer_pos + wheel_pos
        msg.velocity = [0.0] * 4 + wheel_vel
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RobotpadiHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
