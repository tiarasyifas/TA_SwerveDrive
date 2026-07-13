#!/usr/bin/env python3
"""
Differential Drive Nav Goal Node.
Subscribes to /goal_pose and drives to goal with a simple state machine:
ROTATE -> DRIVE -> ALIGN -> DONE.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Robot geometry (must match diff_controller.py)
TRACK_WIDTH = 0.950
HALF_TRACK = TRACK_WIDTH / 2.0
WHEEL_RADIUS = 0.2

# Tuning
GOAL_TOLERANCE_M = 0.20
GOAL_TOLERANCE_RAD = 0.05

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0
MIN_ANGULAR_SPEED = 0.15

LINEAR_KP = 0.6
ANGULAR_KP = 1.2

ROTATE_FIRST_THRESH = 0.30

# States
IDLE = 'IDLE'
ROTATE = 'ROTATE'
DRIVE = 'DRIVE'
ALIGN = 'ALIGN'
DONE = 'DONE'


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def diff_kinematics(linear: float, omega: float):
    left_ms = linear - (omega * HALF_TRACK)
    right_ms = linear + (omega * HALF_TRACK)
    return left_ms / WHEEL_RADIUS, right_ms / WHEEL_RADIUS


class DiffNavGoal(Node):
    def __init__(self):
        super().__init__('diff_nav_goal')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.state = IDLE

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('DiffNavGoal ready - set a 2D Nav Goal in RViz')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation) + math.pi / 2.0
        self.odom_received = True

    def _goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)
        self.state = ROTATE

        self.get_logger().info(
            f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'yaw={math.degrees(self.goal_yaw):.1f} deg)'
        )

    def _control_loop(self):
        if not self.odom_received or self.state in (IDLE, DONE):
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_wrap(-angle_to_goal + self.yaw)
        align_error = angle_wrap(-self.goal_yaw + self.yaw)

        if self.state == ROTATE:
            if abs(heading_error) < GOAL_TOLERANCE_RAD:
                self.state = DRIVE
                self.get_logger().info('Heading aligned -> DRIVE')
            else:
                self._publish(0.0, self._angular_cmd(heading_error))

        elif self.state == DRIVE:
            if distance < GOAL_TOLERANCE_M:
                self._publish(0.0, 0.0)
                self.state = ALIGN
                self.get_logger().info('Position reached -> ALIGN')
                return

            if abs(heading_error) > ROTATE_FIRST_THRESH:
                self._publish(0.0, self._angular_cmd(heading_error))
                return

            linear = min(LINEAR_KP * distance, MAX_LINEAR_SPEED)
            omega = self._angular_cmd(heading_error) * 0.5
            self._publish(linear, omega)

        elif self.state == ALIGN:
            if abs(align_error) < GOAL_TOLERANCE_RAD:
                self._publish(0.0, 0.0)
                self.state = DONE
                self.get_logger().info('Goal complete -> DONE')
            else:
                self._publish(0.0, self._angular_cmd(align_error))

    def _angular_cmd(self, error: float) -> float:
        raw = ANGULAR_KP * error
        raw = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, raw))
        if 0.0 < abs(raw) < MIN_ANGULAR_SPEED:
            raw = math.copysign(MIN_ANGULAR_SPEED, raw)
        return raw

    def _publish(self, linear: float, omega: float):
        left_rad_s, right_rad_s = diff_kinematics(linear, omega)

        wheel_signs = [-1, -1, 1, 1]
        wheel_rates = [left_rad_s, left_rad_s, right_rad_s, right_rad_s]

        msg = Float64MultiArray()
        msg.data = [wheel_signs[i] * wheel_rates[i] for i in range(4)]
        self.wheel_pub.publish(msg)


def main():
    rclpy.init()
    node = DiffNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
