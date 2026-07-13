#!/usr/bin/env python3
"""
Autonomous paddy waypoint goal publisher.

Publishes /goal_pose targets so navigation can run without manual RViz goals.
Also publishes waypoint and path visualization for RViz.
"""

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class PaddyWaypointGoal(Node):
    def __init__(self):
        super().__init__('paddy_waypoint_goal')

        # Tunable parameters for paddy-lane style serpentine path.
        self.declare_parameter('start_x', -7.0)
        self.declare_parameter('start_y', -2.55)
        self.declare_parameter('row_length', 10.0)
        self.declare_parameter('row_spacing', 0.75)
        self.declare_parameter('row_count', 8)
        self.declare_parameter('goal_tolerance', 0.40)
        self.declare_parameter('loop_forever', True)
        self.declare_parameter('path_max_points', 4000)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.row_length = float(self.get_parameter('row_length').value)
        self.row_spacing = float(self.get_parameter('row_spacing').value)
        self.row_count = int(self.get_parameter('row_count').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.loop_forever = bool(self.get_parameter('loop_forever').value)
        self.path_max_points = int(self.get_parameter('path_max_points').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/paddy_waypoints', 10)
        self.waypoint_path_pub = self.create_publisher(Path, '/paddy_waypoint_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_tracking_path', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_ready = False

        self.waypoints = self._build_serpentine_waypoints()
        self.current_idx = 0
        self.last_goal_idx = -1
        self.completed_once = False

        self.robot_path_msg = Path()
        self.robot_path_msg.header.frame_id = 'odom'

        self.waypoint_path_msg = Path()
        self.waypoint_path_msg.header.frame_id = 'odom'
        self._build_waypoint_path_msg()

        self._publish_waypoint_markers()
        self.waypoint_path_pub.publish(self.waypoint_path_msg)

        period = 1.0 / max(rate_hz, 1e-3)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'PaddyWaypointGoal ready with {len(self.waypoints)} waypoints '
            f'(rows={self.row_count}, spacing={self.row_spacing:.2f}m)'
        )

    def _build_serpentine_waypoints(self):
        points = []
        x0 = self.start_x
        y0 = self.start_y

        for row in range(max(self.row_count, 1)):
            y = y0 + row * self.row_spacing
            if row % 2 == 0:
                points.append((x0, y))
                points.append((x0 + self.row_length, y))
            else:
                points.append((x0 + self.row_length, y))
                points.append((x0, y))
        return points

    def _build_waypoint_path_msg(self):
        self.waypoint_path_msg.poses.clear()
        now = self.get_clock().now().to_msg()
        self.waypoint_path_msg.header.stamp = now
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            self.waypoint_path_msg.poses.append(pose)

    def _publish_waypoint_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Line strip through waypoints.
        line = Marker()
        line.header.frame_id = 'odom'
        line.header.stamp = now
        line.ns = 'paddy_waypoints'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r = 0.0
        line.color.g = 0.7
        line.color.b = 1.0
        line.color.a = 0.9
        line.pose.orientation.w = 1.0
        for x, y in self.waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line.points.append(p)
        marker_array.markers.append(line)

        # Numbered waypoint spheres.
        for i, (x, y) in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = 'odom'
            sphere.header.stamp = now
            sphere.ns = 'paddy_waypoints'
            sphere.id = 100 + i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.20
            sphere.scale.y = 0.20
            sphere.scale.z = 0.20
            sphere.color.r = 0.1
            sphere.color.g = 1.0
            sphere.color.b = 0.2
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = now
            label.ns = 'paddy_waypoints'
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.40
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(i)
            marker_array.markers.append(label)

        # Current target marker.
        target = Marker()
        target.header.frame_id = 'odom'
        target.header.stamp = now
        target.ns = 'paddy_waypoints'
        target.id = 5000
        target.type = Marker.ARROW
        target.action = Marker.ADD
        tx, ty = self.waypoints[self.current_idx]
        target.pose.position.x = tx
        target.pose.position.y = ty
        target.pose.position.z = 0.30
        target.pose.orientation.w = 1.0
        target.scale.x = 0.60
        target.scale.y = 0.12
        target.scale.z = 0.12
        target.color.r = 1.0
        target.color.g = 0.4
        target.color.b = 0.0
        target.color.a = 1.0
        marker_array.markers.append(target)

        self.waypoint_marker_pub.publish(marker_array)

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_ready = True

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose

        self.robot_path_msg.header.stamp = pose.header.stamp
        self.robot_path_msg.poses.append(pose)

        if len(self.robot_path_msg.poses) > self.path_max_points:
            self.robot_path_msg.poses = self.robot_path_msg.poses[-self.path_max_points:]

        self.robot_path_pub.publish(self.robot_path_msg)

    def _publish_goal_for_index(self, idx: int):
        gx, gy = self.waypoints[idx]

        # Face the next waypoint direction when possible.
        nx = idx + 1
        if nx >= len(self.waypoints):
            nx = 0 if self.loop_forever else idx
        tx, ty = self.waypoints[nx]
        yaw = math.atan2(ty - gy, tx - gx)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw)

        self.goal_pub.publish(goal)
        self.last_goal_idx = idx

        self.get_logger().info(f'Published goal idx={idx}: ({gx:.2f}, {gy:.2f})')

    def _advance_goal(self):
        if self.current_idx + 1 < len(self.waypoints):
            self.current_idx += 1
            return

        self.completed_once = True
        if self.loop_forever:
            self.current_idx = 0
            self.get_logger().info('Completed all waypoints, looping to start')
        else:
            self.get_logger().info('Completed all waypoints, stopping goal updates')

    def _control_loop(self):
        if not self.odom_ready:
            return

        if not self.loop_forever and self.completed_once:
            return

        # Publish initial goal and current marker set.
        if self.last_goal_idx < 0:
            self._publish_goal_for_index(self.current_idx)
            self._publish_waypoint_markers()
            self.waypoint_path_pub.publish(self.waypoint_path_msg)
            return

        gx, gy = self.waypoints[self.current_idx]
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)

        if dist <= self.goal_tolerance:
            self._advance_goal()
            if self.loop_forever or not self.completed_once:
                self._publish_goal_for_index(self.current_idx)

        # Keep waypoint visuals alive for RViz.
        self._publish_waypoint_markers()
        self.waypoint_path_pub.publish(self.waypoint_path_msg)


def main():
    rclpy.init()
    node = PaddyWaypointGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
