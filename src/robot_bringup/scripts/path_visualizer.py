#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.declare_parameter('odom_topic', '/ground_truth/odom')
        self.declare_parameter('path_topic', '/visual/path')
        self.declare_parameter('heading_topic', '/visual/heading')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'diff_drive_robot/base')
        self.declare_parameter('min_distance_step', 0.02)  # 2 cm

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.min_dist_step = self.get_parameter('min_distance_step').get_parameter_value().double_value

        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.last_x = None
        self.last_y = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.heading_pub = self.create_publisher(Marker, heading_topic, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Path Visualizer aktif: {odom_topic} -> {path_topic} & {heading_topic}")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        stamp = msg.header.stamp

        # Broadcast TF world -> diff_drive_robot/base
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = pos.z
        t.transform.rotation = ori
        self.tf_broadcaster.sendTransform(t)

        # 1. Update Path (Line Tracking) if moved enough
        should_append = False
        if self.last_x is None or self.last_y is None:
            should_append = True
        else:
            dist = math.hypot(pos.x - self.last_x, pos.y - self.last_y)
            if dist >= self.min_dist_step:
                should_append = True

        if should_append:
            self.last_x = pos.x
            self.last_y = pos.y

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = stamp
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.pose = msg.pose.pose
            self.path.poses.append(pose_stamped)

            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)

        # 2. Heading Direction Arrow Marker (Arah Gerak Robot)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "robot_heading"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow base slightly above chassis
        marker.pose.position.x = pos.x
        marker.pose.position.y = pos.y
        marker.pose.position.z = pos.z + 0.08
        marker.pose.orientation = ori

        # Dimensions: length 0.35m, shaft diameter 0.035m, head diameter 0.07m
        marker.scale.x = 0.35
        marker.scale.y = 0.035
        marker.scale.z = 0.035

        # Bright Cyan/Yellow color (R=0.1, G=0.9, B=1.0)
        marker.color.r = 0.1
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 0.95

        self.heading_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

