#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('odom_topic', '/ground_truth/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dist_tolerance', 0.08)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.6)
        self.declare_parameter('align_threshold', 0.12)
        self.declare_parameter('realign_threshold', 0.35)
        self.declare_parameter('enable_vision_assist', True)
        self.declare_parameter('kp_vision', 0.8)
        self.declare_parameter('loop', False)

        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.dist_tolerance = self.get_parameter('dist_tolerance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.align_threshold = self.get_parameter('align_threshold').get_parameter_value().double_value
        self.realign_threshold = self.get_parameter('realign_threshold').get_parameter_value().double_value
        self.enable_vision_assist = self.get_parameter('enable_vision_assist').get_parameter_value().bool_value
        self.kp_vision = self.get_parameter('kp_vision').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        self.waypoints = []
        self.current_idx = 0
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.mission_completed = False
        self.is_aligning = True

        # Vision assist variables
        self.vision_gate_detected = False
        self.vision_centering_error = 0.0

        self.load_waypoints()

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.gate_det_sub = self.create_subscription(Bool, '/vision/gate_detected', self.vision_gate_callback, 10)
        self.centering_sub = self.create_subscription(Float32, '/vision/centering_error', self.vision_error_callback, 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Waypoint Follower siap. Jumlah waypoint: {len(self.waypoints)}")

    def load_waypoints(self):
        if not os.path.exists(self.waypoint_file):
            # Try searching in current directory and parent directories
            alt_path = os.path.join(os.getcwd(), self.waypoint_file)
            if os.path.exists(alt_path):
                self.waypoint_file = alt_path
            else:
                self.get_logger().error(f"File waypoint tidak ditemukan: {self.waypoint_file}")
                return

        with open(self.waypoint_file, 'r') as f:
            data = yaml.safe_load(f)

        if data and 'waypoints' in data:
            self.waypoints = data['waypoints']
            self.get_logger().info(f"Berhasil memuat {len(self.waypoints)} titik dari {self.waypoint_file}")
        else:
            self.get_logger().error(f"Format file waypoint tidak valid: {self.waypoint_file}")

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def vision_gate_callback(self, msg: Bool):
        self.vision_gate_detected = msg.data

    def vision_error_callback(self, msg: Float32):
        self.vision_centering_error = msg.data

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def control_loop(self):
        if self.mission_completed or not self.waypoints:
            return

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        target = self.waypoints[self.current_idx]
        tx = target['x']
        ty = target['y']

        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)

        # Check if current waypoint reached
        if dist < self.dist_tolerance:
            self.get_logger().info(f"Waypoint #{target.get('id', self.current_idx + 1)} tercapai! (dist={dist:.2f}m)")
            self.current_idx += 1
            self.is_aligning = True  # Rotate to align toward next waypoint
            if self.current_idx >= len(self.waypoints):
                if self.loop:
                    self.get_logger().info("Semua waypoint selesai dikunjungi. Mengulang dari awal (loop mode)...")
                    self.current_idx = 0
                else:
                    self.get_logger().info("Semua waypoint berhasil dilalui. Robot berhenti.")
                    self.stop_robot()
                    self.mission_completed = True
                    return

            target = self.waypoints[self.current_idx]
            tx = target['x']
            ty = target['y']
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.current_yaw)

        twist = Twist()

        # State Machine: ROTATE FIRST, THEN DRIVE
        if self.is_aligning:
            # Pure rotation in place until facing the target waypoint
            twist.linear.x = 0.0
            angular_speed = self.kp_angular * angle_error

            # Overcome static friction with minimum angular speed
            min_rot = 0.15
            if abs(angular_speed) < min_rot:
                angular_speed = math.copysign(min_rot, angle_error)

            twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

            # When heading is aligned within tolerance, transition to drive mode
            if abs(angle_error) <= self.align_threshold:
                self.is_aligning = False
        else:
            # If heading drifts beyond realign threshold, pause and re-align
            if abs(angle_error) > self.realign_threshold:
                self.is_aligning = True
                twist.linear.x = 0.0
                twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, self.kp_angular * angle_error))
            else:
                # Drive forward with proportional speed scaled by heading alignment
                alignment_factor = max(0.0, math.cos(angle_error))
                linear_speed = self.kp_linear * dist * alignment_factor
                twist.linear.x = max(0.06, min(self.max_linear_speed, linear_speed))

                # Fine angular adjustments while moving
                angular_speed = self.kp_angular * angle_error

                # Apply Visual Centering Assist when passing through buoy gates
                if self.enable_vision_assist and self.vision_gate_detected:
                    # Vision centering correction (steer towards gate midpoint)
                    vision_correction = -self.kp_vision * self.vision_centering_error
                    # Weighted combination: 50% Waypoint Path + 50% Visual Gate Centering
                    angular_speed = 0.5 * angular_speed + 0.5 * vision_correction

                twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

