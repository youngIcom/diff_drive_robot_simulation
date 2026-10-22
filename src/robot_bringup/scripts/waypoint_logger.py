#!/usr/bin/env python3
import os
import sys
import math
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import yaml


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')

        self.declare_parameter('odom_topic', '/ground_truth/odom')
        self.declare_parameter('output_file', 'waypoints.yaml')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.current_pose = None
        self.waypoints = []

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f"Waypoint Logger aktif. Mendengarkan odom: {odom_topic}")
        self.get_logger().info(f"Target file output: {os.path.abspath(self.output_file)}")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori)
        self.current_pose = {
            'x': round(pos.x, 4),
            'y': round(pos.y, 4),
            'z': round(pos.z, 4),
            'yaw': round(yaw, 4)
        }

    def record_waypoint(self):
        if self.current_pose is None:
            print("\n[PERINGATAN] Data odometry belum diterima!")
            return
        wp = {
            'id': len(self.waypoints) + 1,
            'x': self.current_pose['x'],
            'y': self.current_pose['y'],
            'yaw': self.current_pose['yaw']
        }
        self.waypoints.append(wp)
        print(f"\n[TERCATAT] Waypoint #{wp['id']}: x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={wp['yaw']:.3f} rad ({math.degrees(wp['yaw']):.1f}°)")

    def delete_last_waypoint(self):
        if self.waypoints:
            removed = self.waypoints.pop()
            print(f"\n[DIHAPUS] Waypoint #{removed['id']} dihapus. Total tersisa: {len(self.waypoints)}")
        else:
            print("\n[INFO] Belum ada waypoint yang dicatat.")

    def save_to_file(self):
        if not self.waypoints:
            print("\n[INFO] Tidak ada waypoint untuk disimpan.")
            return False

        data = {'waypoints': self.waypoints}
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"\n[SUKSES] {len(self.waypoints)} waypoint berhasil disimpan ke: {os.path.abspath(self.output_file)}")
        return True


def main(args=None):
    rclpy.init(args=args)
    logger_node = WaypointLogger()

    print("=" * 60)
    print("      MANUAL WAYPOINT LOGGER ROBOT")
    print("=" * 60)
    print("Petunjuk:")
    print(" [ENTER] / [SPACE] : Catat posisi robot saat ini sebagai waypoint")
    print(" [d]               : Hapus waypoint terakhir")
    print(" [s]               : Simpan ke file YAML dan keluar")
    print(" [q]               : Keluar tanpa menyimpan")
    print("=" * 60)

    # Setup terminal raw mode for non-blocking key read
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            rclpy.spin_once(logger_node, timeout_sec=0.05)

            # Check if stdin has input ready
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist:
                key = sys.stdin.read(1)
                if key in ('\n', '\r', ' '):
                    logger_node.record_waypoint()
                elif key.lower() == 'd':
                    logger_node.delete_last_waypoint()
                elif key.lower() == 's':
                    logger_node.save_to_file()
                    break
                elif key.lower() == 'q':
                    print("\nKeluar tanpa menyimpan.")
                    break

            # Live coordinate display on the same line if odom available
            if logger_node.current_pose:
                p = logger_node.current_pose
                deg = math.degrees(p['yaw'])
                count = len(logger_node.waypoints)
                sys.stdout.write(f"\rPose saat ini: x={p['x']:6.3f} | y={p['y']:6.3f} | yaw={deg:6.1f}° | Tercatat: {count} wp ")
                sys.stdout.flush()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

