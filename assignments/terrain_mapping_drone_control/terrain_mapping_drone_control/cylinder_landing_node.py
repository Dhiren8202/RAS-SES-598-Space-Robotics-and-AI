#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time

class ArucoLandingController(Node):
    def __init__(self):
        super().__init__('aruco_landing_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_marker/pose', self.marker_callback, 10)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        self.ctrl_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.marker_pose = None
        self.offboard_counter = 0
        self.aligned = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def marker_callback(self, msg):
        self.marker_pose = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.ctrl_mode_pub.publish(msg)

    def arm(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_command(self, command, param1=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def send_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.offboard_counter == 10:
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.arm()
            self.get_logger().info('Offboard mode and arm sent')
        self.offboard_counter += 1

        if self.marker_pose is None:
            self.get_logger().info("Waiting for marker pose...")
            return

        x = self.marker_pose.pose.position.x
        y = self.marker_pose.pose.position.y
        z = 0.5  # Descend to 0.5m above marker

        # Check alignment
        if abs(x) < 0.1 and abs(y) < 0.1:
            if not self.aligned:
                self.get_logger().info("Aligned with marker. Initiating landing...")
                self.land()
                self.aligned = True
        else:
            self.send_setpoint(x, y, z)
            self.get_logger().info(f"Aligning to marker: x={x:.2f}, y={y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
