#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy
import open3d as o3d
import numpy as np

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/drone/front_depth/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received = False

    def listener_callback(self, msg):
        if not self.received:
            self.get_logger().info('Received point cloud — saving to file...')

            # Convert ROS2 PointCloud2 to structured numpy array
            pc_np = ros2_numpy.numpify(msg)

            # Extract XYZ values, filtering out NaNs
            xyz = np.array([
                [p['x'], p['y'], p['z']]
                for p in pc_np
                if not np.isnan(p['x']) and not np.isnan(p['y']) and not np.isnan(p['z'])
            ])

            if xyz.size == 0:
                self.get_logger().warn("No valid 3D points found in point cloud.")
                return

            # Convert to Open3D point cloud and save
            pc_o3d = o3d.geometry.PointCloud()
            pc_o3d.points = o3d.utility.Vector3dVector(xyz)
            o3d.io.write_point_cloud("spiral_frame.ply", pc_o3d)

            self.get_logger().info('Point cloud saved as spiral_frame.ply')
            self.received = True

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
