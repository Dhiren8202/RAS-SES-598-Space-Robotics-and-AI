#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy
import open3d as o3d
import numpy as np
import os

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/drone/front_depth/points',
            self.listener_callback,
            10)
        self.subscription
        self.received = False

        # Ensure save directory exists
        self.output_dir = 'pointcloud_data'
        os.makedirs(self.output_dir, exist_ok=True)

    def listener_callback(self, msg):
        if not self.received:
            self.get_logger().info('Received point cloud — saving to file...')

            # Convert to NumPy structured array
            pc_np = ros2_numpy.numpify(msg)

            # Extract XYZ with NaN filtering
            xyz = np.array([
                [p['x'].item(), p['y'].item(), p['z'].item()]
                for p in pc_np
                if not np.isnan(p['x']) and not np.isnan(p['y']) and not np.isnan(p['z'])
            ])

            if xyz.size == 0:
                self.get_logger().warn("No valid points found.")
                return

            # Save using Open3D
            pc_o3d = o3d.geometry.PointCloud()
            pc_o3d.points = o3d.utility.Vector3dVector(xyz)

            filepath = os.path.join(self.output_dir, "spiral_frame.ply")
            o3d.io.write_point_cloud(filepath, pc_o3d)

            self.get_logger().info(f"Point cloud saved to {filepath}")
            self.received = True

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
