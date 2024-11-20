import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from sensor_msgs_py.point_cloud2 import read_points

class KeyframePointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('keyframe_pointcloud_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/run_slam/keyframes',  # Adjust this topic name to match your setup
            self.point_cloud_callback,
            10
        )

    def point_cloud_callback(self, msg):
        # Extract 3D points from the PointCloud2 message
        points = []
        for p in read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        points = np.array(points)

        # Create an Open3D Point Cloud object
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        # Visualize the point cloud
        self.get_logger().info(f"Visualizing {len(points)} points in the point cloud...")
        o3d.visualization.draw_geometries([pc])

def main(args=None):
    rclpy.init(args=args)
    node = KeyframePointCloudVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
