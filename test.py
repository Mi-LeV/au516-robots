#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from sensor_msgs.point_cloud2 import read_points

def point_cloud_callback(msg):
    # Extract 3D points from the PointCloud2 message
    points = []
    for p in read_points(msg, skip_nans=True):
        points.append([p[0], p[1], p[2]])
    points = np.array(points)

    # Create an Open3D Point Cloud object
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pc])

def main():
    rospy.init_node('keyframe_pointcloud_visualizer')
    rospy.Subscriber('/orb_slam2_mono/keyframes', PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
