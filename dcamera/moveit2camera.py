#!/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import SetBool,Empty
import numpy as np
import open3d as o3d
import sys
import multiprocessing
import threading

from dcamera import Mechmind


class Moveit2PCD(multiprocessing.Process):
    def __init__(self, camera_K):
        super().__init__()
        self.camera_queue = multiprocessing.Manager().Queue(10)
        self.robot_running_queue = multiprocessing.Manager().Queue(1)
        self.k_inv = np.linalg.inv(camera_K)
        self.start()
             
    def run(self):
        rclpy.init(args=sys.argv)
        node = Node('pcd_pub')
        pcd_pub = node.create_publisher(PointCloud2, "/mechmind/pcd", 20)
        clear_flag = False
        clear_octomap_client = node.create_client(Empty, "/clear_octomap")
        
        while not clear_octomap_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        def clear_octomap():
            while True:
                clear_flag = self.robot_running_queue.get()
                if clear_flag:
                    clear_octomap_client.call(Empty.Request())
                    clear_flag = False
        clear_octomap_thread = threading.Thread(target=clear_octomap,daemon=True, args=())
        clear_octomap_thread.start()
   
        while rclpy.ok():
            rgb, depth = self.camera_queue.get()
            header = Header()
            pcd = Moveit2PCD.get_cld_in_camera(rgb,depth,self.k_inv).astype(np.float32)
            header.frame_id = "camera_link"
            header.stamp = node.get_clock().now().to_msg()
            fields_3d_color = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyzrgb')]
            pcd2 = point_cloud2.create_cloud(header=header, fields=fields_3d_color,  points=pcd)
            # pcd2 = point_cloud2.create_cloud_xyz32(header=header, points=pcd)
            
            pcd_pub.publish(pcd2)
            
        node.destroy_node()
        rclpy.shutdown()
    
    @staticmethod
    def get_cld_in_camera(rgb, depth, k_inv):
        valid_depth = depth >= 1e-6
        points_2d = np.argwhere(valid_depth)[:, (1, 0)]
        depth = depth[points_2d[:, 1], points_2d[:, 0]]
        rgb = rgb[points_2d[:, 1], points_2d[:, 0], ::-1]
 
        ones = np.ones((points_2d.shape[0], 1), dtype=points_2d.dtype)
        # points_2d_nml = np.hstack((points_2d, ones))
        points_2d_nml = np.c_[points_2d, ones]
        points_3d = k_inv.dot(points_2d_nml.T).T
        # points_3d = points_3d * np.expand_dims(depth, axis=0).repeat(3, axis=0).T
        points_3d = np.multiply(points_3d, np.expand_dims(depth, axis=0).repeat(3, axis=0).T)
        # points, colors = Moveit2PCD.outlier_remove(points_3d[::100], rgb[::100])
        pcd = np.c_[points_3d[::100], rgb[::100]]
        return pcd
    
    @staticmethod
    def outlier_remove(points:np.array, colors:np.array) -> tuple[np.array, np.array]:
        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(points)
        o3d_pcd.colors = o3d.utility.Vector3dVector(colors/255.0)
        _, ind = o3d_pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2.0)
        inlier_cloud = o3d_pcd.select_by_index(ind)
        return np.asarray(inlier_cloud.points), np.asarray(inlier_cloud.colors)*255.0
