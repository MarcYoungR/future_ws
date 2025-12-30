#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import sensor_msgs.point_cloud2 as pc2
from pcl import PointCloud

def custommsg_to_pointcloud2(custom_msg):
    # 创建一个PointCloud2消息
    cloud_msg = PointCloud2()
    cloud_msg.header = custom_msg.header  # 使用相同的header
    cloud_msg.header.frame_id = "body"
    cloud_msg.height = 1
    cloud_msg.width = len(custom_msg.points)

    # 设置字段
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]

    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 每个点的字节数
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    cloud_msg.is_dense = True  # 无无效点

    # 构造点云数据
    points = []
    for point in custom_msg.points:
        points.append([point.x, point.y, point.z, point.reflectivity])

    # 使用pc2.create_cloud方法生成PointCloud2数据
    cloud_msg.data = pc2.create_cloud(cloud_msg.header, cloud_msg.fields, points).data

    return cloud_msg

def filter_voxel_and_remove_near_points(cloud_msg, distance_threshold=1.0, voxel_size=0.05):
    # 将PointCloud2转换为PCL点云
    pc_data = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # 将生成器转换为列表
    pc_data_list = list(pc_data)

    # 将点云数据转换为PCL点云对象
    cloud = PointCloud()
    cloud.from_list(pc_data_list)

    # 体素滤波
    voxel = cloud.make_voxel_grid_filter()
    voxel.set_leaf_size(voxel_size, voxel_size, voxel_size)  # 设置体素大小
    cloud_filtered = voxel.filter()

    # 去除近距离点云（例如，距离阈值小于1.0的点）
    pcl_points = cloud_filtered.to_list()
    filtered_points = [point for point in pcl_points if (point[0]**2 + point[1]**2 + point[2]**2)**0.5 > distance_threshold]

    # 将处理后的点云转换为PointCloud2
    filtered_cloud = PointCloud()
    filtered_cloud.from_list(filtered_points)
    filtered_cloud_msg = pc2.create_cloud_xyz32(cloud_msg.header, filtered_cloud.to_list())

    return filtered_cloud_msg

def callback(custom_msg):
    # 将CustomMsg数据转换为PointCloud2数据
    pointcloud_msg = custommsg_to_pointcloud2(custom_msg)

    # 对点云进行体素滤波和去除近距离点
    filtered_pointcloud_msg = filter_voxel_and_remove_near_points(pointcloud_msg)

    # 发布处理后的点云数据
    pub.publish(filtered_pointcloud_msg)

if __name__ == '__main__':
    rospy.init_node('custom_to_pointcloud2_node')
    
    # 创建发布器
    pub = rospy.Publisher('/livox_filtered_pointcloud', PointCloud2, queue_size=10)
    
    # 订阅CustomMsg主题
    rospy.Subscriber('/livox/lidar', CustomMsg, callback)

    rospy.spin()
