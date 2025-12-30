#!/usr/bin/env python
# 订阅livox_ros_driver2/CustomMsg的原始数据生成PointCloud2点云数据
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import sensor_msgs.point_cloud2 as pc2

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

def callback(custom_msg):
    # 将CustomMsg数据转换为PointCloud2数据
    pointcloud_msg = custommsg_to_pointcloud2(custom_msg)
    pub.publish(pointcloud_msg)

if __name__ == '__main__':
    rospy.init_node('custom_to_pointcloud2_node')
    
    # 创建发布器
    pub = rospy.Publisher('/livox_pointcloud', PointCloud2, queue_size=10)
    
    # 订阅CustomMsg主题
    rospy.Subscriber('/livox/lidar', CustomMsg, callback)

    rospy.spin()
