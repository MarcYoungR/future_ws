#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdometryToPoseStamped:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('odometry_to_pose_stamped_converter', anonymous=False)

        # 订阅 /odom 主题
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # 发布 /pose_stamped 主题
        self.pose_pub = rospy.Publisher('pose_stamped', PoseStamped, queue_size=10)

    def odom_callback(self, msg):
        # 创建 PoseStamped 消息
        pose_stamped = PoseStamped()

        # 从 Odometry 消息中提取并填充 PoseStamped
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # 发布 PoseStamped 消息
        self.pose_pub.publish(pose_stamped)

if __name__ == '__main__':
    try:
        # 实例化转换器并开始ROS循环
        converter = OdometryToPoseStamped()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
