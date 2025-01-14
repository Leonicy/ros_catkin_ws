#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

def main():
    # 初始化一个名为 cmd_vel_publisher 的 ROS 节点
    rospy.init_node('cmd_vel_publisher')
    
    # 创建一个名为 pub 的发布者，将消息发布到 cmd_vel 话题，消息类型为 Twist，队列长度为 10
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 发布频率10Hz
    cmd_vel_msg = Twist()  # 创建消息对象
    
    # 获取控制参数
    wheelbase = rospy.get_param('~wheelbase', 0.09)  # 轴距，单位：米
    track_width = rospy.get_param('~track_width', 0.23)  # 轮距，单位：米
    max_steering_angle = rospy.get_param('~max_steering_angle', math.radians(44.9))  # 最大转向角,单位：弧度
    
    # 获取最大速度和最大转向角度
    max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # 最大线速度（米/秒）
    
    while not rospy.is_shutdown():
        # 获取用户输入或其他控制逻辑来设置速度和转向角
        linear_speed = rospy.get_param('~linear_speed', 0.2)  # 线速度，单位：米/秒
        steering_angle = rospy.get_param('~steering_angle', 0.1)  # 转向角速度，单位：弧度
        
        # 限制线速度和转向角在最大值范围内
        linear_speed = min(linear_speed, max_linear_speed)  # 确保线速度不会超出最大值
        steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle))  # 转向角度在最大范围内
        
        # 发布 cmd_vel 消息
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = steering_angle
        
        rospy.loginfo("Publishing: linear_speed = %f, steering_angle = %f", linear_speed, steering_angle)
        
        try:
            pub.publish(cmd_vel_msg)  # 发布消息
        except rospy.ROSInterruptException:
            rospy.logerr("Failed to publish cmd_vel message")
        
        rate.sleep()  # 按照设定的 10Hz 频率休眠，以控制发布频率

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
