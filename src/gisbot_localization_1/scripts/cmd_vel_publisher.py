#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

def main():
    #初始化一个名为 cmd_vel_publisher 的 ROS 节点
    rospy.init_node('cmd_vel_publisher')
    #创建一个名为 pub 的发布者，将消息发布到 cmd_vel 话题，消息类型为 Twist，队列长度为 10
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率10Hz
    cmd_vel_msg = Twist() #创建消息对象
    wheelbase = 0.09  # 轴距，单位：米
    track_width = 0.23  # 轮距，单位：米
    max_steering_angle = math.radians(44.9)  # 最大转向角,要和my_car的转向关节最大角对齐，单位：弧度
    while not rospy.is_shutdown():
        # 获取用户输入或其他控制逻辑来设置速度和转向角
        linear_speed = 0.2  # 线速度，单位：米/秒
        steering_angle = 0.0  # 转向角速度，单位：弧度
        # 简单的阿克曼转向几何计算
        if abs(steering_angle) > max_steering_angle:
            steering_angle = math.copysign(max_steering_angle, steering_angle)
            #inner_wheel_angle = math.atan(math.tan(steering_angle) * (wheelbase / (wheelbase / math.cos(steering_angle) - track_width / 2)))
            #outer_wheel_angle = math.atan(math.tan(steering_angle) * (wheelbase / (wheelbase / math.cos(steering_angle) + track_width / 2)))
            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = steering_angle
            pub.publish(cmd_vel_msg)
            rate.sleep() # 按照设定的 10Hz 频率休眠，以控制发布频率

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass