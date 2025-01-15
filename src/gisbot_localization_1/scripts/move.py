#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_vehicle():
    rospy.init_node('move_vehicle', anonymous=True)
    pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # 创建 Twist 消息
    move_cmd = Twist()
    move_cmd.linear.x = 1.0  # 设置前进速度
    move_cmd.angular.z = 0.0  # 设置转弯速度

    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_vehicle()
    except rospy.ROSInterruptException:
        pass
