#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist  # 导入 Twist 消息类型
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

def set_throttle_steer(data):
    throttle = data.drive.speed * 33.333  # 转换单位
    steer = data.drive.steering_angle

    # 发布轮子的速度和转向角度
    pub_vel_back_left_wheel.publish(throttle)
    pub_vel_back_right_wheel.publish(throttle)
    pub_vel_front_left_wheel.publish(throttle)
    pub_vel_front_right_wheel.publish(throttle)

    pub_pos_front_left_wheel_joint_position.publish(steer)
    pub_pos_front_right_wheel_joint_position.publish(steer)

def cmd_vel_callback(msg):
    # 这里处理 /cmd_vel 发布的 Twist 消息
    throttle = msg.linear.x * 33.333  # 根据线速度计算转速
    steer = msg.angular.z  # 根据角速度计算转向角度

    # 发布速度和转向命令
    pub_vel_back_left_wheel.publish(throttle)
    pub_vel_back_right_wheel.publish(throttle)
    pub_vel_front_left_wheel.publish(throttle)
    pub_vel_front_right_wheel.publish(throttle)

    pub_pos_front_left_wheel_joint_position.publish(steer)
    pub_pos_front_right_wheel_joint_position.publish(steer)

def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)#注册节点

    # 初始化发布器，只会创建一次
    global pub_vel_back_left_wheel, pub_vel_back_right_wheel, pub_vel_front_left_wheel, pub_vel_front_right_wheel
    global pub_pos_front_left_wheel_joint_position, pub_pos_front_right_wheel_joint_position
    # 创建发布者 topic_name/消息类型/队列长度
    pub_vel_back_left_wheel = rospy.Publisher('/my_car/back_left_wheel_velocity_controller/command', Float64, queue_size=10)
    pub_vel_back_right_wheel = rospy.Publisher('/my_car/back_right_wheel_velocity_controller/command', Float64, queue_size=10)
    pub_vel_front_left_wheel = rospy.Publisher('/my_car/front_left_wheel_velocity_controller/command', Float64, queue_size=10)
    pub_vel_front_right_wheel = rospy.Publisher('/my_car/front_right_wheel_velocity_controller/command', Float64, queue_size=10)
    pub_pos_front_left_wheel_joint_position = rospy.Publisher('/my_car/front_left_wheel_joint_position_controller/command', Float64, queue_size=10)
    pub_pos_front_right_wheel_joint_position = rospy.Publisher('/my_car/front_right_wheel_joint_position_controller/command', Float64, queue_size=10)

    # 订阅 /ackermann_cmd_mux/output 来控制车辆的速度和转向
    #rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # 订阅 /cmd_vel 来接收速度控制命令，cmd_vel_callback是回调函数处理信息
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback,queue_size=10)

    rospy.spin() #自锁函数，循环查询队列等待信息数据，一旦有了数据立刻回到毁掉函数

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
