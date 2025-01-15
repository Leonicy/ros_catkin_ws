#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move

    pub_vel_back_left_wheel = rospy.Publisher('/my_car/back_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_back_right_wheel = rospy.Publisher('/my_car/back_right_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_front_left_wheel = rospy.Publisher('/my_car/front_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_front_right_wheel = rospy.Publisher('/my_car/front_right_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_pos_front_left_wheel_joint_position = rospy.Publisher('/my_car/front_left_wheel_joint_position_controller/command', Float64, queue_size=1)
    pub_pos_front_right_wheel_joint_position = rospy.Publisher('/my_car/front_right_wheel_joint_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed*33.333
    steer = data.drive.steering_angle

    pub_vel_back_left_wheel.publish(throttle)
    pub_vel_back_right_wheel.publish(throttle)
    pub_vel_front_left_wheel.publish(throttle)
    pub_vel_front_right_wheel.publish(throttle)
    pub_pos_front_left_wheel_joint_position.publish(steer)
    pub_pos_front_right_wheel_joint_position.publish(steer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
