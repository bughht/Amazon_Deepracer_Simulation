#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

# robot_name = rospy.get_param("robot_name")

def set_throttle_steer(data):
    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher("/deepracer1/left_rear_wheel_velocity_controller/command", Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher("/deepracer1/right_rear_wheel_velocity_controller/command", Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher("/deepracer1/left_front_wheel_velocity_controller/command", Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher("/deepracer1/right_front_wheel_velocity_controller/command", Float64, queue_size=1)
    pub_pos_left_steering_hinge = rospy.Publisher("/deepracer1/left_steering_hinge_position_controller/command", Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher("/deepracer1/right_steering_hinge_position_controller/command", Float64, queue_size=1)

    throttle = data.drive.speed*28
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():
    global flag_move
    rospy.init_node('servo_commands', anonymous=True)
    # robot_name = rospy.get_param('~robot_name')
    rospy.Subscriber("/deepracer1/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
