#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from ackermann_msgs.msg import AckermannDriveStamped

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
settings = termios.tcgetattr(sys.stdin)

def pub_cmd():
    index = 1
    rospy.init_node("pub_cmd")
    # rospy.set_param("Car Index", 1)
    pub = rospy.Publisher("/deepracer"+str(index)+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    akm = AckermannDriveStamped()
    
    while 1:
        x=0
        a=0

        key = getKey()
        rospy.loginfo("键盘已录入：%s", key)
        if key == 'w':
            x=0.3
            a=0
        elif key == 's':
            x=-0.3
            a=0
        elif key == 'a':
            x=0.3
            a=0.7
        elif key == 'd':
            x=0.3
            a=-0.7
        elif key == 'x':
            x=0
            a=0
        elif key == 'o':
            break
        else:
            continue

        # akm.drive.speed = x*6.97674
        akm.drive.speed = x
        akm.drive.steering_angle = a*0.7
        # akm.drive.jerk = 2

        pub.publish(akm)

if __name__=="__main__":
    pub_cmd()
    pass
