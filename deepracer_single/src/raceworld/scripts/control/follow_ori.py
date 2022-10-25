#! /usr/bin/env python
import imghdr
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


def set_roi_forward(h, w, mask):
    search_top = int(3 * h / 4)
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass


err = 0


def follow_line(image, color):
    global err
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([26, 43, 46])
    upper_yellow = numpy.array([34, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w = mask.shape
    # print(h,w)
    mask = set_roi_forward(h, w, mask)
    M = cv2.moments(mask)

    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])+235
        cy = int(M['m01'] / M['m00'])
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        # BEGIN CONTROL
        err = cx - w / 2 - 500 + cy*0.4
        print(err, cx, cy)
        # print(M, err, cx, cy)
        twist = Twist()
        if abs(err) > 130:
            twist.linear.x = 0.53
            twist.angular.z = -float(err / 2.0) / 46
        elif abs(err) > 55:
            twist.linear.x = .98
            twist.angular.z = -float(err / 2.0) / 42
        else:
            twist.linear.x = 1.35
            twist.angular.z = -float(err / 2.0) / 38

        cmd_vel_pub.publish(twist)
    cv2.imshow("window1", image)
    cv2.waitKey(1)
    pass


def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame, "yellow")
    pass


if __name__ == '__main__':
    rospy.init_node("follower")
    rospy.Subscriber(
        "/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
    pass
