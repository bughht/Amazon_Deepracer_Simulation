#!/usr/bin/env python3
import imghdr
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


src_points = np.array(
    [[1., 200.], [-239., 479.], [639., 200.], [878., 479.]], dtype="float32")
dst_points = np.array([[1., 1.], [1., 479.], [639., 1.],
                      [639., 479.]], dtype="float32")
M = cv2.getPerspectiveTransform(src_points, dst_points)


def follow_line(image, color):
    cmd_vel_pub = rospy.Publisher("cmd_akm1", Twist, queue_size=10)

    target_position = 190

    image = cv2.warpPerspective(image, M, (640, 480), cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_gray = np.array([30, 200, 0])
    upper_gray = np.array([45, 255, 255])
    mask = cv2.inRange(hsv, lower_gray, upper_gray)
    kernel = np.ones((2, 2), dtype=np.uint8)
    mask_enrode = cv2.erode(mask, kernel=kernel)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask_dilate = cv2.dilate(mask_enrode, kernel=kernel)
    contours, _ = cv2.findContours(
        mask_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cnts = np.zeros((len(contours), 4))
    for cnt_idx, cnt in enumerate(contours):
        cnts[cnt_idx] = [cnt[:, :, 0].min(), cnt[:, :, 1].min(),
                         cnt[:, :, 0].max(), cnt[:, :, 1].max()]
    # print(contours[0].shape)
    print(cnts)

    min_x = target_position
    min_y = 480
    max_x = target_position
    max_y = 0
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if y < min_y:
            min_y = y
            min_x = x
        if y > max_y:
            max_y = y
            max_x = x
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)

    h, w = mask.shape

    err = (min_x - target_position) / 2.0 + (max_x - target_position)
    # err = np.sign(err)*(abs(err) + y / 30)

    img = cv2.circle(image, (int(min_x), 200), 3, (0, 255, 0), -1)
    img = cv2.circle(image, (int(max_x), 300), 3, (0, 255, 0), -1)
    img = cv2.circle(image, (int(err+320), 240), 3, (0, 255, 0), -1)
    img = cv2.circle(image, (int(min_x), int(min_y)), 15, (22, 255, 0), -1)
    img = cv2.circle(image, (int(max_x), int(max_y)), 17, (220, 255, 0), -1)
    img = cv2.circle(image, (target_position, 240), 5, (20, 255, 0), -1)
    twist = Twist()
    # twist.linear.x = 1.5 - abs(float(err / 2.0) / 10) / 3
    twist.linear.x = 1.0
    # if twist.linear.x < 0.6:
    #     twist.linear.x = 0.6
    twist.angular.z = -float(err / 40.0)  
    print(err, twist.linear.x, twist.angular.z)
    cmd_vel_pub.publish(twist)
    # print(h,w)

    cv2.imshow("aa", mask)
    cv2.imshow("aaa", image)
    cv2.waitKey(1)


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
