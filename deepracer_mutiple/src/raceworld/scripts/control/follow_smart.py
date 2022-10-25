#! /usr/bin/env python
from math import atan2
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

SRC_BIAS_WIDTH = 1e3
SRC_BIAS_HEIGHT = 200
IMG_WIDTH = 640
IMG_HEIGHT = 480

V_MIN = 1.35


# Transform parameter.
src_points = np.array(
    [[0, SRC_BIAS_HEIGHT], [-SRC_BIAS_WIDTH, IMG_HEIGHT], [IMG_WIDTH, SRC_BIAS_HEIGHT], [IMG_WIDTH+SRC_BIAS_WIDTH, IMG_HEIGHT]], dtype="float32")
dst_points = np.array([[0, 0], [0, IMG_HEIGHT], [IMG_WIDTH, 0],
                      [IMG_WIDTH, IMG_HEIGHT]], dtype="float32")
M = cv2.getPerspectiveTransform(src_points, dst_points)

ang = 0.0

# Accumulated error.
pre_err_sum = 0.0


def Sigmoid_01(x, r, alpha):
    return 2*r/(1+np.exp(-alpha*x))-r


def follow_line(image):
    global ang
    global pre_err_sum
    # Use Acker to publish speed and angle.
    cmd_vel_pub = rospy.Publisher(
        "/deepracer1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)
    # twist = Twist()
    akm = AckermannDriveStamped()

    # Morphological process.
    image = cv2.warpPerspective(image, M, (640, 480), cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_gray = np.array([30, 200, 0])
    upper_gray = np.array([45, 255, 255])
    mask = cv2.inRange(hsv, lower_gray, upper_gray)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask_erode = cv2.erode(mask, kernel=kernel)
    #kernel = np.ones((2, 200), dtype=np.uint8)
    #mask_dilate = cv2.dilate(mask_enrode, kernel=kernel)

    # Use function in cv2 to find contours.
    # Return three values: Input img array; Contour[list]; Hierarchy.
    contours, _ = cv2.findContours(
        # mask_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        mask_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Initialize cnts.
    cnts = np.zeros((len(contours), 2))

    # Go through all contours and get the upper-left point of all contours.
    # cnt is a three dim vector, the last one store the x and y information.
    for cnt_idx, cnt in enumerate(contours):
        cnts[cnt_idx] = [cnt[:, :, 0].min(), cnt[:, :, 1].min()]

    # Modify all points, x -> 480-x; y -> y-640/2.
    # To meed the transform above.
    cnts[:, 1] = IMG_HEIGHT-cnts[:, 1]
    cnts[:, 0] = cnts[:, 0]-IMG_WIDTH/2

    # If there are more than 10 points(namely more than 10 yellow dash in the car sight).
    if cnts.shape[0] > 10:

        # The slop of top two point.(botton line)
        bottom_k = (cnts[0, 1]-cnts[1, 1])/(cnts[0, 0]-cnts[1, 0])
        bottom_pt = np.array([cnts[0, 0]-cnts[0, 1]/bottom_k, 0])

        # top_k = (cnts[-1, 1]-cnts[])

        # Use know parameter to guess local road.
        loc_road_vec = cnts[0, :]-bottom_pt
        rem_road_vec = cnts[-1, :]-cnts[-2, :]

        # Calculate error value.
        err_ang_tgt = atan2(loc_road_vec[0], loc_road_vec[1])
        err_rem_tgt = atan2(rem_road_vec[0], rem_road_vec[1])
        # left
        err_loc_tgt = bottom_pt[0] * \
            np.cos(np.mean([err_ang_tgt, err_rem_tgt]))  # + 100
        # right
        # err_loc_tgt = bottom_pt[0] - 83

        err = np.array([err_ang_tgt, err_rem_tgt, err_loc_tgt])
        # if np.abs(err[1]) > -0.8:
        #     param = np.array([0.77, 0.8, 5.0e-3])
        #     akm.drive.speed = 1.8
        # elif err[1] < -0.5:
        #     param = np.array([0.77, 0.72, 5.0e-3])
        # else:
        # Hyper-parameter.
        param = np.array([0.4, 0.25, 2.7e-3])
        # param = np.array([0.40, 0.25, 2.7e-3])

        # Fusion all the value and parameter.
        err_sum = -err.dot(param)
        print(err_sum)
        # ang *= 1+1.7*(IMG_HEIGHT-cnts[-1, 1])/800

        # PID: P and D
        ang = Sigmoid_01(err_sum, 0.5, 2.0)  # - 0.1 * (err_sum-pre_err_sum)
        # ang = np.sign(ang)*(abs(ang)+(IMG_HEIGHT-cnts[-1, 1])*8.0e-4)
        ang = max(-0.475, ang)
        ang = min(0.475, ang)

        # Compute the final speed and angle,(variable speed)
        akm.drive.speed = 3.3 - (np.log(2*abs(ang)+1)) * 1.7
        # akm.drive.speed = max(4.5 - abs(err_rem_tgt) * 4.8-abs(err_loc_tgt)*5e-2, V_MIN)
        # akm.drive.acceleration = 0.6
        akm.drive.steering_angle = ang
        print(err, akm.drive.speed, ang)
        # twist.linear.x = 1
        # twist.angular.z = float(ang)
        pre_err_sum = err_sum

    # Compensent for lossing sight.
    else:
        akm.drive.speed = 3.3 - (np.log(2*abs(ang)+1)) * 1.7
        akm.drive.steering_angle = ang * 1.1

        # twist.linear.x = 1.

    # cmd_vel_pub.publish(twist)
    cmd_vel_pub.publish(akm)

    cv2.imshow("aa", mask)
    cv2.imshow("aaa", image)
    cv2.waitKey(1)


def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame)
    pass


if __name__ == '__main__':
    rospy.init_node("follower")
    rospy.Subscriber(
        "/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    # "/deepracer1/camera/zed_right/image_rect_color_right", Image, image_callback)
    rospy.spin()
    pass
