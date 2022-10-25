#!/usr/bin/env python3
from math import atan2
import queue
from unittest import case
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Bool
import time

SRC_BIAS_WIDTH = 1e3
SRC_BIAS_HEIGHT = 200
IMG_WIDTH = 640
IMG_HEIGHT = 480

V_MIN = 2

src_points = np.array(
    [[0, SRC_BIAS_HEIGHT], [-SRC_BIAS_WIDTH, IMG_HEIGHT], [IMG_WIDTH, SRC_BIAS_HEIGHT], [IMG_WIDTH+SRC_BIAS_WIDTH, IMG_HEIGHT]], dtype="float32")
dst_points = np.array([[0, 0], [0, IMG_HEIGHT], [IMG_WIDTH, 0],
                      [IMG_WIDTH, IMG_HEIGHT]], dtype="float32")
M = cv2.getPerspectiveTransform(src_points, dst_points)

STOP_FLAG = False


def Sigmoid_01(x, r, alpha):
    return 2*r/(1+np.exp(-alpha*x))-r


class follow_switch:
    def __init__(self, str_id) -> None:
        time.sleep(1.5)
        self.str_id = str_id
        self.frame_left = np.zeros((480, 640, 3), dtype='uint8')
        self.frame_right = np.zeros((480, 640, 3), dtype='uint8')
        self.ang = 0.0
        self.err_sum = 0.0
        self.last_err_sum = 0.0
        self.speed = 0.0
        self.left_line = False
        self.tag = -1
        self.STOPFLAG = False
        self.cmd_vel_pub = rospy.Publisher(
            "/deepracer"+str_id+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)

        rospy.init_node("follow_switch_node")
        rospy.Subscriber(
            "/deepracer"+str_id+"/camera/zed_left/image_rect_color_left",
            Image, self.image_callback_left, queue_size=3)
        rospy.Subscriber(
            "/deepracer"+str_id+"/camera/zed_right/image_rect_color_right",
            Image, self.image_callback_right, queue_size=3)
        if str_id == "1":
            rospy.Subscriber("/deepracer/openvino", Bool, self.STOP_callback)

        # rospy.Timer(rospy.Duration(0.001), self.follow_line)
        # rospy.spin()

    def STOP_callback(self, msg):
        global STOP_FLAG
        if msg.data == True:
            STOP_FLAG = True

    def image_callback_left(self, msg):
        bridge = cv_bridge.CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_left = frame
        self.follow_line(frame)

    def image_callback_right(self, msg):
        bridge = cv_bridge.CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_right = frame

    def follow_line(self, image):
        global STOP_FLAG
        akm = AckermannDriveStamped()
        # print(cam_left, self.left_line)
        if STOP_FLAG:
            # print("CMD STOP")
            akm.drive.speed = 0
            akm.drive.steering_angle = 0
            self.cmd_vel_pub.publish(akm)
            return

        # if self.left_line:
        #     image = self.frame_right
        # else:
        #     image = self.frame_left
        # print("running")

        image = cv2.warpPerspective(image, M, (640, 480), cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_gray = np.array([30, 200, 0])
        upper_gray = np.array([45, 255, 255])
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask_erode = cv2.erode(mask, kernel=kernel)
        #kernel = np.ones((2, 200), dtype=np.uint8)
        #mask_dilate = cv2.dilate(mask_enrode, kernel=kernel)
        contours, _ = cv2.findContours(
            # mask_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            mask_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        cnts = np.zeros((len(contours), 2))
        for cnt_idx, cnt in enumerate(contours):
            cnts[cnt_idx] = [cnt[:, :, 0].min(), cnt[:, :, 1].min()]

        cnts[:, 1] = IMG_HEIGHT-cnts[:, 1]
        cnts[:, 0] = cnts[:, 0]-IMG_WIDTH/2

        if cnts.shape[0] > 10:
            bottom_k = (cnts[0, 1]-cnts[1, 1])/(cnts[0, 0]-cnts[1, 0])
            bottom_pt = np.array([cnts[0, 0]-cnts[0, 1]/bottom_k, 0])

            loc_road_vec = cnts[0, :]-bottom_pt
            rem_road_vec = cnts[-1, :]-cnts[-2, :]

            err_ang_tgt = atan2(loc_road_vec[0], loc_road_vec[1])
            err_rem_tgt = atan2(rem_road_vec[0], rem_road_vec[1])

            err_loc_tgt = bottom_pt[0] * \
                np.cos(np.mean([err_ang_tgt, err_rem_tgt]))  # + 80

            err = np.array([err_ang_tgt, err_rem_tgt, err_loc_tgt])
            param = np.array([0.40, 0.252, 2.7e-3])

            self.err_sum = -err.dot(param)
            self.ang = Sigmoid_01(
                self.err_sum**3+1.3*self.err_sum, 0.5, 5) + 0.43*(self.err_sum-self.last_err_sum)
            self.ang = max(-0.475, self.ang)
            self.ang = min(0.475, self.ang)

            self.speed = 1.5 - (np.log(2*abs(self.ang)+1))*0.7
            # if self.tag == -1:

            akm.drive.speed = self.speed
            akm.drive.steering_angle = self.ang

            self.last_err_sum = self.err_sum

        else:

            self.speed = 1.5-(np.log(2*abs(self.ang)+1))*.7

            akm.drive.speed = self.speed
            akm.drive.steering_angle = self.ang*1.25

        # cmd_vel_pub.publish(twist)
        # print(self.left_line, self.speed, self.ang)
        self.cmd_vel_pub.publish(akm)


if __name__ == '__main__':
    follow_switch("1")
    follow_switch("2")
    follow_switch("3")
    follow_switch("4")
    rospy.spin()
