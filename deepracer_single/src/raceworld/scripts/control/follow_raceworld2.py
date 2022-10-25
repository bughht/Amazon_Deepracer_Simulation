#! /usr/bin/env python
from math import atan2
import rospy
import time
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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


def Sigmoid_01(x, r, alpha):
    return 2*r/(1+np.exp(-alpha*x))-r


class follow_switch:
    def __init__(self) -> None:
        time.sleep(0.5)
        self.frame_left = np.zeros((480, 640, 3), dtype='uint8')
        self.frame_right = np.zeros((480, 640, 3), dtype='uint8')
        self.ang = 0.0
        self.err_sum = 0.0
        self.last_err_sum = 0.0
        self.speed = 0.0
        self.left_line = False
        self.tag = -1
        self.STOPFLAG = False
        self.lost_time = 0
        self.START_FLAG = True
        self.cmd_vel_pub = rospy.Publisher(
            "/deepracer1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)

        rospy.init_node("follow_raceworld2_node")
        rospy.Subscriber("/qrtag", Int32, self.switch_callback)
        rospy.Subscriber("/openvino", Bool, self.stop_callback)
        rospy.Subscriber(
            "/deepracer1/camera/zed_left/image_rect_color_left",
            Image, self.image_callback_left, queue_size=3)
        rospy.Subscriber(
            "/deepracer1/camera/zed_right/image_rect_color_right",
            Image, self.image_callback_right, queue_size=3)

        # rospy.Timer(rospy.Duration(0.001), self.follow_line)
        rospy.spin()

    def switch_callback(self, tag):
        tag = tag.data
        if tag != -1:
            print("TAG{}".format(tag))
            self.tag = tag
        if tag == 0:
            self.left_line = True
        elif tag == 1:
            self.left_line = True
        elif tag == 2:
            self.left_line = False
        elif tag == 3:
            self.left_line = False

    def stop_callback(self, stopflag):
        stopflag = stopflag.data
        if stopflag == True:
            print("STOP!!!!!")
            self.STOPFLAG = True

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
        akm = AckermannDriveStamped()
        if self.START_FLAG:
            self.START_FLAG = False
            akm.drive.speed = 0.5
            self.cmd_vel_pub.publish(akm)
            time.sleep(0.01)
            # akm.drive.speed = 1.8
            # self.cmd_vel_pub.publish(akm)
            return

        # print(cam_left, self.left_line)
        if self.STOPFLAG:
            print("CMD STOP")
        if self.STOPFLAG:  # and self.tag == 3:  # and self.err_sum < 0.1:
            time.sleep(0.01)
            akm.drive.acceleration = -10
            akm.drive.speed = 0
            self.cmd_vel_pub.publish(akm)
            return

        # if self.left_line:
        #     image = self.frame_right
        # else:
        #     image = self.frame_left

        image = cv2.warpPerspective(image, M, (640, 480), cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_gray = np.array([30, 200, 0])
        upper_gray = np.array([45, 255, 255])
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask_erode = cv2.erode(mask, kernel=kernel)
        # kernel = np.ones((2, 200), dtype=np.uint8)
        # mask_dilate = cv2.dilate(mask_enrode, kernel=kernel)
        contours, _ = cv2.findContours(
            # mask_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            mask_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        cnts = np.zeros((len(contours), 2))
        for cnt_idx, cnt in enumerate(contours):
            cnts[cnt_idx] = [cnt[:, :, 0].min(), cnt[:, :, 1].min()]

        cnts[:, 1] = IMG_HEIGHT-cnts[:, 1]
        cnts[:, 0] = cnts[:, 0]-IMG_WIDTH/2

        if cnts.shape[0] > 10:
            self.lost_time = 0
            bottom_k = (cnts[0, 1]-cnts[1, 1])/(cnts[0, 0]-cnts[1, 0])
            bottom_pt = np.array([cnts[0, 0]-cnts[0, 1]/bottom_k, 0])

            loc_road_vec = cnts[0, :]-bottom_pt
            rem_road_vec = cnts[-1, :]-cnts[-2, :]

            err_ang_tgt = atan2(loc_road_vec[0], loc_road_vec[1])
            err_rem_tgt = atan2(rem_road_vec[0], rem_road_vec[1])

            # left
            # if self.left_line:
            #     err_loc_tgt = bottom_pt[0] * \
            #         np.cos(np.mean([err_ang_tgt, err_rem_tgt])) - 50
            # else:
            #     err_loc_tgt = bottom_pt[0] * \
            #         np.cos(np.mean([err_ang_tgt, err_rem_tgt])) + 80
            if self.tag == -1:
                err_loc_tgt = bottom_pt[0] * \
                    np.cos(np.mean([err_ang_tgt, err_rem_tgt])) + 93.3
            elif self.tag == 0:
                err_loc_tgt = bottom_pt[0] * \
                    np.cos(np.mean([err_ang_tgt, err_rem_tgt])) - 73
            elif self.tag == 2:
                err_loc_tgt = bottom_pt[0] * \
                    np.cos(np.mean([err_ang_tgt, err_rem_tgt])) + 83
            elif self.tag == 1:
                err_loc_tgt = bottom_pt[0] * \
                    np.cos(np.mean([err_ang_tgt, err_rem_tgt])) - 35
            elif self.tag == 3:
                err_loc_tgt = bottom_pt[0] * \
                    np.cos(np.mean([err_ang_tgt, err_rem_tgt])) + 125

            err = np.array([err_ang_tgt, err_rem_tgt, err_loc_tgt])
            param = np.array([0.43, 0.252, 2.73e-3])
            # param = np.array([0.4, 0.3, 2.73e-3])

            self.err_sum = -err.dot(param)
            # self.ang = Sigmoid_01(
            #     self.err_sum**3+1.3*self.err_sum, 0.5, 5) + 0.4444360341*(self.err_sum-self.last_err_sum)  # 0.43
            # self.ang = max(-0.485, self.ang)
            # self.ang = min(0.485, self.ang)

            # self.speed = 3-(np.log(2*abs(self.ang)+1))*2.3
            if self.tag == -1:
                self.ang = Sigmoid_01(
                    self.err_sum**3+1.3*self.err_sum, 0.5, 3.84) + 0.42*(self.err_sum-self.last_err_sum)
                self.speed = 3.3-(np.log(2*abs(self.ang)+1))*2.25
                self.speed = min(self.speed, 2.5)
                # akm.drive.acceleration = 0.048
                # akm.drive.steering_angle_velocity = 2.2
                # akm.drive.acceleration = 0.4
                # akm.drive.steering_angle_velocity = 0.8
                akm.drive.speed = self.speed
                akm.drive.steering_angle = self.ang
            if self.tag == 0:
                self.speed = 3-(np.log(2*abs(self.ang)+1))*3.0
                self.ang = Sigmoid_01(
                    self.err_sum**3+1.3*self.err_sum, 0.5, 4) + 0.3*(self.err_sum-self.last_err_sum)
                akm.drive.speed = self.speed
                akm.drive.steering_angle = self.ang
            if self.tag == 2:
                self.speed = 3-(np.log(2*abs(self.ang)+1))*2.7
                # print(self.speed)
                # self.speed = min(self.speed, 2.5)
                # self.ang *= 1.02
                self.ang = Sigmoid_01(
                    self.err_sum**3+1.3*self.err_sum, 0.5, 5) + 0.4551*(self.err_sum-self.last_err_sum)
                akm.drive.speed = self.speed
                akm.drive.steering_angle = self.ang
            if self.tag == 1:
                # akm.drive.acceleration = 1
                # akm.drive.steering_angle_velocity = -1.5
                self.speed = 2.3-(np.log(2*abs(self.ang)+1))*1.5
                self.ang = Sigmoid_01(
                    self.err_sum**3+1.3*self.err_sum, 0.5, 5) + 0.43*(self.err_sum-self.last_err_sum)  # 0.366565
                # self.ang = Sigmoid_01(
                #     self.err_sum**3+1.3*self.err_sum, 0.5, 3) + 0.5*(self.err_sum-self.last_err_sum)
                # self.ang *= 0.95
                akm.drive.speed = self.speed
                akm.drive.steering_angle = self.ang
            if self.tag == 3:
                self.ang = Sigmoid_01(
                    self.err_sum**3+1.3*self.err_sum, 0.5, 2) + 0.53*(self.err_sum-self.last_err_sum)  # 0.52
                self.speed = 3.1
                # self.ang = - 0.3
                akm.drive.speed = self.speed
                akm.drive.steering_angle = self.ang

            self.last_err_sum = self.err_sum
            # twist.linear.x = 1
            # twist.angular.z = float(ang)

        else:
            self.lost_time += 1
            # self.speed = 2.5-(np.log(2*abs(self.ang)+1))*1.7
            # akm.drive.steering_angle = self.ang*1.2 \
            #     * (np.exp(-self.lost_time))
            if self.tag == -1:
                akm.drive.steering_angle = self.ang*1.2
                akm.drive.speed = 1.625
            if self.tag == 0:
                akm.drive.steering_angle = self.ang*1.8
                akm.drive.speed = 1.8
            if self.tag == 2:
                # akm.drive.steering_angle = self.ang*1.9
                akm.drive.steering_angle = self.ang * \
                    1.9*(np.exp(-self.lost_time*0.01)*0.5+0.5)
                akm.drive.speed = 1.5
                # self.speed = 0.8
                # akm.drive.steering_angle = self.ang *\
                # 1.5*np.exp(-self.lost_time)
            if self.tag == 1:
                # akm.drive.steering_angle = self.ang*1.1
                akm.drive.speed = 1.5
                # akm.drive.steering_angle = self.ang*1.05
                # self.speed = 1.0
                akm.drive.steering_angle = self.ang * \
                    1.9*np.exp(-self.lost_time*0.1)
            if self.tag == 3:
                self.speed = 3
                self.ang = -0.35
            # twist.linear.x = 1.
            # akm.drive.speed = self.speed

        # cmd_vel_pub.publish(twist)
        # rospy.loginfo("Speed{}, Angle{}".format(
            # akm.drive.speed, akm.drive.steering_angle))
        self.cmd_vel_pub.publish(akm)

        cv2.imshow("aa", mask_erode)
        cv2.imshow("aaa", image)
        cv2.waitKey(1)


if __name__ == '__main__':
    follow_switch()
