import cv_bridge
import cv2 as cv
import rospy
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import os


class qrtag_node:
    def __init__(self) -> None:
        self.CAM_SIZE = [480, 640, 3]

        rospy.init_node("qrtag_node")

        self.qrdetect_init()
        self.frame = np.zeros((2, *self.CAM_SIZE), dtype='uint8')
        rospy.Subscriber(
            "/deepracer1/camera/zed_left/image_rect_color_left",
            Image,
            self.image_callback_left
        )
        rospy.Subscriber(
            "/deepracer1/camera/zed_right/image_rect_color_right",
            Image,
            self.image_callback_right
        )

        self.pub = rospy.Publisher("/qrtag", Int32, queue_size=5)
        # self.rate = rospy.Rate(20)

        rospy.spin()

    def qrdetect_init(self):
        self.qr_detector = apriltag.Detector()

    def detect_tag(self):
        self.QR_CODE = Int32()
        self.QR_CODE = -1
        self.QR_TAGS = [None, None]
        for frame_idx in range(2):
            self.frame_gray = cv.cvtColor(
                self.frame[frame_idx], cv.COLOR_BGR2GRAY)
            self.QR_TAGS[frame_idx] = self.qr_detector.detect(self.frame_gray)
            if self.QR_TAGS[frame_idx]:
                for tag in self.QR_TAGS[frame_idx]:
                    self.QR_CODE = tag.tag_id
                    print("TAG DETECTED ID {}\n".format(self.QR_CODE))

        self.pub.publish(self.QR_CODE)
        # if not 0 in self.STOP_cord[0] or not 0 in self.STOP_cord[1]:
        #     print("STOP SIGN Detected\n")#,self.STOP_cord)

    def image_callback_left(self, msg):
        bridge = cv_bridge.CvBridge()
        self.frame[0] = bridge.imgmsg_to_cv2(msg, 'bgr8')

    def image_callback_right(self, msg):
        bridge = cv_bridge.CvBridge()
        self.frame[1] = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.detect_tag()


if __name__ == "__main__":
    qrtag_node()
