#! /usr/bin/env python
import imghdr
from pickletools import uint8
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


def transform():
    '''
    Reture the basic parameter of transform.
    '''
    src_points = np.array(
        [[1., 200.], [-239., 479.], [639., 200.], [878., 479.]], dtype="float32")
    dst_points = np.array([[1., 1.], [1., 479.], [639., 1.], [
                          639., 479.]], dtype="float32")
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    return M


class CameraNode():
    '''
    Provide a "follow_line(self, image)" function to drive along a line.
    '''

    def __init__(self, name) -> None:
        '''
        name: the name of camera, like: "/deepracer1/camera/zed_left/image_rect_color_left".

        Initialize class and define some internel parameter.
        '''
        self.cam_sub = rospy.Subscriber(name, Image, self.image_callback)
        self.ctl_pub = rospy.Publisher('cmd_akm1', Twist, queue_size=10)
        self.lower_gray = np.array([30, 250, 0])
        self.upper_gray = np.array([45, 255, 255])
        self.target_position = 170
        self.erode_kernel = np.ones((5, 5), dtype=np.uint8)
        self.dilate_kernel = np.ones((5, 5), dtype=np.uint8)
        self.twist = Twist()

    def image_callback(self, msg):
        '''
        msg: the message from publisher.

        Callback function for subscriber.
        Each time well do the "follow_line()" function.
        '''

        # Transfer Image to cv2 image.
        brige = cv_bridge.CvBridge()

        # Get frame iamge.
        frame = brige.imgmsg_to_cv2(msg, 'bgr8')
        self.follow_line(frame)

    def follow_line(self, img):
        '''
        img: the image get from camera.

        Image process and use the information to guide control.
        Will call "auto_drive()" function to publish control message.
        '''

        # Get the basic parameter for transfrom.
        M = transform()

        # Morpholigical process.
        img = cv2.warpPerspective(img, M, (640, 480), cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masked = cv2.inRange(hsv, self.lower_gray, self.upper_gray)
        eroded = cv2.erode(masked, kernel=self.erode_kernel)
        dilated = cv2.dilate(eroded, kernel=self.dilate_kernel)

        # Use dilated image to find the right way to control the vehicle.
        # self.twist.linear.x means the speed.
        # self.twist.angular.z means the angle
        contours, _ = cv2.findContours(
            dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        min_x = self.target_position
        # min_x = 640
        min_y = 480
        max_x = self.target_position
        # max_x = 0
        max_y = 0
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if y < min_y:
                min_y = y
                min_x = x
            if y > max_y:
                max_y = y
                max_x = x
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)

        h, w = masked.shape

        err = (min_x - self.target_position) / \
            2.0 + (max_x - self.target_position)

        img = cv2.circle(img, (int(min_x), 200), 3, (0, 255, 0), -1)
        img = cv2.circle(img, (int(max_x), 300), 3, (0, 255, 0), -1)
        img = cv2.circle(img, (int(err+320), 240), 3, (0, 255, 0), -1)
        img = cv2.circle(img, (int(min_x), int(min_y)), 8, (255, 0, 0), -1)
        img = cv2.circle(img, (int(max_x), int(max_y)), 8, (0, 255, 0), -1)
        # self.twist.linear.x = 1.5 - abs(float(err / 2.0) / 18) / 3
        self.twist.linear.x = 0.8
        self.twist.angular.z = -float(err / 32.0)
        # print(h,w)

        cv2.imshow("aa", masked)
        cv2.imshow("aaa", img)
        cv2.waitKey(1)

        # Publish twist data.
        # self.ctl_pub.publish(self.twist)

    def __del__(self):
        '''
        Destructor, set the speed and angular to zero.
        '''
        self.twist.linear.x = 0
        self.twist.angular.z = 0

        # Publish twist data.
        # self.ctl_pub.publish(self.twist)
        print('__del__ is working...')


if __name__ == "__main__":
    rospy.init_node('camera_node')
    left_name = "/deepracer1/camera/zed_left/image_rect_color_left"
    right_name = "/deepracer1/camera/zed_left/image_rect_color_left"
    left_camera = CameraNode(left_name)
    # right_camera = CameraNode(right_name)
    rospy.spin()
    pass
