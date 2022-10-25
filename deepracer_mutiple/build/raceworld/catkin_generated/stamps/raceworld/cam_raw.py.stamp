#!/usr/bin/python3
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image,CompressedImage


class Camera:
    def __init__(self) -> None:
        self.frame_left=None
        self.frame_right=None
        rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, self.image_callback_left)
        rospy.Subscriber("/deepracer1/camera/zed_right/image_rect_color_right", Image, self.image_callback_right)
        self.plot()

    def image_callback_left(self,msg):
        bridge = cv_bridge.CvBridge()
        self.frame_left = bridge.imgmsg_to_cv2(msg, 'bgr8')
        print(self.frame_left.dtype)

    def image_callback_right(self,msg):
        bridge = cv_bridge.CvBridge()
        self.frame_right = bridge.imgmsg_to_cv2(msg, 'bgr8')
        
    def plot(self):
        while True:
            if self.frame_right is not None:
                cv2.imshow('window_right', self.frame_right)
                print(self.frame_right.shape)
            if self.frame_left is not None:
                cv2.imshow('window_left', self.frame_left)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("get_camera")
    Camera()
    rospy.spin()
    pass