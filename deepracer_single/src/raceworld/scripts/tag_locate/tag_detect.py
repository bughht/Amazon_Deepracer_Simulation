#! /usr/bin/env python
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import apriltag
import numpy as np

def image_callback(msg):
    detector = apriltag.Detector()
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector.detect(gray):
        tags = detector.detect(gray)
    # if not tags:
        for tag in tags:
            print()
            cv2.drawContours(frame,[np.expand_dims(tag.corners.astype('int32'),0)],-1,(0,255,0),2)
            cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            print("tag id: ", tag.tag_id)
    cv2.imshow('window', frame)
    cv2.waitKey(1)
    pass

def get_camera():
    rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left", Image, image_callback)
    pass

if __name__ == '__main__':
    rospy.init_node("get_camera")
    get_camera()
    rospy.spin()
    pass