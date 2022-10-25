import cv_bridge
import cv2 as cv
import rospy
from openvino.inference_engine import IECore
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import os


class openvino_node:
    def __init__(self) -> None:
        self.CAM_SIZE = [480, 640, 3]
        self.COCO_STOPSIGN = 13
        self.NETWORK_PATH = "/home/doin4/Electro_RaceCar/models/ssdlite_FP32"
        self.MODEL_NAME = "ssdlite_mobilenet_v2"

        rospy.init_node("openvino_cam")

        self.openvino_init()
        self.frame = np.zeros((2, *self.CAM_SIZE), dtype='uint8')
        rospy.Subscriber(
            "/deepracer1/camera/zed_left/image_rect_color_left", Image, self.image_callback_left)
        rospy.Subscriber("/deepracer1/camera/zed_right/image_rect_color_right",
                         Image, self.image_callback_right)

        self.pub = rospy.Publisher("/openvino", Bool, queue_size=5)

        rospy.spin()

    def openvino_init(self):
        self.ie = IECore()
        self.net = self.ie.read_network(
            model=os.path.join(self.NETWORK_PATH, self.MODEL_NAME+".xml"),
            weights=os.path.join(self.NETWORK_PATH, self.MODEL_NAME+".bin")
        )
        self.exec_net = self.ie.load_network(self.net, "CPU")

        self.output_layer_ir = next(iter(self.exec_net.outputs))
        self.input_layer_ir = next(iter(self.exec_net.input_info))

        # B,C,H,W = batch size, number of channels, height, width
        B, C, self.H, self.W = self.net.input_info[self.input_layer_ir].tensor_desc.dims
        self.frame_resize_input = np.zeros(
            (2, 3, self.H, self.W), dtype='uint8')
        self.STOP_cord = None

    def detect_STOP(self):
        self.result_ir = np.zeros((2, 100, 7))
        self.STOP_cord = np.zeros((2, 4))
        self.STOP_FLAG = Bool()
        self.STOP_FLAG = False
        for frame_idx in range(2):
            self.frame_resize_input[frame_idx] = np.expand_dims(
                cv.resize(self.frame[frame_idx], (self.W, self.H)).transpose(2, 0, 1), 0)

            result = self.exec_net.infer(
                inputs={self.input_layer_ir: self.frame_resize_input[frame_idx]})
            self.result_ir[frame_idx] = result[self.output_layer_ir][0, 0, :, :]
            if self.COCO_STOPSIGN in self.result_ir[frame_idx, :, 1]:
                # SIGN_idx=np.where(self.result_ir[frame_idx,:,1]==self.COCO_STOPSIGN)
                # self.STOP_cord[frame_idx] = self.result_ir[frame_idx][SIGN_idx][:,3:]
                print("STOP SIGN Detected\n")  # ,self.STOP_cord)
                self.STOP_FLAG = True
        self.pub.publish(self.STOP_FLAG)
        # if not 0 in self.STOP_cord[0] or not 0 in self.STOP_cord[1]:
        #     print("STOP SIGN Detected\n")#,self.STOP_cord)

    def image_callback_left(self, msg):
        bridge = cv_bridge.CvBridge()
        self.frame[0] = bridge.imgmsg_to_cv2(msg, 'bgr8')

    def image_callback_right(self, msg):
        bridge = cv_bridge.CvBridge()
        self.frame[1] = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.detect_STOP()


if __name__ == "__main__":
    openvino_node()
