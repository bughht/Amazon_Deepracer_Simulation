import rospy, cv_bridge
import cv2 as cv
import sys, select, termios, tty
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image,CompressedImage
import numpy as np 
from openvino.inference_engine import IECore
import apriltag

CAM_SIZE=[480,640,3]
COCO_STOPSIGN=13

class Auto_Cam:
    def __init__(self) -> None:
        self.openvino_init()
        self.qrdetect_init()
        self.frame= np.zeros((2,*CAM_SIZE),dtype="uint8")
        self.settings=termios.tcgetattr(sys.stdin)
        rospy.Subscriber("/deepracer1/camera/zed_left/image_rect_color_left",Image,self.image_callback_left)
        rospy.Subscriber("/deepracer1/camera/zed_right/image_rect_color_right",Image,self.image_callback_right)

    def openvino_init(self): 
        self.ie=IECore()
        self.net = self.ie.read_network(model="/home/bughht/deepracer/Electro_RaceCar/models/ssdlite_FP32/ssdlite_mobilenet_v2.xml",weights='/home/bughht/deepracer/Electro_RaceCar/models/ssdlite_FP32/ssdlite_mobilenet_v2.bin')
        self.exec_net = self.ie.load_network(self.net, "CPU")

        self.output_layer_ir = next(iter(self.exec_net.outputs))
        self.input_layer_ir = next(iter(self.exec_net.input_info))

        # B,C,H,W = batch size, number of channels, height, width
        B, C, self.H, self.W = self.net.input_info[self.input_layer_ir].tensor_desc.dims       
        self.frame_resize_input=np.zeros((2,3,self.H,self.W),dtype='uint8')
        self.STOP_cord=None
        
    def qrdetect_init(self):
        self.qr_detector=apriltag.Detector()
        self.QR_TAGS=[None,None]

    def detect_QR_STOP(self):
        self.result_ir=np.zeros((2,100,7))
        self.STOP_cord=np.zeros((2,4))
        self.QR_TAG= None
        self.STOP_FLAG=False
        for frame_idx in range(2):
            self.frame_gray=cv.cvtColor(self.frame[frame_idx],cv.COLOR_BGR2GRAY)
            self.QR_TAGS[frame_idx]=self.qr_detector.detect(self.frame_gray)
            if self.QR_TAGS[frame_idx]:
                for tag in self.QR_TAGS[frame_idx]:
                    print("TAG ID {}\n".format(tag.tag_id))

            self.frame_resize_input[frame_idx]=np.expand_dims(cv.resize(self.frame[frame_idx],(self.W,self.H)).transpose(2,0,1),0)

            result=self.exec_net.infer(inputs={self.input_layer_ir:self.frame_resize_input[frame_idx]})
            self.result_ir[frame_idx]=result[self.output_layer_ir][0,0,:,:]
            if COCO_STOPSIGN in self.result_ir[frame_idx,:,1]:
                SIGN_idx=np.where(self.result_ir[frame_idx,:,1]==COCO_STOPSIGN)
                self.STOP_cord[frame_idx] = self.result_ir[frame_idx][SIGN_idx][:,3:]
                self.STOP_FLAG=True

        if not 0 in self.STOP_cord[0] or not 0 in self.STOP_cord[1]:
            print("STOP SIGN Detected\n")#,self.STOP_cord)

        
        # if 13 in self.result_ir[:,:,1]:
        #     SIGN_idx=np.where(self.result_ir[:,:,1]==COCO_STOPSIGN)
        #     self.STOP_cord=self.result_ir[SIGN_idx][:,3:]
        #     print("STOP SIGN Detected",self.result_ir[SIGN_idx][:,2],self.STOP_cord)

    def image_callback_left(self,msg):
        bridge = cv_bridge.CvBridge()
        self.frame[0]=bridge.imgmsg_to_cv2(msg,'bgr8')
        self.detect_QR_STOP()

    def image_callback_right(self,msg):
        bridge = cv_bridge.CvBridge()
        self.frame[1]=bridge.imgmsg_to_cv2(msg,'bgr8')
        self.combine_plot()

    def combine_plot(self):
        self.frame_aug=self.frame.copy()
        for frame_idx in range(2):
            if self.QR_TAGS[frame_idx]:
                for tag in self.QR_TAGS[frame_idx]:
                    self.frame_aug[frame_idx]=cv.drawContours(self.frame_aug[frame_idx],[np.expand_dims(tag.corners.astype('int32'),0)],-1,(0,0,255),3)
            if self.STOP_cord[frame_idx][0]!=0: 
                top_left=tuple((self.STOP_cord[frame_idx,0:2]*CAM_SIZE[:2][::-1]).astype(int))
                bottom_right=tuple((self.STOP_cord[frame_idx,2:4]*CAM_SIZE[:2][::-1]).astype(int))
                # print(self.STOP_cord[frame_idx,0:2]-self.STOP_cord[frame_idx,2:4])
                cv.rectangle(self.frame_aug[frame_idx],top_left,bottom_right,(0,255,255),3)
                
        # self.frame_STOP=``
        self.frame_c=np.concatenate((self.frame_aug[0],self.frame_aug[1]),axis=1)
        cv.imshow("Cam",self.frame_c)
        cv.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("Auto_Cam")
    Auto_Cam()
    rospy.spin()
    pass
