import cv2
import numpy as np

def get_path():
    return "/home/doin4/Pictures/test.png"

def empty(a):
    pass

def _createTrackbar():
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 320)
    cv2.createTrackbar("Hue min", "TrackBars", 0, 179, empty)
    cv2.createTrackbar("Hue max", "TrackBars", 179, 179, empty)
    cv2.createTrackbar("Sat min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Sat max", "TrackBars", 255, 255, empty)
    cv2.createTrackbar("Val min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Val max", "TrackBars", 255, 255, empty)

def _getTrackbarPos():
    h_min = cv2.getTrackbarPos("Hue min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val max", "TrackBars")
    return h_min, h_max, s_min, s_max, v_min, v_max

if __name__ == "__main__":
    path = get_path()
    img = cv2.imread(path)
    # img = cv2.VideoCapture()
    # kernel = np.ones((20, 20))
    # img = cv2.erode(img, kernel)
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgCanny = cv2.Canny(img, 100, 100)
    _createTrackbar()

    while(1):
        h_min, h_max, s_min, s_max, v_min, v_max = _getTrackbarPos()
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        Mask = cv2.inRange(imgHSV, lower, upper)
        imgResult = cv2.bitwise_and(img, img, mask=Mask)
        cv2.imshow('img', imgResult)
        cv2.waitKey(1)