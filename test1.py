import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

low_red = np.array([0, 30, 130])
high_red = np.array([20, 180, 255])

low_blue = np.array([60,110,100])
high_blue = np.array([190,255,255])

low_yellow = np.array([5,110,14])
high_yellow = np.array([80,200,255])

erosion = 2
dilation = 6
_,frame = cap.read()
width,height,_ = frame.shape

while True:
    _,frame = cap.read()

    frame_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    mask = cv.inRange(frame_hsv,low_red,high_red)
    mask_blue = cv.inRange(frame_hsv,low_blue,high_blue)
    mask_yellow = cv.inRange(frame_hsv, low_yellow, high_yellow)

    kernel = np.ones(shape=(3,3))
    mask_yellow = cv.erode(mask_yellow,kernel,erosion)
    mask_yellow = cv.dilate(mask_yellow,kernel,dilation)
    mask = cv.erode(mask,kernel,erosion)
    mask = cv.dilate(mask,kernel,dilation)  
    mask_blue= cv.erode(mask_blue,kernel,erosion)
    mask_blue = cv.dilate(mask_blue,kernel,dilation)

    mask1 = cv.bitwise_or(mask,mask_blue)
    mask1 = cv.bitwise_or(mask1,mask_yellow)

    mask_blue = np.zeros(shape=(width,height))
    mask_blue[frame[...,0] > 100] = 255
    mask_blue[frame[...,1] > 200] = 0
    mask_blue[frame[...,2] > 200] = 0

    # cv.imshow("mask", mask1)
    cv.imshow("mask_blue ",mask_blue)
    # cv.imshow("mask_red",   mask)
    # cv.imshow("mask_yellow",mask_yellow)
    if cv.waitKey(10) & 0xFF == 27:
        break
cv.destroyAllWindows()
cap.release()   
