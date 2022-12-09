import cv2
from cv2 import inRange
import numpy as np
import threading
import time

cap = cv2.VideoCapture(0)
circle_array = []
detected_circlesblue = None
detected_circlesred = None
detected_circlesyellow = None
img = []
hsv_img = []

def blue_mask():
    global detected_circlesblue
    global hsv_img
    global img
    while True:
        low_blue = np.array([93, 91, 160])
        high_blue = np.array([163, 255, 255])
        maskB = cv2.inRange(hsv_img, low_blue, high_blue)
        kernal = np.ones((3, 3), np.uint8)
        blue_mask = cv2.dilate(maskB, kernal)
        blue = cv2.bitwise_and(img, img, mask=blue_mask)
        #cv2.imshow("blue mask",blue)
        grayblue = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
        gray_blurredblue = cv2.blur(grayblue, (3, 3))
        detected_circlesblue = cv2.HoughCircles(gray_blurredblue,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
        if detected_circlesred is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circlesred = np.uint16(np.around(detected_circlesred))
def red_mask ():
    global detected_circlesred
    global hsv_img
    global img
    while True :
        low_red = np.array([139, 57, 150])
        high_red = np.array([179, 163, 255])
        maskR = cv2.inRange(hsv_img, low_red, high_red)
        kernal = np.ones((3, 3), np.uint8)
        red_mask = cv2.dilate(maskR, kernal)
        red = cv2.bitwise_and(img, img, mask=red_mask)
        #cv2.imshow("red mask",red)
        grayred = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        gray_blurredred = cv2.blur(grayred, (3, 3))
        detected_circlesred = cv2.HoughCircles(gray_blurredred,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
        if detected_circlesred is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circlesred = np.uint16(np.around(detected_circlesred))
def yellow_mask ():
    global detected_circlesyellow
    global hsv_img
    global img
    while True:
        low_yellow = np.array([11,153,177])
        high_yellow = np.array([179,255,255])
        maskY = cv2.inRange(hsv_img, low_yellow, high_yellow)
        kernal = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.dilate(maskY, kernal)
        yellow = cv2.bitwise_and(img,img, mask=yellow_mask)
        #cv2.imshow("yellow mask",yellow)
        grayyellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
        gray_blurredyellow = cv2.blur(grayyellow, (3, 3))
        detected_circlesyellow = cv2.HoughCircles(gray_blurredyellow,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
        if detected_circlesyellow is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circlesyellow = np.uint16(np.around(detected_circlesyellow))

def image_read ():
    global hsv_img
    global img
    while True:
        success, img = cap.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break

blueThread = threading.Thread(target=blue_mask)
redThread = threading.Thread(target=red_mask)
yellowThread = threading.Thread(target=yellow_mask)
imageThread = threading.Thread(target=image_read)

imageThread.start()
time.sleep(2)
blueThread.start()
redThread.start()
yellowThread.start()

while True:
    print("blue",detected_circlesblue)
    print("red",detected_circlesred)
    print("yellow", detected_circlesyellow)
    if detected_circlesblue is not None:
        for pt in detected_circlesblue[0, :]:
            aBlue, bBlue, rBlue = pt[0], pt[1], pt[2]
        if detected_circlesred is not None:
            for pt in detected_circlesred[0, :]:
                aRed, bRed, rRed = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                #cv2.circle(img, (aRed, bRed), rRed, (0, 255, 0), 2)
            if detected_circlesyellow is not None:
                for pt in detected_circlesyellow[0, :]:
                    aYellow, bYellow, rYellow = pt[0], pt[1], pt[2]
                    
                    # Draw the circumference of the circle.
                    #cv2.circle(img, (aYellow, bYellow), rYellow, (255, 255, 0), 2)

                    #Draw a small circle (of radius 1) to show the center.
                    cv2.circle(img, (aYellow, bYellow), 1, (0, 0, 255), 3)
                    adiff1 = float(aBlue - aRed)
                    
                    bdiff1 = float(bBlue - bRed)

                    if (abs(adiff1)<=5) and (abs(bdiff1)<=5):
                        #if (int(aRed)in range(160,480)) and (int(bRed)in range(120,360)):

                        cv2.circle(img,(aRed,bRed),rRed,(0,255,0),2)

                        cv2.circle(img,(aRed,bRed), 1,(0,255,0),4)
                    cv2.imshow("image", img)