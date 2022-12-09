import cv2
from cv2 import inRange
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #masking blue
    low_blue = np.array([74, 123, 104])
    high_blue = np.array([168, 255, 255])
    maskB = cv2.inRange(hsv_img, low_blue, high_blue)
    kernal = np.ones((3, 3), np.uint8)
    blue_mask = cv2.dilate(maskB, kernal)
    blue = cv2.bitwise_and(img, img, mask=blue_mask)
    #cv2.imshow("blue mask",blue)
    grayblue = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
    gray_blurredblue = cv2.blur(grayblue, (3, 3))


    #masking red
    low_red = np.array([138, 102, 85])
    high_red = np.array([179, 163, 255])
    maskR = cv2.inRange(hsv_img, low_red, high_red)
    red_mask = cv2.dilate(maskR, kernal)
    red = cv2.bitwise_and(img, img, mask=red_mask)
    #cv2.imshow("red mask",red)
    grayred = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    gray_blurredred = cv2.blur(grayred, (3, 3))

    #masking yellow
    low_yellow = np.array([2,117,0])
    high_yellow = np.array([99,255,255])
    maskY = cv2.inRange(hsv_img, low_yellow, high_yellow)
    yellow_mask = cv2.dilate(maskY, kernal)
    yellow = cv2.bitwise_and(img,img, mask=yellow_mask)
    #cv2.imshow("yellow mask",yellow)
    grayyellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
    gray_blurredyellow = cv2.blur(grayyellow, (3, 3))

    #blue circle
    detected_circlesblue = cv2.HoughCircles(gray_blurredblue,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
    if detected_circlesblue is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circlesblue = np.uint16(np.around(detected_circlesblue))

        for pt in detected_circlesblue[0, :]:
            aBlue, bBlue, rBlue = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            #cv2.circle(img, (aBlue, bBlue), rBlue, (255, 0, 0), 2)

        #red circle
        detected_circlesred = cv2.HoughCircles(gray_blurredred,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
        if detected_circlesred is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circlesred = np.uint16(np.around(detected_circlesred))
            
            for pt in detected_circlesred[0, :]:
                aRed, bRed, rRed = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                #cv2.circle(img, (aRed, bRed), rRed, (0, 255, 0), 2)

            #yellow circle
            detected_circlesyellow = cv2.HoughCircles(gray_blurredyellow,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
            if detected_circlesyellow is not None:
                
                # Convert the circle parameters a, b and r to integers.
                detected_circlesyellow = np.uint16(np.around(detected_circlesyellow))

                for pt in detected_circlesyellow[0, :]:
                    aYellow, bYellow, rYellow = pt[0], pt[1], pt[2]
                    
                    # Draw the circumference of the circle.
                    #cv2.circle(img, (aYellow, bYellow), rYellow, (255, 255, 0), 2)

                    # Draw a small circle (of radius 1) to show the center.
                    #cv2.circle(img, (aYellow, bYellow), 1, (0, 0, 255), 3)
                    adiff1 = int(aBlue - aRed)
                    
                    bdiff1 = int(bBlue - bRed)

                    if (abs(adiff1)<=10) and (abs(bdiff1)<=10):
                        #if (int(aRed)in range(160,480)) and (int(bRed)in range(120,360)):

                        cv2.circle(img,(aRed,bRed),rRed,(0,255,0),2)

                        cv2.circle(img,(aRed,bRed), 1,(0,255,0),4)
                    
                   
                    cv2.imshow("Detected Circle", img)



    if cv2.waitKey(1) & 0xff == ord('q'):
        break
