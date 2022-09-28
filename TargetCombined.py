import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while True:
    success, img = cap.read()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    KERNAL = np.ones((3,3),np.uint8)

    #masking blue
    low_blue = np.array([93, 68, 158])
    high_blue = np.array([170, 255, 255])
    maskB = cv2.inRange(hsv_img, low_blue, high_blue)

    
    #masking red
    low_red = np.array([139, 57, 150])
    high_red = np.array([179, 163, 255])
    maskR = cv2.inRange(hsv_img, low_red, high_red)

    red_mask = cv2.dilate(maskR, KERNAL)
    red = cv2.bitwise_and(img, img, mask=red_mask)
    #cv2.imshow("red mask",red)
    grayred = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    gray_blurredred = cv2.blur(grayred, (3, 3))


    #masking yellow
    low_yellow = np.array([24,85,145])
    high_yellow = np.array([80,255,255])
    maskY = cv2.inRange(hsv_img, low_yellow, high_yellow)

    yellow_mask = cv2.dilate(maskY, KERNAL)
    yellow = cv2.bitwise_and(img,img, mask=yellow_mask)
    #cv2.imshow("yellow mask",yellow)
    grayyellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
    gray_blurredyellow = cv2.blur(grayyellow, (3, 3))


    #applying morphological close for noise removal

    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11))

    binaryblue = cv2.threshold(maskB, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    closingblue = cv2.morphologyEx(binaryblue, cv2.MORPH_CLOSE, kernal, iterations=1)
    

    binaryyellow = cv2.threshold(maskY, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    closingyellow = cv2.morphologyEx(binaryyellow, cv2.MORPH_CLOSE, kernal, iterations=1)

    binaryred = cv2.threshold(maskR, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    closingred = cv2.morphologyEx(binaryred, cv2.MORPH_CLOSE, kernal, iterations=1)

    #finding largest contour of each colour

    contours,hierarchy = cv2.findContours(closingblue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt = contours
    blue_contour = []
    max = 0
    for i in cnt:
        area = cv2.contourArea(i)
        if (area>max):
            max = area
            blue_contour = i
    final = cv2.drawContours(img, blue_contour, -1, (255,0, 0), 3)
    
    

    '''contours,hierarchy = cv2.findContours(closingyellow,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt = contours
    yellow_contour = []
    max = 0
    for i in cnt:
        area = cv2.contourArea(i)
        if (area>max):
            max = area
            yellow_contour = i
    final = cv2.drawContours(img, yellow_contour, -1, (0, 255,0), 3)'''
    

    '''contours,hierarchy = cv2.findContours(closingred,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt = contours
    red_contour = []
    max = 0
    for i in cnt:
        area = cv2.contourArea(i)
        if (area>max):
            max = area
            red_contour = i
    final = cv2.drawContours(img, red_contour, -1, (0, 0, 255), 3)'''
    

    #convex hulls

    for blue_contour in contours:
        bluehull = cv2.convexHull(blue_contour)
        cv2.drawContours(img, [bluehull], 0, (255, 0, 0), 2)

    
    '''for yellow_contour in contours:
        yellowhull = cv2.convexHull(yellow_contour)
        cv2.drawContours(img, [yellowhull], 0, (0,255,0), 2)'''
    
    
    '''for red_contour in contours:
        redhull = cv2.convexHull(red_contour)
        cv2.drawContours(img, [redhull], 0, (0, 0, 255), 2)'''

    #red circle

    detected_circlesred = cv2.HoughCircles(gray_blurredred,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
    if detected_circlesred is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circlesred = np.uint16(np.around(detected_circlesred))

        for pt in detected_circlesred[0, :]:
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            cv2.circle(img, (a, b), r, (0, 255, 0), 2)


    #yellow circle
    detected_circlesyellow = cv2.HoughCircles(gray_blurredyellow,
                                           cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
    if detected_circlesyellow is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circlesyellow = np.uint16(np.around(detected_circlesyellow))

        for pt in detected_circlesyellow[0, :]:
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            cv2.circle(img, (a, b), r, (255, 255, 0), 2)

            
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
            cv2.imshow("Detected Circle", img)



    #cv2.imshow('final', final)



    if cv2.waitKey(1) & 0xff == ord('q'):
        break

