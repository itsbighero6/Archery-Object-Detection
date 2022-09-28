import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while True:
    success, img = cap.read()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #masking blue
    low_blue = np.array([95, 73, 195])
    high_blue = np.array([143, 255, 255])
    maskB = cv2.inRange(hsv_img, low_blue, high_blue)


    #masking red
    low_red = np.array([139, 57, 150])
    high_red = np.array([179, 163, 255])
    maskR = cv2.inRange(hsv_img, low_red, high_red)
    

    #masking yellow
    low_yellow = np.array([15,27,172])
    high_yellow = np.array([48,125,255])
    maskY = cv2.inRange(hsv_img, low_yellow, high_yellow)


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
    
    

    contours,hierarchy = cv2.findContours(closingyellow,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt = contours
    yellow_contour = []
    max = 0
    for i in cnt:
        area = cv2.contourArea(i)
        if (area>max):
            max = area
            yellow_contour = i
    final = cv2.drawContours(img, yellow_contour, -1, (0, 255,0), 3)
    

    contours,hierarchy = cv2.findContours(closingred,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt = contours
    red_contour = []
    max = 0
    for i in cnt:
        area = cv2.contourArea(i)
        if (area>max):
            max = area
            red_contour = i
    final = cv2.drawContours(img, red_contour, -1, (0, 0, 255), 3)
    

    # fitting ellipses

    ''' blue_ellipse = cv2.fitEllipse(blue_contour)
    ellipseblue = cv2.ellipse(img,blue_ellipse,(255,0,0),2)
    

    yellow_ellipse = cv2.fitEllipse(yellow_contour)
    ellipseyellow = cv2.ellipse(img,yellow_ellipse,(0,255,0),2)
    

    red_ellipse = cv2.fitEllipse(red_contour)
    ellipsered = cv2.ellipse(img,red_ellipse,(0,0,255),2)'''

    #convex hulls

    '''for blue_contour in contours:
        bluehull = cv2.convexHull(blue_contour)
        cv2.drawContours(img, [bluehull], 0, (255, 0, 0), 2)

    
    for yellow_contour in contours:
        yellowhull = cv2.convexHull(yellow_contour)
        cv2.drawContours(img, [yellowhull], 0, (0,255,0), 2)
    
    
    for red_contour in contours:
        redhull = cv2.convexHull(red_contour)
        cv2.drawContours(img, [redhull], 0, (255, 255, 0), 2)'''



    cv2.imshow('final', final)



    if cv2.waitKey(1) & 0xff == ord('q'):
        break

