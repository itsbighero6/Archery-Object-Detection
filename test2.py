import numpy as np
import cv2 as cv

cap = cv.VideoCapture(0)


while True:
    _,img = cap.read()
    img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    img = cv.medianBlur(img,5)
    cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
    circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=10,maxRadius=60)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
    # draw the outer circle
        cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    cv.imshow('detected circles',cimg)

    if cv.waitKey(10) & 0xFF == 27:
        break
cv.waitKey(0)
cv.destroyAllWindows()