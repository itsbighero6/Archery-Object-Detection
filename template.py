import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

cap = cv.VideoCapture(0)

template = cv.imread('/home/itsbighero6/Documents/Archery-Object-Detection-1/Resources/target.png',0)
while True:
    _,img_rgb = cap.read()
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
    w, h = template.shape[::-1]
    res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)
    threshold = 0.
    loc = np.where( res >= threshold)
    for pt in zip(*loc[::-1]):
        cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
    cv.imshow('res.png',img_rgb)
    if cv.waitKey(10) & 0xFF == 27:
        break
cap.release()