import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while True:
    success, img = cap.read()
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low_red = np.array([130, 77, 70])
    high_red = np.array([220, 255, 255])
    mask = cv2.inRange(hsv_img, low_red, high_red)
    kernal = np.ones((3,3),np.uint8)
    red_mask = cv2.dilate(mask, kernal)
    red = cv2.bitwise_and(img, img, mask=red_mask)
    gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.blur(gray, (3, 3))
    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)

    # Draw circles that are detected.
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            cv2.circle(img, (a, b), r, (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
            cv2.imshow("Detected Circle", img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break