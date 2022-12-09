from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import numpy as np
import time
import threading
import cv2
cap = cv2.VideoCapture(0)
x_c = 0
y_c = 0
detected = 0
vehicle = connect('tcp:127.0.0.1:5762', baud=57600, wait_ready=True)
def opencv_code ():
  global x_c
  global y_c
  global detected
    # This code changes the value of global variable detected to 1 when centre is detected
    # and pass the values of detected center's x and y coordinates to x_c and y_c
    ####### ENTER OPENCV CODE BELOW #######
  while True:
    success, img = cap.read()

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #masking blue
    low_blue = np.array([93, 91, 160])
    high_blue = np.array([163, 255, 255])
    maskB = cv2.inRange(hsv_img, low_blue, high_blue)
    kernal = np.ones((3, 3), np.uint8)
    blue_mask = cv2.dilate(maskB, kernal)
    blue = cv2.bitwise_and(img, img, mask=blue_mask)
    #cv2.imshow("blue mask",blue)
    grayblue = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
    gray_blurredblue = cv2.blur(grayblue, (3, 3))


    #masking red
    low_red = np.array([139, 57, 150])
    high_red = np.array([179, 163, 255])
    maskR = cv2.inRange(hsv_img, low_red, high_red)
    red_mask = cv2.dilate(maskR, kernal)
    red = cv2.bitwise_and(img,img, mask=red_mask)
    #cv2.imshow("red mask",red)
    grayred = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    gray_blurredred = cv2.blur(grayred, (3, 3))

    #masking yellow
    low_yellow = np.array([11,153,177])
    high_yellow = np.array([179,255,255])
    maskY = cv2.inRange(hsv_img, low_yellow, high_yellow)
    yellow_mask = cv2.dilate(maskY, kernal)
    yellow = cv2.bitwise_and(img,img, mask=yellow_mask)
    #cv2.imshow("yellow mask",yellow)
    grayyellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
    gray_blurredyellow = cv2.blur(grayyellow, (3, 3))

    #blue circle
    detected_circlesblue = cv2.HoughCircles(gray_blurredblue,cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=1, maxRadius=1000)
    print(detected_circlesblue)
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

                    adiff1 = float(aBlue - aRed)
                    
                    bdiff1 = float(bBlue - bRed)

                    if (abs(adiff1)<=5) and (abs(bdiff1)<=5):
                        if (int(aRed)in range(160,480)) and (int(bRed)in range(120,360)):

                            cv2.circle(img,(aRed,bRed),rRed,(0,255,0),2)

                            cv2.circle(img,(aRed,bRed), 1,(0,255,0),4)

                    
                   
                    cv2.imshow("Detected Circle", img)
                    x_c = aRed
                    y_c = bRed
                    detected = 1



    if cv2.waitKey(1) & 0xff == ord('q'):
        break


    ####### ENTER OPENCV CODE ABOVE #######

def centering_and_payload_drop():
    print("centering function runned")
    # this code uses the values stored in x_c and y_c
    # only to be called after center is detected and mode changed to guided
    ####### ENTER ASWALAN'S CODE BELOW #######



    ####### ENTER ASWALAN'S CODE ABOVE #######
    ####### ENTER PAYLOAD DROP'S CODE BELOW #######



    ####### ENTER PAYLOAD DROP'S CODE ABOVE #######
    return 0
def drone_run_code():
    global detected
    arm_and_takeoff(30)
    vehicle.mode = VehicleMode("AUTO")
    while detected != 1:
        print("waiting to detect target")
    vehicle.mode = VehicleMode("GUIDED")
    centering_and_payload_drop()
    vehicle.mode = VehicleMode("LAND")

def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

thread1 = threading.Thread(target=drone_run_code)
thread1.start()
opencv_code()
