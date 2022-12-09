from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import threading
import numpy as np
import cv2
from simple_pid import PID
cap = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
x_c = 0
y_c = 0
detected = 0
vehicle = connect('tcp:127.0.0.1:5762', baud=57600, wait_ready=True)
def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
def send_ned_velocity(velocity_x, velocity_y):
    condition_yaw(0)
  # this code is used to move drone  in veclocity_x and velocity_y
  #code taken from https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink
    vehicle.send_mavlink(msg)
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
    #print(detected_circlesblue)
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
    global width
    global height
    global x_c
    global y_c
    pid_x = PID(-0.02, 0, -0.012, setpoint = int (width/2))
    pid_y = PID(0.02, 0, 0, setpoint =int (height/2))
    print("centering function runned")
    condition_yaw(0)
    # this code uses the values stored in x_c and y_c
    # only to be called after center is detected and mode changed to guided
    ####### ENTER ASWALAN'S CODE BELOW #######
    while True:
        x = x_c
        y = y_c
        if x == int(width/2) and y == int (height/2):
      	    break 
        velocity_x = pid_x(x)
        velocity_y = pid_y(y)
        send_ned_velocity(velocity_x, velocity_y)
        time.sleep(1)
    ####### ENTER ASWALAN'S CODE ABOVE #######
    ####### ENTER PAYLOAD DROP'S CODE BELOW #######
    print("payload drop")


    ####### ENTER PAYLOAD DROP'S CODE ABOVE #######
    return 0
def drone_run_code():
  global detected
  arm_and_takeoff(30)
  vehicle.mode = VehicleMode("AUTO")
  while detected != 1:
      #print("waiting to detect target")
      print(vehicle.location.global_relative_frame)
      print(vehicle.location.local_frame)
  vehicle.mode = VehicleMode("GUIDED")
  centering_and_payload_drop()
  vehicle.mode = VehicleMode("RTL")

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
