from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math
import time
import threading
import numpy as np
import cv2
from simple_pid import PID
#cap = cv2.VideoCapture(0)
width = 320
height = 240
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
    
def pidtune():
    while True:
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        alt = vehicle.location.global_relative_frame.alt
        newlat = -35.3633233
        newlon = 149.1653142
        if get_distance(lat , lon, newlat, newlon) == 60 : #set an arbitrary minimum distance between the drone and the center
            detected = 1
        a = get_xy(lat , lon , newlat, newlon)
        get_pixel_errors(a) 


def get_distance(lat1, lon1, lat2, lon2):
    #distance between two location
    p = 0.017453292519943295     #Pi/180
    a = 0.5 - math.cos((lat2 - lat1) * p)/2 + math.cos(lat1 * p) * math.cos(lat2 * p) * (1 - math.cos((lon2 - lon1) * p)) / 2
    return 12742 * math.asin(math.sqrt(a)) * 1000
def get_dEast(lat, newlong, lon):
    R = 6378137.0
    p = math.pi / 180
    dEast = R*p*(math.cos(p*lat))*(newlong-lon)
    return dEast
def get_dNorth(newlat, lat):
    R = 6378137.0
    p = math.pi / 180
    dNorth = R*p*(newlat - lat)
    return dNorth
def get_xy(lat , lon , newlat, newlon):
    dEast = get_dEast(lat, newlon, lon)
    dNorth = get_dNorth(newlat, lat)
    yaw = vehicle.attitude.yaw
    x = dEast*math.cos(yaw) - dNorth*math.sin(yaw)
    y = dEast*math.sin(yaw) + dNorth*math.cos(yaw)
    xy = [x,y]
    return xy

def get_pixel_errors(xy):
    global detected
    global x_c
    global y_c
    altitude = vehicle.location.global_relative_frame.alt
    if altitude != 0:
	    x_fov = 2*altitude*math.tan(1.085594794740/2)
	    y_fov = 2*altitude*math.tan(0.851720674973/2)
	    # suppose the camera is vga 640X480p 
	    # x_fov corresponds to 640p therefore
	    # 1m corresponds to 
	    p_x = 640/x_fov
	    p_y = 480/y_fov
	    #xy = get_xy()
	    x = xy[0]
	    y = xy[1]
	    x_pe = x*p_x
	    y_pe = y*p_y
	    x_c = int(320 - x_pe)
	    y_c = int(240 - y_pe)
def send_ned_velocity(velocity_x, velocity_y):
    condition_yaw(0)
    #changing coordinates to local frame
    #yaw = vehicle.attitude.yaw
    #v_x = velocity_x*math.cos(yaw) + velocity_y*math.sin(yaw)
    #v_y = -velocity_x*math.sin(yaw)+ velocity_y*math.cos(yaw)
    #print("yaw:" , yaw, " v_x = ", v_x,"  v_y= ", v_y)
  # this code is used to move drone  in veclocity_x and velocity_y
  #code taken from https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_y, velocity_x, 0, # x, y, z velocity in m/s
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
    pid_x = PID(-0.02, 0, -0.012, setpoint = int (320))
    pid_y = PID(0.02, 0, 0, setpoint =int (240))
    print("centering function runned")
    # this code uses the values stored in x_c and y_c
    # only to be called after center is detected and mode changed to guided
    ####### ENTER ASWALAN'S CODE BELOW #######
    #pid_x.sample_time = 0.5
    #pid_y.sample_time = 0.5
    while True:
        x = x_c
        y = y_c
        if x == int(320) and y == int (240):
      	    break 
        velocity_x = pid_x(x)
        velocity_y = pid_y(y)
        send_ned_velocity(velocity_x, velocity_y)
        print("y: ",y , "  x:",x , "  velocity_y:", velocity_y, "  velocity_x:", velocity_x)
        time.sleep(1)
    ####### ENTER ASWALAN'S CODE ABOVE #######
    ####### ENTER PAYLOAD DROP'S CODE BELOW #######
    a_location = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, 20)
    vehicle.simple_goto(a_location)
    while True:
    	print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    	#Break and return from function just below target altitude.        
    	if vehicle.location.global_relative_frame.alt<=20*1.05: 
      	    print ("Reached target altitude")
      	    break
    	time.sleep(1)
    while True:
        x = x_c
        y = y_c
        if x == int(320) and y == int (240):
      	    break 
        velocity_x = pid_x(x)
        velocity_y = pid_y(y)
        send_ned_velocity(velocity_x, velocity_y)
        print("y: ",y , "  x:",x , "  velocity_y:", velocity_y, "  velocity_x:", velocity_x)
        time.sleep(1)


    print("payload drop")


    ####### ENTER PAYLOAD DROP'S CODE ABOVE #######
    return 0
def drone_run_code():
  global detected
  arm_and_takeoff(30)
  #vehicle.mode = VehicleMode("AUTO")
  #while detected != 1:
      #print("waiting to detect target")
      #x = 1
      
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
#opencv_code()
pidtune()

