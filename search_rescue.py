from __future__ import division
from math import cos, asin, sqrt, sin, pi, radians

# import the necessary packages
import numpy as np
import cv2

from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil # Needed for command message definitions

import time


#connect
vehicle = connect("tcp:127.0.0.1:5762", wait_ready = True)

# load the image, clone it for output, and then convert it to grayscale
cap = cv2.VideoCapture(0)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt )
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print ("Reached target altitude")
            break
        time.sleep(1)

def get_distance(lat1, lon1, lat2, lon2):
    #distance between two location
    p = 0.017453292519943295     #Pi/180
    a = 0.5 - cos((lat2 - lat1) * p)/2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
    return 12742 * asin(sqrt(a)) * 1000

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*cos(pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/pi)
    newlon = original_location.lon + (dLon * 180/pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def send_ned_position(d_north, d_east):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        d_north, d_east, 0, # x, y, z positions 
        0,0,0, # x, y, z velocity in m/s (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
    vehicle.send_mavlink(msg)
    time.sleep(2)
    print ("success")
        

arm_and_takeoff(20)

print ("Set default/target airspeed to 10")
vehicle.airspeed = 10

print ("Going towards first point for 30 seconds ...")

width_px = cap.get(3)
height_px = cap.get(4)


while not vehicle.mode == VehicleMode("AUTO"):
    vehicle.mode = VehicleMode("AUTO")
    print ("auto")
    time.sleep(1)
    
print ("image processing time")
while True:

    while vehicle.mode == VehicleMode("LOITER"):
        vehicle.mode = VehicleMode("RTL")
        time.sleep(1)

    current_width_meter = 1.02 * vehicle.location.global_relative_frame.alt #1.02 * vehicle.location.global_relative_frame.alt
    current_height_meter = 0.75 * vehicle.location.global_relative_frame.alt #0.75 * vehicle.location.global_relative_frame.alt

    
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detect circles in the image
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50)
     
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
     
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            print (x,y,r)
        
        img = gray[int(y-r/3):int(y+r/3), int(x-r/5):int(x+r/5)]
        crop = np.float32(img)

        corners = cv2.goodFeaturesToTrack(crop,1,0.01,10)
        if corners is not None:
            corners = np.int0(corners)
        else:
            print ("No corner found. Quitting.")
            break

        for corner in corners:
            a,b = corner.ravel()
            print ("target location", a,b)
            #important line that finds the location of the target.
            cv2.circle(frame, (int(x-r/5+a),int(y-r/3+b)),3,255,-1)
            #end of the important line

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, "x:", (10,20), font, 0.6, (0,0,255), 1,cv2.LINE_AA)
        cv2.putText(frame, str(x-r/5+a), (25,20), font, 0.6, (0,0,255), 1,cv2.LINE_AA)
        cv2.putText(frame, "y:", (10,40), font, 0.6, (0,0,255), 1,cv2.LINE_AA)
        cv2.putText(frame, str(y-r/3+b), (25,40), font, 0.6, (0,0,255), 1,cv2.LINE_AA)

        del_x_px = (x-r/5+a) - width_px/2
        del_y_px = (y-r/3+b) - height_px/2

        del_x_meter = del_x_px / width_px * current_width_meter
        del_y_meter = (del_y_px / height_px) * current_height_meter

        #vehicle orientation
        yaw_degree = vehicle.heading  #vehicle.heading
        print (yaw_degree, del_x_meter, del_y_meter)
        
        d_north = del_y_meter * cos(radians(yaw_degree)) - del_x_meter * sin(radians(yaw_degree))
        d_east = del_y_meter * sin(radians(yaw_degree)) + del_x_meter * cos(radians(yaw_degree))

        new_location = get_location_metres(vehicle.location.global_relative_frame,d_north, d_east)
        print (vehicle.location.global_relative_frame, new_location)
        #print get_distance(new_location.lat, new_location.lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)

        while not vehicle.mode == VehicleMode("GUIDED"):
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.01)
        
        send_ned_position(d_north, d_east)
            
        #show the output image
        cv2.imshow("output", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            vehicle.close()
            print("Completed")
            break
        
    else:
        print ("no target found")
        while not vehicle.mode == VehicleMode("AUTO"):
            vehicle.mode = VehicleMode("AUTO")
            time.sleep(0.01)
        print( vehicle.heading, vehicle.location.global_relative_frame.alt)
        time.sleep(0.05)
        cv2.imshow("output", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
cap.release()
cv2.destroyAllWindows()
