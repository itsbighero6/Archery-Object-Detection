# IMPORT 

from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from simple_pid import PID
import RPi.GPIO as GPIO

import time
import threading

import cv2
import numpy as np

# VIDEO CAPTURE

cap = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

#VARIABLES

x_c = 0
y_c = 0
detected = 0

# CONNECT TO THE VEHICLE

vehicle = connect('tcp:127.0.0.1:5762', baud=57600, wait_ready=True)

# FUNCTIONS

def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y):

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

def centering_and_payload_drop():
    global width
    global height
    global x_c
    global y_c
    pid_x = PID(1 , 0, 0, setpoint = int (width/2))
    pid_y = PID(1 , 0, 0, setpoint =int (height/2))
    print("centering function runned")
    # this code uses the values stored in x_c and y_c
    # only to be called after center is detected and mode changed to guided
    
    while True:
        x = x_c
        y = y_c
        if x == int(width/2) and y == int (height/2):
      	    break 
        velocity_x = pid_x(x)
        velocity_y = pid_y(y)
        send_ned_velocity(velocity_x, velocity_y)
        time.sleep(0.5)
    
    # PAYLOAD DROP

    # Set GPIO numbering mode
    GPIO.setmode(GPIO.BOARD)

    # Set pin 11 as an output, and set servo1 as pin 11 as PWM
    GPIO.setup(11,GPIO.OUT)
    servo1 = GPIO.PWM(11,50) # Note 11 is pin, 50 = 50Hz pulse

    #start PWM running, but with value of 0 (pulse off)
    servo1.start(0)

    # Define variable duty
    duty = 2

    print ("Turning  to 90 degrees for 2 seconds")
    servo1.ChangeDutyCycle(7)
    time.sleep(2)


    servo1.stop()
    GPIO.cleanup()

    print("Payload Dropped !")


def drone_run_code():

  global detected

  arm_and_takeoff(30)
  vehicle.mode = VehicleMode("AUTO")
  while detected != 1:
      print("waiting to detect target")
  vehicle.mode = VehicleMode("GUIDED")
  centering_and_payload_drop()
  vehicle.mode = VehicleMode("RTL")






