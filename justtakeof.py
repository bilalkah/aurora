#!/usr/bin/env python

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument("--connect", default='127.0.0.1:14550')
    args = parser.parse_args()
    
    connection_string = args.connect
    
    vehicle = connect(connection_string, wait_ready=True)
    
    return vehicle

def arm_and_takeof(aTargetAltitude):
    while not vehicle.is_armable:
        print ("Waiting for vehicle to become armable")
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print ("Waiting for mode change")
        time.sleep(1)
        
    vehicle.armed = True
    while not vehicle.armed:
        print ("Waiting for arming")
        time.sleep(1)
    
    vehicle.simple_takeoff(aTargetAltitude)
    
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
    
    print("Takeoff Successful")
    return None

vehicle = connectMyCopter()
print("Connected")

vehicle.mode = VehicleMode("GUIDED")
arm_and_takeof(5)
time.sleep(6)
land = 0

while land == 0:
    land = input("Press enter to land")

vehicle.mode = VehicleMode("LAND")
time.sleep(5)
print("Landing Successful")
print("Arducopter version: %s"%vehicle.version)
time.sleep(2)

vehicle.close()

