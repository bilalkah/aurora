#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from utils import *
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil 
import time
import math
import numpy as np
import argparse  
from monitor import *
from servo import *

# Count of program time
programTime = time.time()

# Arg parser for connecting to the vehicle
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string.")
args = parser.parse_args()
connection_string = args.connect


# Vehicle connection
vehicle = connectCopter(connection_string)

# Servo connection

aktuator = myServo()
aktuator.change_duty(135)
#------------------------------------------------------
# Threading for monitor camera

myThread = ThreadedVideoStream(vehicle=vehicle, imwrite=True)
#myThread.setColor("red")

#------------------------------------------------------

counterblue = False
counterred = False
trueBlue = 3
trueRed = 2.5

descendAlt = 2.0

# GPS data of waypoints
# [0]: latitude
# [1]: longitude
# [2]: altitude
missions = readmission("sualma.waypoints")


# List that will contain blue and red pool's gps coordinates
poolLocations = []
poolLocations.append(LocationGlobal (missions[0][0],missions[0][1], missions[0][2]))
start = time.time()
# Arm in GUIDED mode and take of 15 alitude
arm_and_takeoff(vehicle,"GUIDED",15)

# Set ground speed for 5 m/s
set_ground_speed(vehicle, 5)


for i in range(len(missions)):
    print("Mission "+str(i)+" started.")
    
    if i == 3:
        myThread.setColor("red")
        set_ground_speed(vehicle, 1.5)
    elif i == 4:
        set_ground_speed(vehicle, 5)
    if i == 5:
        set_ground_speed(vehicle, 5)

        for j in range(len(poolLocations)):
            
            if j == 0:
                myThread.setColor("blue")
                descendAlt = 1.5
            elif j == 1:
                myThread.setColor("red")
                descendAlt = 3
            time.sleep(0.5)
            
            
            targetLocation = poolLocations[j]
            vehicle.simple_goto(targetLocation)
            targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            lat,lon,alt = targetLocation.lat,targetLocation.lon,targetLocation.alt
            
            while vehicle.mode.name=="GUIDED": 

                remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
                print(" Distance to target: ", remainingDistance)
                if remainingDistance<=targetDistance*0.05 or remainingDistance < 0.15: 
                    print("Reached target")
                    break;
                time.sleep(1)
            
            
            ascendAlt = vehicle.location.global_relative_frame.alt
            altArr = np.linspace(ascendAlt, descendAlt, num=4)
            
            if j == 0:
                print("Descending to get water.")
            else:
                print("Descending to release water.")
            
            print("Aktuator opening..")
            aktuator.change_duty(45)
            time.sleep(2)
            for idx in range(len(altArr)):
                fixPosition(vehicle, myThread.readLoc())
                print(altArr[idx])
                vehicle.simple_goto(LocationGlobalRelative (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, altArr[idx]))
                
                while True:
                    """
                    if j == 0 and waterSense(myThread.read()[1]):
                        print("Water detected.")
                        break
                    """
                    print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
                    if vehicle.location.global_relative_frame.alt<=altArr[idx]*1.1: 
                        print("Reached target altitude")
                        break
                    time.sleep(1)
            if vehicle.mode == "GUIDED":
                print("Brake mod")
                vehicle.mode = VehicleMode("BRAKE")
                time.sleep(1)
                print("Aktuator closing")
                aktuator.change_duty(135)
                time.sleep(2)
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
            
            vehicle.simple_goto(LocationGlobalRelative (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, ascendAlt))
            if j == 0:
                print("Water has been taken. Getting on the rise.")
            else:
                print("Water has been released. Getting on the rise.")
            while True:
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
                if vehicle.location.global_relative_frame.alt>=ascendAlt*0.95: 
                    print("Reached target altitude.")
                    break
                time.sleep(1)
        

    
            
    print("\nTarget GPS locations:\nLat\t\tLon\t\tAlt")
    print(str(missions[i][0])+"\t"+str(missions[i][1])+"\t"+str(missions[i][2]))
    
    if type(vehicle.location.global_relative_frame) is LocationGlobal:
        targetLocation=LocationGlobal(missions[i][0],missions[i][1],missions[i][2])
    elif type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
        targetLocation=LocationGlobalRelative(missions[i][0],missions[i][1],missions[i][2])
    
    vehicle.simple_goto(targetLocation)
    targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    
        
    while vehicle.mode.name=="GUIDED":
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        """
        if i == 0 and counterblue is False:
            returnVal = get_relative_dNorth_dEast(vehicle,myThread.readLoc(),trueBlue)
            if returnVal is not None:
                myThread.setColor("red")
                poolLocations.insert(0, returnVal)
                print("*******************")
                print("Blue area detected.")
                print("*******************")
                print("Blue Coords: %f %f %f", returnVal[0], returnVal[1], returnVal[2])
                myThread.setBlue()
                counterblue = True
                myThread.setLoc()
                time.sleep(1)
        """      
        if i == 3 and counterred is False:
            returnVal = get_relative_dNorth_dEast(vehicle,myThread.readLoc(),trueRed)
            if returnVal is not None:
                poolLocations.append(returnVal)
                print("*******************")
                print("Red area detected.")
                print("*******************")
                print("Red Coords: ", returnVal.lat, returnVal.lon, returnVal.alt)
                myThread.setRed()
                counterred = True
                set_ground_speed(vehicle, 5)
                time.sleep(1)
                
        
        print(" Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.05 or remainingDistance < 0.15: 
            print("Reached target.")
            break
        time.sleep(0.5)
        
stop = time.time()
# Land the vehicle
if vehicle.mode == "GUIDED":
    print("Landing..")
    vehicle.mode = VehicleMode("LAND")

    while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
            if vehicle.location.global_relative_frame.alt<0.2: #Trigger just below target alt.
                print("Landed")
                break
            time.sleep(1)

    # Print duty time
    totaltime = time.time() - programTime
    print("%d:%02d"%(totaltime//60,totaltime%60))

    vehicle.armed = False
    # Wait for disarming and close the connection
    while vehicle.armed:
        print("Waiting for disarming.")
        time.sleep(1)
    print("Disarmed.")

    vehicle.close()
    print("Vehicle closed.")

    #myThread.finish()
    print(stop-start)
