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
myThread.setColor("blue")

#------------------------------------------------------

counterblue = False
counterred = False
trueBlue = 3
trueRed = 2.5
descendAlt = 4.0

# GPS data of waypoints
# [0]: latitude
# [1]: longitude
# [2]: altitude
# mission-2-1.waypoints = from start to beginnig of pool areas
# mission-2-2.waypoints = from pool areas to pool areas
# mission-2-3.waypoints = from pool areas to end
missions = []
missions.append(readmission("mission-2-1.waypoints"))
missions.append(readmission("mission-2-2.waypoints"))
missions.append(readmission("mission-2-3.waypoints"))

# List that will contain blue and red pool's gps coordinates
poolLocations = []

# Arm in GUIDED mode and take of 10 alitude
arm_and_takeoff(vehicle,"GUIDED",10)

# Set ground speed for 10 m/s
set_ground_speed(vehicle, 10)


"""
Mission for loop
Between mission[0] and mission[1] the copter will fly to each waypoint and search for the red and blue pools
Between mission[1] and mission[2] the copter will fly 
    First blue pool area and it will get lower and wait until it takes water and it will get higher
    Second red pool area and it will get lower and wait until it releases water and it will get higher
    After it will fly to the end of the mission
"""


for i in range(len(missions)):
    print("Mission "+str(i)+" started.")
    
    
    if i == 2:
        set_ground_speed(vehicle, 5)

        for j in range(len(poolLocations)):
            
            if j == 0:
                myThread.setColor("blue")
            elif j == 1:
                myThread.setColor("red")
            time.sleep(0.5)
            
            
            targetLocation = poolLocations[j]
            vehicle.simple_goto(targetLocation)
            targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)

            
            while vehicle.mode.name=="GUIDED": 

                remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
                print(" Distance to target: ", remainingDistance)
                if remainingDistance<=targetDistance*0.05 or remainingDistance < 0.15: 
                    print("Reached target")
                    break;
                time.sleep(1)
            
            
            ascendAlt = vehicle.location.global_relative_frame.alt
            altArr = np.linspace(ascendAlt, descendAlt, num=3)
            
            if j == 0:
                print("Descending to get water.")
            else:
                print("Descending to release water.")
            
            print("Aktuator opening..")
            aktuator.change_duty(45)
            
            for x in altArr:
                fixPosition(vehicle, myThread.readLoc())
                
                vehicle.simple_goto(LocationGlobalRelative (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, x))
                
                while True:
                    print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
                    if vehicle.location.global_relative_frame.alt<=descendAlt*1.05: 
                        print("Reached target altitude")
                        break
                    time.sleep(1)
                
            print("Aktuator closing")
            aktuator.change_duty(135)
            
            
            
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
        

    for j in range(len(missions[i])):
        count = 0
        if i == 1 and j == 0:
            myThread.setColor(color="blue")
            set_ground_speed(vehicle, 2)
            
        print("\nTarget GPS locations:\nLat\t\tLon\t\tAlt")
        print(str(missions[i][j][0])+"\t"+str(missions[i][j][1])+"\t"+str(missions[i][j][2]))
        
        if type(vehicle.location.global_relative_frame) is LocationGlobal:
            targetLocation=LocationGlobal(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        elif type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
            targetLocation=LocationGlobalRelative(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        
        vehicle.simple_goto(targetLocation)
        targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        
        if i == 1 and j == 1:
            set_ground_speed(vehicle, 10)
            
        while vehicle.mode.name=="GUIDED":
            remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            if i == 1 and j == 0 and counterblue is False:
                returnVal = get_relative_dNorth_dEast(vehicle,myThread.readLoc(),trueBlue)
                if returnVal is not None:
                    myThread.setColor("red")
                    poolLocations.insert(0, returnVal)
                    print("*******************")
                    print("Blue area detected.")
                    print("*******************")
                    myThread.setBlue()
                    counterblue = True
                    myThread.setLoc()
                    time.sleep(1)
                    
            elif i == 1 and j == 0 and counterred is False:
                returnVal = get_relative_dNorth_dEast(vehicle,myThread.readLoc(),trueRed)
                if returnVal is not None:
                    poolLocations.append(returnVal)
                    print("*******************")
                    print("Red area detected.")
                    print("*******************")
                    myThread.setRed()
                    counterred = True
                    time.sleep(1)
                    
            count += 1
            if count %5 == 0:
                print(" Distance to target: ", remainingDistance)
                
            if remainingDistance<=targetDistance*0.05 or remainingDistance < 0.15: 
                print("Reached target.")
                break
            time.sleep(0.5)

# Land the vehicle
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

myThread.finish()
