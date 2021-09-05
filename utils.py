from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import cv2
import numpy as np
from realPos import *

def connectCopter(connection_string):
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle,mode,aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode(mode)
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_relative_dNorth_dEast(vehicle, info):
    
    (sizeX,sizeY )= process(info,altitude = vehicle.location.global_relative_frame.alt)
    currentLocation = vehicle.location.global_relative_frame
    headingAngle = vehicle.heading
    (xg,yg,wg,hg,centerY,centerX) = info
    
    print(sizeX*sizeY, "m2 detected.")
    if sizeX*sizeY >= 2.5*2.5*0.9:
        objX = xg + wg/2
        objY = yg + hg/2
        lengthX = abs(centerX-objX)
        lengthY = abs(centerY-objY)
        radianOfObj = math.atan2(lengthY, lengthX)
        angleOfObj = math.degrees(radianOfObj)
        hipotenuse = math.sqrt(sizeX**2+sizeY**2)
                
        dNorth , dEast = 0,0
        if centerX < objX and centerY > objY:
            angleOfObj = 90 - angleOfObj
        elif centerX < objX and centerY <= objY:
            angleOfObj = 90 + angleOfObj
        elif centerX >= objX and centerY < objY:
            angleOfObj = 270 - angleOfObj
        elif centerX > objX and centerY >= objY:
            angleOfObj = 270 + angleOfObj
                    
        NorthToTargetAngle = angleOfObj + headingAngle
        NorthToTargetAngle %= 360
                
        if NorthToTargetAngle > 0 and NorthToTargetAngle <= 90:
            dNorth = 1
            dEast = 1
            NorthToTargetAngle = 90 - NorthToTargetAngle
        elif NorthToTargetAngle > 90 and NorthToTargetAngle <= 180:
            dNorth = -1
            dEast = 1
            NorthToTargetAngle = NorthToTargetAngle - 90  
        elif NorthToTargetAngle > 180 and NorthToTargetAngle <= 270:
            dNorth = -1
            dEast = -1
            NorthToTargetAngle = 270 - NorthToTargetAngle
        elif NorthToTargetAngle > 270 and NorthToTargetAngle <= 360:
            dNorth = 1
            dEast = -1
            NorthToTargetAngle = NorthToTargetAngle - 270
        dNorth *= hipotenuse*math.sin(math.radians(NorthToTargetAngle))
        dEast *= hipotenuse*math.cos(math.radians(NorthToTargetAngle))
        targetLocation = get_location_metres(currentLocation, dNorth, dEast)
        return targetLocation
    else:
        return None


def readmission(aFileName):
    
    print("\nReading mission from file: %s" % aFileName)
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                print(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                gpsData = [ln_param5, ln_param6, 10.0]
                missionlist.append(gpsData)
    return missionlist


def set_ground_speed(vehicle, speed):
    print("Setting ground speed to " + str(speed))
    vehicle.groundspeed = speed

