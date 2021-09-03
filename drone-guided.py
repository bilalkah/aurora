#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import cv2
#Set up option parsing to get connection string
import argparse  


totaltime = time.time()
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

vidcap = cv2.VideoCapture(0)
success,imagedisp = vidcap.read()
height, width, layers = imagedisp.shape

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)



def arm_and_takeoff(aTargetAltitude):
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
    vehicle.mode = VehicleMode("GUIDED")
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
        
    return targetlocation;

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.05: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)
    return targetLocation
    


def get_relative_dNorth_dEast(color):
    success,imagedisp = vidcap.read()
    currentLocation = vehicle.location.global_relative_frame
    headingAngle = vehicle.heading
    altitude = vehicle.location.global_relative_frame.alt
    Threshold = 1000
    pixel = (altitude*6,48-e4)/3
    if color == "blue":
        lower_color = np.array([100,50,50])
        upper_color = np.array([130,255,255])
    elif color == "red":
        lower_color = np.array([160,50,50])
        upper_color = np.array([180,255,255])
        
    while success:
        hsv = cv2.cvtColor(imagedisp, cv2.COLOR_BGR2HSV)
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange (hsv, lower_color, upper_color)
        bluecnts = cv2.findContours(mask.copy(),
                              cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)[-2]
        x,y=0,0
        if len(bluecnts)>0:
            blue_area = max(bluecnts, key=cv2.contourArea)
            (xg,yg,wg,hg) = cv2.boundingRect(blue_area)
            print(color,wg*hg,pixel*pixel*0.95)
            if wg*hg >= pixel*pixel*0.95:
                (centerY,centerX) = imagedisp.shape[:2]
                centerY //= 2
                centerX //= 2
                objX = xg + wg/2
                objY = yg + hg/2
                lengthX = abs(centerX-objX)
                lengthY = abs(centerY-objY)
                radianOfObj = math.atan2(lengthY, lengthX)
                angleOfObj = math.degrees(radianOfObj)
                hipotenuse = math.sqrt(lengthX**2+lengthY**2)*altitude
                
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
    

# Mission import
def readmission(aFileName):

    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
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

def upload_mission(missionlist):

    print("\nUpload mission from a list")
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def distance_to_current_waypoint():

    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

missions = []
missions.append(readmission("mission-2-1.waypoints"))
missions.append(readmission("mission-2-2.waypoints"))
missions.append(readmission("mission-2-3.waypoints"))

poolLocations = []


arm_and_takeoff(10)
vehicle.mode = VehicleMode("GUIDED")
print("Set groundspeed to 10m/s.")
vehicle.groundspeed=10

for i in range(len(missions)):
    print("Mission "+str(i)+" started.")
    if i == 2:
        print("Set groundspeed to 5m/s.")
        vehicle.groundspeed = 5

        for j in range(len(poolLocations)):
            descendAlt = 3
            targetLocation = poolLocations[j]
            vehicle.simple_goto(targetLocation)
            targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)

            
            while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
                #print "DEBUG: mode: %s" % vehicle.mode.name

                remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
                print("Distance to target: ", remainingDistance)
                if remainingDistance<=targetDistance*0.05: #Just below target, in case of undershoot.
                    print("Reached target")
                    break;
                time.sleep(1)
            ascendAlt = vehicle.location.global_relative_frame.alt
            if j == 0:
                print("Descending to get water.")
            else:
                print("Descending to release water.")
            vehicle.simple_goto(LocationGlobal (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, descendAlt))
            while True:
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
                if vehicle.location.global_relative_frame.alt<=descendAlt*1.05: #Trigger just below target alt.
                    print("Reached target altitude")
                    break
                time.sleep(1)
            time.sleep(10)
            vehicle.simple_goto(LocationGlobal (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, ascendAlt))
            if j == 0:
                print("Water has been taken. Getting on the rise.")
            else:
                print("Water has been released. Getting on the rise.")
            while True:
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
                if vehicle.location.global_relative_frame.alt>=ascendAlt*0.95: #Trigger just below target alt.
                    print("Reached target altitude")
                    break
                time.sleep(1)
        

    for j in range(len(missions[i])):
        print("\nTarget GPS locations:\nLat\t\tLon\t\tAlt")
        print(str(missions[i][j][0])+"\t"+str(missions[i][j][1])+"\t"+str(missions[i][j][2]))
        
        if type(vehicle.location.global_relative_frame) is LocationGlobal:
            targetLocation=LocationGlobal(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        elif type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
            targetLocation=LocationGlobalRelative(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        vehicle.simple_goto(targetLocation)
        targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)

        counterblue = 0
        counterred = 0
        while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            if i == 1 and j == 0 and counterblue == 0:
                returnVal = get_relative_dNorth_dEast("blue")
                if returnVal is not None:
                    poolLocations.append(returnVal)
                    print("Blue area detected.")
                    counterblue = 1
            elif i == 1 and j == 0 and counterred == 0:
                returnVal = get_relative_dNorth_dEast("red")
                if returnVal is not None:
                    poolLocations.append(returnVal)
                    print("Blue area detected.")
                    counterblue = 1
            if counterblue*counterred == 1:
                vidcap.release()
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.05: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)


print("Landing..")
vehicle.mode = VehicleMode("LAND")
time.sleep(2)
vehicle.close()

print((time.time()-totaltime)//60,(time.time()-totaltime)%60)
