from __future__ import print_function
from utils import *
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil 
import time
import argparse  

# Arg parser for connecting to the vehicle
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string.")
args = parser.parse_args()
connection_string = args.connect


# Vehicle connection
vehicle = connectCopter(connection_string)

missions = []
missions.append(readmission("test.waypoints"))

arm_and_takeoff(vehicle,"GUIDED",10)

set_ground_speed(vehicle, 10)

for i in range(len(missions)):
    print("Mission "+str(i)+" started.")    

    for j in range(len(missions[i])):
        print("\nTarget GPS locations:\nLat\t\tLon\t\tAlt")
        print(str(missions[i][j][0])+"\t"+str(missions[i][j][1])+"\t"+str(missions[i][j][2]))
        
        if type(vehicle.location.global_relative_frame) is LocationGlobal:
            targetLocation=LocationGlobal(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        elif type(vehicle.location.global_relative_frame) is LocationGlobalRelative:
            targetLocation=LocationGlobalRelative(missions[i][j][0],missions[i][j][1],missions[i][j][2])
        
        vehicle.simple_goto(targetLocation)
        targetDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)

        while vehicle.mode.name=="GUIDED": 

            remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.05 or remainingDistance < 0.15: 
                print("Reached target.")
                break;
            time.sleep(2)
            
print("Landing..")
vehicle.mode = VehicleMode("LAND")

while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt<0.2: #Trigger just below target alt.
            print("Landed")
            break
        time.sleep(1)
        
vehicle.armed = False
# Wait for disarming and close the connection
while vehicle.armed:
    print("Waiting for disarming.")
    time.sleep(1)
print("Disarmed.")

vehicle.close()
print("Vehicle closed.")