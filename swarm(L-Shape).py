from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

connection_string_one = 'udp:127.0.0.1:14553'
connection_string_two = 'udp:127.0.0.1:14554'
connection_string_three = 'udp:127.0.0.1:14555'
sitl = None

# Connect to the Vehicle
print('Connecting to first vehicle on: %s' % connection_string_one)
vehicle1 = connect(connection_string_one, wait_ready=True)
print('Connecting to second vehicle on: %s' % connection_string_two)
vehicle2 = connect(connection_string_two, wait_ready=True)
# print('Connecting to third vehicle on: %s' % connection_string_three)
# vehicle3 = connect(connection_string_three, wait_ready=True)

def get_location_metres(original_location, dNorth, dEast, alt):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, alt)

aSize = 40
print('=========   Defining points    ==========')
theta = math.pi/3
home = get_location_metres(vehicle1.location.global_frame, 0, 0, 10)
pt1 = get_location_metres(vehicle1.location.global_frame, 0, aSize, 10)
pt2 = get_location_metres(vehicle1.location.global_frame, aSize, aSize, 10)
print('=====   Defining points completed    ====')

def arm_and_takeoff(aTargetAltitude, vehicle):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

print('Create a new mission (for current location)')
# add_mission()

arm_and_takeoff(10, vehicle2)
vehicle2.simple_goto(get_location_metres(vehicle1.location.global_frame, 0, -80, 10), groundspeed=20)
time.sleep(20)
arm_and_takeoff(10, vehicle1)
vehicle1.simple_goto(pt1, groundspeed=10)
for i in range(8) :
    p = get_location_metres(vehicle1.location.global_relative_frame, 0, -80, 10)
    vehicle2.simple_goto(p, groundspeed=20)
    time.sleep(2)
vehicle1.simple_goto(pt2, groundspeed=10)
for i in range(8) :
    p = get_location_metres(vehicle1.location.global_relative_frame, 0, -80, 10)
    vehicle2.simple_goto(p, groundspeed=20)
    time.sleep(2)
# arm_and_takeoff(10, vehicle2)

print('Return to launch')
vehicle1.simple_goto(home, groundspeed=10)
vehicle2.simple_goto(home, groundspeed=10)
time.sleep(20)


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle1.close()
vehicle2.close()
# vehicle3.close()