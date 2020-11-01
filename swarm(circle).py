from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

connection_string_one = 'udp:127.0.0.1:14552'
connection_string_two = 'udp:127.0.0.1:14553'
sitl = None

# Connect to the Vehicle
print('Connecting to first vehicle on: %s' % connection_string_one)
vehicle1 = connect(connection_string_one, wait_ready=True)
print('Connecting to second vehicle on: %s' % connection_string_two)
vehicle2 = connect(connection_string_two, wait_ready=True)

def get_location_metres(original_location, dNorth, dEast, alt):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, alt)

aSize = 20
print('=========   Defining points    ==========')
pt1 = get_location_metres(vehicle1.location.global_frame, 0, aSize, 30)
point1=[]
omega = math.pi/25
for i in range(1,51) :
    point1.append(get_location_metres(vehicle1.location.global_frame, aSize * (math.sin(omega * i)), aSize * (math.cos(omega * i)), 30))

ref = get_location_metres(vehicle2.location.global_frame, 0, 0, 10 + aSize)
point2=[]
for i in range(1,51) :
    point2.append(get_location_metres(ref, 0, aSize * (math.sin(omega * i)), ref.alt - aSize * (math.cos(omega * i)) ))
print('=====   Defining points completed    ====')


def add_mission():

    cmds_1 = vehicle1.commands

    print(" Clear any existing commands")
    cmds_1.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds_1.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    
    for i in point1 :
        cmds_1.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i.lat, i.lon, 1))
    for i in point2 :
        cmds_1.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i.lat, i.lon, 1))
    
    print(" Upload new commands to vehicle")
    cmds_1.upload()

    cmds_2 = vehicle2.commands

    print(" Clear any existing commands")
    cmds_2.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds_2.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    
    for i in point1 :
        cmds_2.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i.lat, i.lon, 1))
    for i in point2 :
        cmds_2.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i.lat, i.lon, 1))
    
    print(" Upload new commands to vehicle")
    cmds_2.upload()
    time.sleep(5)


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
add_mission()

arm_and_takeoff(30, vehicle1)
vehicle1.simple_goto(pt1, groundspeed=40)
time.sleep(8)
arm_and_takeoff(10, vehicle2)

print("Starting mission")

print("Following 1st-circular trajactory")

i = 0
while(i<len(point1)) :
    vehicle1.simple_goto(point1[i],groundspeed=40)
    vehicle2.simple_goto(point2[i],groundspeed=40)
    i += 1
    time.sleep(3)

print('Return to launch')
vehicle1.simple_goto(home, groundspeed=10)
vehicle2.simple_goto(home, groundspeed=10)
time.sleep(20)


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle1.close()
vehicle2.close()