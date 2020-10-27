from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = 'udp:127.0.0.1:14550'


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

aSize = 38
def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, 10)

h = (math.sqrt(3)/2)*aSize
home = get_location_metres(vehicle.location.global_frame, 0, 0)
point1 = get_location_metres(vehicle.location.global_frame, 0, aSize)
point2 = get_location_metres(vehicle.location.global_frame, h, aSize/2)
point3 = get_location_metres(vehicle.location.global_frame, h, -aSize/2)
point4 = get_location_metres(vehicle.location.global_frame, 0, -aSize)
point5 = get_location_metres(vehicle.location.global_frame, -h, -aSize/2)
point6 = get_location_metres(vehicle.location.global_frame, -h, aSize/2)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



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


def download_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def add_mission(aLocation, aSize):

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, 14))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point6.lat, point6.lon, 14))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 14))

    print(" Upload new commands to vehicle")
    cmds.upload()
    time.sleep(5)


def arm_and_takeoff(aTargetAltitude):
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
add_mission(vehicle.location.global_frame,50)


arm_and_takeoff(10)

print("Starting mission")

vehicle.simple_goto(point1, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point2, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point3, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point4, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point5, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point6, groundspeed=10)
time.sleep(18)
vehicle.simple_goto(point1, groundspeed=10)
time.sleep(18)

print('Return to launch')
vehicle.simple_goto(home, groundspeed=10)
time.sleep(20)


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

