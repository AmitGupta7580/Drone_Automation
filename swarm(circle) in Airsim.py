import airsim
import math

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

print("Defining points")
aSize = 10
point1=[]
point2=[]
omega = math.pi/25
for i in range(1,50) :
    point1.append([0, aSize * (math.sin(omega * i)), -15+aSize * (math.cos(omega * i)), 3])
    point2.append([aSize * (math.sin(omega * i)), aSize * (math.cos(omega * i)), -15, 3])

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="Drone1").join()
client.moveToPositionAsync(0, 0, -5, 5, vehicle_name="Drone1").join()
client.takeoffAsync(vehicle_name="Drone2").join()
client.moveToPositionAsync(0, 10, -15, 5, vehicle_name="Drone2").join()

i=0
while(i<len(point1)) :
	client.moveToPositionAsync(point1[i][0], point1[i][1], point1[i][2], point1[i][3], vehicle_name="Drone1").join()
	client.moveToPositionAsync(point2[i][0], point2[i][1], point2[i][2], point2[i][3], vehicle_name="Drone2").join()
	i += 1
print("Mission completed")

client.moveToPositionAsync(0, 0, 0, 5, vehicle_name="Drone1").join()
client.moveToPositionAsync(0, 4, 0, 5, vehicle_name="Drone2").join()

# landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
# if landed == airsim.LandedState.Landed:
#     print("already landed...")
# else:
#     print("landing...")
#     client.landAsync(vehicle_name="Drone1").join()

# client.armDisarm(False)
# client.enableApiControl(False)