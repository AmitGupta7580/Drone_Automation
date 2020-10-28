import airsim
import math

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Defining points")
aSize = 20
point=[]
omega = math.pi/25
for i in range(1,50) :
    point.append([0, aSize * (math.sin(omega * i)), -30+aSize * (math.cos(omega * i)), 5])


# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
# initialize the position
client.moveToPositionAsync(0, 0, -5, 5).join()

print('go on a trajactory')
for p in point :
	client.moveToPositionAsync(p[0], p[1], p[2], p[3]).join()



landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("already landed...")
else:
    print("landing...")
    client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)