from airsim import *
import time
import math


client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
delay = 5

def flip() :
	client.moveByAngleRatesThrottleAsync(20, 0, 0, 5, 0.3).join()
	client.moveToPositionAsync(0, 0, -5, 5).join()

client.takeoffAsync().join()

client.moveToPositionAsync(0, 0, -5, 5).join()
time.sleep(delay)

flip()
time.sleep(4)
flip()
time.sleep(3)

landed = client.getMultirotorState().landed_state
if landed == LandedState.Landed:
    print("already landed...")
else:
    print("landing...")
    client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)