import asyncio
from dronekit import LocationGlobalRelative, LocationGlobal
from mavsdk import System

async def get_Global_Location(drone) :
    async for terrain_info in drone.telemetry.home():
        alt = terrain_info.absolute_altitude_m
        lat = terrain_info.latitude_deg
        lon = terrain_info.longitude_deg
        break
    return LocationGlobal(lat, lon, alt)


async def run():
    ###################################################################################
    # Connecting Drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    ###################################################################################

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health() :
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    home = await get_Global_Location(drone)
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)
    while True:
        async for terrain_info in drone.telemetry.home():
            alt = terrain_info.absolute_altitude_m
            break
        print(alt)
        await asyncio.sleep(1)
        if alt >= 2 :
            break
    ###################################################################################
    #goto_location() takes Absolute MSL altitude 
    # await drone.action.goto_location(47.399386, 8.535245, flying_alt, 0)
    ###################################################################################
    await drone.action.return_to_launch()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())