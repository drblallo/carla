import sys
import glob
import os
import yaml
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  # type: ignore

#=== Load config file for CARLA connection ===
with open("VFA_carla_scripts/config.yaml", "r") as f:
    config = yaml.safe_load(f)

host = config["simulation"]["host"]
port = config["simulation"]["port"]
timeout = config["simulation"]["timeout"]

# === Connect to CARLA ===
client = carla.Client(host, port)
client.set_timeout(timeout)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# === Print spawn points ===
spawn_points = world.get_map().get_spawn_points()
custom_loc = carla.Location(x=-149.999, y=392.250, z=1.0)
spawn_custom = carla.Transform(custom_loc, carla.Rotation(pitch=0, yaw=64.856110, roll=0))

print("Spawn point A: {spawn_points[46]}")
print("Spawn point B: {spawn_custom}")

# === Vehicle spawning ===
vehicle_bp1 = blueprint_library.filter("vehicle.toyota.*")[0]
vehicle_bp2 = blueprint_library.filter("vehicle.mercedes.coupe*")[0]

spawn_transform1 = spawn_points[46]
spawn_transform2 = spawn_custom

vehicle1 = world.try_spawn_actor(vehicle_bp1, spawn_transform1)
vehicle2 = world.try_spawn_actor(vehicle_bp2, spawn_transform2)

spawned_vehicles = []

custom_loc_walker = carla.Location(x=-138.963455, y=390.100, z=4.000000)
spawn_custom_walker = carla.Transform(custom_loc_walker, carla.Rotation(pitch=0, yaw=74.490, roll=0))
actor_bp1 = blueprint_library.filter("walker.*")[39]
walker1 = world.try_spawn_actor(actor_bp1, spawn_custom_walker)
if walker1:
    print(f"Spawned walker1: {walker1.type_id} at {spawn_custom_walker}")
    spawned_vehicles.append(walker1)

time.sleep(3)

if vehicle1:
    print(f"Spawned vehicle1: {vehicle1.type_id} at {spawn_transform1}")
    spawned_vehicles.append(vehicle1)
    vehicle1.set_autopilot(True)
else:
    print("Failed to spawn Toyota.")

time.sleep(5.8)

# After connecting to CARLA, get the traffic manager
traffic_manager = client.get_trafficmanager()

# Configure aggressive behavior for vehicle2
if vehicle2:
    vehicle2.set_autopilot(True, traffic_manager.get_port())

    # Make vehicle2 more aggressive
    #traffic_manager.ignore_vehicles_percentage(vehicle1,100)
    traffic_manager.ignore_vehicles_percentage(vehicle2, 100)
    #traffic_manager.aggressive_behavior_percentage(vehicle2, 100)  # Maximum aggression
    #traffic_manager.distance_to_leading_vehicle(vehicle2, 0.5)    # Very close following
    #traffic_manager.vehicle_percentage_speed_difference(vehicle2, -50)  # 50% faster than speed limit
    #traffic_manager.ignore_lights_percentage(vehicle2, 100)       # Ignore traffic lights
    #traffic_manager.ignore_signs_percentage(vehicle2, 100)        # Ignore traffic signs

if vehicle2:
    print(f"Spawned vehicle2: {vehicle2.type_id} at {spawn_transform2}")
    spawned_vehicles.append(vehicle2)
else:
    print("Failed to spawn Mercedes.")

try:
    while True:
        world.wait_for_tick()

except KeyboardInterrupt:
    print("\nStopping simulation...")

finally:
    print("Cleaning up actors...")
    for v in spawned_vehicles:
        if v and v.is_alive:
            v.destroy()
            print(f"Destroyed {v.type_id}")
    print("=== Done ===")
