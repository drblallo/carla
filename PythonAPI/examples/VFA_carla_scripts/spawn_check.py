#!/usr/bin/env python
import sys
import glob
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  # type: ignore
import yaml
import time

# === Load config file for CARLA connection ===
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

# === Print available spawn points ===
spawn_points = world.get_map().get_spawn_points()

print("==== Available Spawn Points ====")
for i, sp in enumerate(spawn_points):
    print(f"{i}: {sp}")

# === Draw debug markers ===
if spawn_points:
    for i in range(len(spawn_points)):
        loc = spawn_points[i].location
        world.debug.draw_string(
            loc,
            f"SpawnPoint {i}",
            draw_shadow=True,
            color=carla.Color(r=255, g=0, b=0),
            life_time=300.0,
            persistent_lines=True
        )
print(f"start: {spawn_points[206]}")
print(f"end: {spawn_points[26]}")

print(f"spawn manual control: {spawn_points[259]}")

custom_loc = carla.Location(x=-150.4921875, y=392.38191406, z=4.0)
custom_loc = carla.Location(x=-142.0, y=398.0, z=0.0) #Transform(Location(x=-138.963455, y=396.850616, z=4.000000)
world.debug.draw_string(
    custom_loc,
    "My Custom Point",
    draw_shadow=True,
    color=carla.Color(r=0, g=255, b=0),
    life_time=300.0,
    persistent_lines=True
)

custom_loc = carla.Location(x=-149.999, y=392.250, z=1.0)
spawn_custom = carla.Transform(custom_loc, carla.Rotation(pitch=0, yaw=64.856110, roll=0))

print(f"Spawn point A: {spawn_points[46]}")
print(f"Spawn point B: {spawn_custom}")

# === Vehicle spawning ===
vehicle_bp1 = blueprint_library.filter("vehicle.toyota.*")[0]
vehicle_bp2 = blueprint_library.filter("vehicle.dodge.*")[2]

spawn_transform1 = spawn_points[46]
spawn_transform2 = spawn_points[35] #spawn_custom

vehicle1 = world.try_spawn_actor(vehicle_bp1, spawn_transform1)
vehicle2 = world.try_spawn_actor(vehicle_bp2, spawn_transform2)

spawned_actors = []

if vehicle1:
    print(f"Spawned vehicle1: {vehicle1.type_id} at {spawn_transform1}")
    spawned_actors.append(vehicle1)
else:
    print("Failed to spawn Toyota.")

if vehicle2:
    print(f"Spawned vehicle2: {vehicle2.type_id} at {spawn_transform2}")
    spawned_actors.append(vehicle2)
else:
    print("Failed to spawn Mercedes.")

# === Walker spawning === 
# 21, 35, 39=police
custom_loc_walker = carla.Location(x=-138.963455, y=390.100, z=4.000000)
spawn_custom_walker = carla.Transform(custom_loc_walker, carla.Rotation(pitch=0, yaw=74.490, roll=0))
actor_bp1 = blueprint_library.filter("walker.*")[21]
walker1 = world.try_spawn_actor(actor_bp1, spawn_custom_walker)
if walker1:
    print(f"Spawned walker1: {walker1.type_id} at {spawn_custom_walker}")
    spawned_actors.append(walker1)
    #walker1.blend_pose(True) T-pose
else:
    print("Failed to spawn walker.")

if walker1:
    # Spawn the AI controller for the walker
    walker_controller_bp = blueprint_library.find('controller.ai.walker')
    walker_controller = world.spawn_actor(walker_controller_bp, carla.Transform(), walker1)
    
    # Get the walker's current transform
    walker_transform = walker1.get_transform()
    
    # Calculate a point in front of the walker
    forward_vector = walker_transform.get_forward_vector()
    # Try alternative method - set walker control directly
    walker_control = carla.WalkerControl()
    walker_control.speed = 1.4
    walker_control.direction = forward_vector  # Use the forward vector you calculated
    
    # Apply the control directly to the walker
    walker1.apply_control(walker_control)
    print("Applied direct walker control")

    # Add to cleanup list
    spawned_actors.append(walker_controller)

time.sleep(10)

print("Cleaning up actors...")
for v in spawned_actors:
    if v and v.is_alive:
        v.destroy()
        print(f"Destroyed {v.type_id}")
print("=== Done ===")
