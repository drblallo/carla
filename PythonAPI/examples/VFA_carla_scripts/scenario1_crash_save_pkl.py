#!/usr/bin/env python
import sys
import glob
import os
import yaml
import time
import logging

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  # type: ignore

# Import the trajectory recorder and collision detector modules
from trajectory_recorder import TrajectoryRecorder, RecordingMode
from collision_detector import CollisionDetector, CollisionMethod
from utils import save_combined_data, setup_world_settings
from actor_manager import ActorManager

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('scenario1_crash.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# === Load config file for CARLA connection ===
with open("VFA_carla_scripts/config.yaml", "r") as f:
    config = yaml.safe_load(f)

host = config["simulation"]["host"]
port = config["simulation"]["port"]
timeout = config["simulation"]["timeout"]

def main():
    # === Connect to CARLA ===
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    
    # === Configure world for asynchronous mode ===
    settings = world.get_settings()
    settings.synchronous_mode = False  # Asynchronous mode
    settings.no_rendering_mode = False
    world.apply_settings(settings)
    
    logger.info("World configured for asynchronous mode")
    
    # === Initialize trajectory recorder (10Hz) ===
    trajectory_recorder = TrajectoryRecorder(
        world=world,
        save_interval=300,  # Save every 300 frames
        filename="output/scenario1_trajectories.pkl",
        record_bbox=True,  # Record bounding boxes
        recording_mode=RecordingMode.SIMULATION_TIME,  # Best for async mode
        frequency=10.0,  # 10Hz recording frequency
        frame_interval=1,
        fixed_delta_seconds=0.05,
        actor_cache_interval=30,
        enable_gps=True
    )
    
    # === Initialize collision detector ===
    collision_detector = CollisionDetector(
        world=world,
        detection_method=CollisionMethod.HYBRID,
        distance_threshold=2.0,
        collision_update_interval=3,
        actor_cache_interval=30
    )
    
    logger.info("Trajectory recorder and collision detector initialized")
    
    # === Prepare spawn points ===
    spawn_points = world.get_map().get_spawn_points()
    # custom_loc = carla.Location(x=-149.999, y=392.250, z=1.0)
    custom_loc = carla.Location(x=172.42326172, y=335.92167969, z=1.0)
    spawn_custom = carla.Transform(custom_loc, carla.Rotation(pitch=0, yaw=64.856110, roll=0))

    print(f"Spawn point A: {spawn_points[46]}")
    print(f"Spawn point B: {spawn_custom}")

    # === Prepare vehicle blueprints ===
    vehicle_bp2 = blueprint_library.filter("vehicle.dodge*")[2]
    vehicle_bp1 = blueprint_library.filter("vehicle.mercedes.coupe*")[0]
    walker_bp = blueprint_library.filter("walker.*")[39]

    spawn_transform1 = spawn_points[46]
    spawn_transform2 = spawn_points[35] #spawn_custom
    
    # Walker spawn point
    custom_loc_walker = carla.Location(x=-138.963455, y=390.100, z=4.000000)
    spawn_custom_walker = carla.Transform(custom_loc_walker, carla.Rotation(pitch=0, yaw=74.490, roll=0))

    # === Initialize variables for the main loop ===
    spawned_vehicles = []
    vehicle1 = None
    vehicle2 = None
    walker1 = None
    traffic_manager = client.get_trafficmanager()
    
    # Timing variables (in seconds)
    start_time = None
    #vehicle1_start_time = 3
    vehicle1_start_time = 2.1
    started_1 = False
    vehicle2_start_time = 0 
    started_2 = False
    
    # Collision detection variables
    collision_detected = False
    vehicles_stopped = False
    collision_time = None
    walker_moving_to_destination = False
    walker_at_destination = False

    frame_count = 0

    # === Spawn actors at the beginning ===
    vehicle1 = world.try_spawn_actor(vehicle_bp1, spawn_transform1)
    vehicle2 = world.try_spawn_actor(vehicle_bp2, spawn_transform2)
    walker1 = world.try_spawn_actor(walker_bp, spawn_custom_walker)

    if vehicle1:
        print(f"Spawned vehicle1: {vehicle1.type_id} at {spawn_transform1}")
        spawned_vehicles.append(vehicle1)
        #traffic_manager.ignore_vehicles_percentage(vehicle1, 100)
        
        # Add collision sensor to vehicle1
        collision_detector.add_collision_sensor(vehicle1)
        logger.info(f"Vehicle1 spawned: {vehicle1.type_id} at {spawn_transform1}")
    else:
        print("Failed to spawn vehicle 1.")

    if vehicle2:
        # Make vehicle2 more aggressive
        #traffic_manager.ignore_vehicles_percentage(vehicle2, 100)
        traffic_manager.ignore_lights_percentage(vehicle2, 100)       
        traffic_manager.ignore_signs_percentage(vehicle2, 100)
        destination = spawn_points[175].location
        traffic_manager.set_path(vehicle2, [destination])

        logger.info(f"Vehicle2 spawned: {vehicle2.type_id} at {spawn_transform2}")
        spawned_vehicles.append(vehicle2)
        
        # Add collision sensor to vehicle2
        collision_detector.add_collision_sensor(vehicle2)
    else:
        print("Failed to spawn vehicle 2.")

    if walker1:
        print(f"Spawned walker1: {walker1.type_id} at {spawn_custom_walker}")
        spawned_vehicles.append(walker1)
    else:    
        print("Failed to spawn walker.")

    time.sleep(10)

    logger.info("Starting simulation loop with trajectory recording")
    
    try:
        while True:

            # Wait for next tick in asynchronous mode
            snapshot = world.wait_for_tick() #world.tick() --> sim va avanti di 50ms
            elapsed_time = snapshot.timestamp.elapsed_seconds
            
            # Set start time on first iteration
            if start_time is None:
                start_time = elapsed_time
            
            # Calculate relative time since start
            relative_time = elapsed_time - start_time
            
            # === Check for collisions involving our vehicles ===
            if not collision_detected:
                recent_collisions = collision_detector.get_recent_collisions(time_window=1.0)
                
                for collision in recent_collisions:
                    # Check if either of our vehicles is involved in the collision
                    vehicle1_id = vehicle1.id if vehicle1 else None
                    vehicle2_id = vehicle2.id if vehicle2 else None
                    
                    if ((collision['actor1_id'] == vehicle1_id or collision['actor1_id'] == vehicle2_id) or
                        (collision['actor2_id'] == vehicle1_id or collision['actor2_id'] == vehicle2_id)):
                        
                        collision_detected = True
                        collision_time = relative_time
                        
                        logger.info(f"COLLISION DETECTED at time {relative_time:.2f}s!")
                        logger.info(f"Collision between Actor {collision['actor1_id']} ({collision['actor1_type']}) and Actor {collision['actor2_id']} ({collision['actor2_type']})")
                        print(f"\n=== COLLISION DETECTED ===")
                        print(f"Time: {relative_time:.2f}s")
                        print(f"Actors: {collision['actor1_type']} vs {collision['actor2_type']}")
                        print(f"Location: x={collision['location']['x']:.2f}, y={collision['location']['y']:.2f}")
                        print("Stopping vehicles...")
                        break
            
            # === Stop vehicles after collision ===
            if collision_detected and not vehicles_stopped:
                if vehicle1 and vehicle1.is_alive:
                    vehicle1.set_autopilot(False)
                    # Apply brakes and stop the vehicle
                    control = carla.VehicleControl()
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.hand_brake = True
                    vehicle1.apply_control(control)
                    
                if vehicle2 and vehicle2.is_alive:
                    vehicle2.set_autopilot(False)
                    # Apply brakes and stop the vehicle
                    control = carla.VehicleControl()
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.hand_brake = True
                    vehicle2.apply_control(control)
                
                vehicles_stopped = True
                logger.info("Both vehicles stopped after collision")

                if walker1 and walker1.is_alive and not walker_moving_to_destination:
                    target_location = spawn_points[118].location
                    
                    # Use manual walker control instead of AI controller
                    walker_moving_to_destination = True
                    logger.info(f"Walker moving manually to spawn_points[118]: {target_location}")

            # Add this in the main loop for manual walker movement:
            if walker_moving_to_destination and not walker_at_destination and walker1:
                current_location = walker1.get_location()
                target_location = carla.Location(x=-142.0, y=398.0, z=0.0)
                
                # Calculate direction to target
                direction = target_location - current_location
                distance = direction.length()
                logger.info(f"Distance to destination: {distance:.2f}m")
                
                if distance > 2.0:  # Still moving to destination
                    # Normalize direction
                    direction = direction / distance
                    
                    # Apply walker control
                    walker_control = carla.WalkerControl()
                    walker_control.speed = 1.4  # Walking speed
                    walker_control.direction = carla.Vector3D(direction.x, direction.y, 0)
                    walker1.apply_control(walker_control)
                    
                else:
                    # Reached destination
                    walker_control = carla.WalkerControl()
                    walker_control.speed = 0.0
                    walker1.apply_control(walker_control)
                    walker1.blend_pose(True)  # T-pose
                    walker_at_destination = True
                    logger.info("Walker reached destination and T-posed!")
                                        
            # === Start vehicles only if no collision has been detected ===
            if not collision_detected:
                if vehicle1 and not started_1 and relative_time >= vehicle1_start_time:
                    vehicle1.set_autopilot(True)
                    started_1 = True
                    logger.info(f"Vehicle1 autopilot started at time {relative_time:.2f}s")
                
                if vehicle2 and not started_2 and relative_time >= vehicle2_start_time:
                    vehicle2.set_autopilot(True, traffic_manager.get_port())
                    started_2 = True
                    logger.info(f"Vehicle2 autopilot started at time {relative_time:.2f}s")
            
            # === Record trajectories for all alive actors ===
            current_frame = trajectory_recorder.frame_count
            
            # Get vehicles using actor manager cache
            all_vehicle_actors = trajectory_recorder.actor_manager.get_vehicles(current_frame)
            for actor in all_vehicle_actors:
                try:
                    if actor.is_alive:
                        trajectory_recorder.record_actor(actor, elapsed_time, actor_type="vehicle")
                except:
                    continue
            
            # Get walkers using actor manager cache
            all_walker_actors = trajectory_recorder.actor_manager.get_walkers(current_frame)
            for actor in all_walker_actors:
                try:
                    if actor.is_alive:
                        trajectory_recorder.record_actor(actor, elapsed_time, actor_type="walker")
                except:
                    continue
            
            # Update trajectory recorder
            trajectory_recorder.tick()
            
            # Update collision detection
            collision_detector.update(current_frame=frame_count)
            
            frame_count += 1
            
            # Log progress every 100 frames
            if frame_count % 100 == 0:
                status = "COLLISION - VEHICLES STOPPED" if collision_detected else "RUNNING"
                logger.info(f"Frame {frame_count}, Elapsed time: {elapsed_time:.2f}s, Relative time: {relative_time:.2f}s - Status: {status}")
                
                # Print trajectory stats
                stats = trajectory_recorder.get_stats()
                logger.info(f"Trajectories recorded: {stats['vehicle_count']} vehicles, {stats['walker_count']} walkers")
                
                # Print collision stats
                collision_stats = collision_detector.get_collision_statistics()
                logger.info(f"Collisions detected: {collision_stats['total_collisions']}")

    except KeyboardInterrupt:
        print("\nStopping simulation...")

    finally:
        logger.info("Simulation ended, saving trajectory data...")
        
        if collision_detected:
            logger.info(f"Final status: COLLISION OCCURRED at time {collision_time:.2f}s - Vehicles were stopped")
        else:
            logger.info("Final status: No collision detected during simulation")
        
        # Save combined trajectory and collision data
        try:
            combined_data = save_combined_data(
                trajectory_recorder, 
                collision_detector, 
                "output/scenario1_crash_data.pkl"
            )
            
            logger.info(f"Data saved successfully!")
            logger.info(f"Trajectories: {len(combined_data['trajectories']['vehicles'])} vehicles, {len(combined_data['trajectories']['walkers'])} walkers")
            logger.info(f"Collisions: {len(combined_data['collisions']['events'])} events")
            
        except Exception as e:
            logger.error(f"Failed to save data: {e}")
            
            # Fallback: save just trajectories
            try:
                trajectory_recorder.save()
                logger.info("Trajectory data saved as fallback")
            except Exception as e2:
                logger.error(f"Failed to save trajectory fallback: {e2}")
        
        # Cleanup
        print("Cleaning up actors...")
        
        # Cleanup recorders
        if collision_detector:
            collision_detector.cleanup()
        if trajectory_recorder:
            trajectory_recorder.cleanup()
        
        # Destroy spawned actors
        for v in spawned_vehicles:
            if v and v.is_alive:
                v.destroy()
                print(f"Destroyed {v.type_id}")
        
        print("=== Done ===")

if __name__ == '__main__':
    main()