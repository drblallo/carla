#!/usr/bin/env python

import glob
import os
import sys
import time
import logging

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import carla  # type: ignore
from numpy import random

from utils import (
    CarlaConfig, load_config, get_actor_blueprints,
    send_vehicle_cam, send_walker_cam, save_combined_data,
    setup_world_settings, setup_traffic_manager, initialize_recorders,
    print_performance_stats, cleanup_simulation, spawn_vehicles, spawn_walkers
)
from carla_step_manager import CarlaStepClient

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('00_v2x_simulation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def run_simulation(config: CarlaConfig):
    """Main simulation loop"""
    vehicles_list = []
    walkers_list = []
    all_id = []

    # Setup random seed
    seed = config.simulation.seed if config.simulation.seed is not None else int(time.time())
    random.seed(seed)
    logger.info(f"Random seed set to: {seed}")

    # Connect to CARLA
    client = carla.Client(config.simulation.host, config.simulation.port)
    client.set_timeout(config.simulation.timeout)
    synchronous_master = False
    logger.info(f"Connected to CARLA server at {config.simulation.host}:{config.simulation.port}")

    # Initialize recorders
    trajectory_recorder = None
    collision_detector = None

    #activate all light of the cars   [PA]
    #light_state = carla.VehicleLightState(carla.VehicleLightState.All)

    world = None
    elapsed_time = 0
    frame_count = 0
    tempi_steps = []

    try:
        world = client.get_world()
        logger.info("World loaded successfully")

        # Setup traffic manager
        traffic_manager = setup_traffic_manager(client, config)
        logger.info("Traffic manager configured")

        # Configure simulation settings
        settings = world.get_settings()
        synchronous_master = setup_world_settings(world, config, synchronous_master)
        logger.info(f"Simulation settings configured - Synchronous: {not config.simulation.asynch}")

        if not config.simulation.asynch:
            traffic_manager.set_synchronous_mode(True)

        # Initialize recorders
        trajectory_recorder, collision_detector = initialize_recorders(world, config, settings)
        if trajectory_recorder:
            logger.info("Trajectory recorder initialized")
        if collision_detector:
            logger.info("Collision detector initialized")

        # Spawn vehicles and walkers
        vehicles_list = spawn_vehicles(world, client, config, traffic_manager, collision_detector)
        walkers_list, all_id = spawn_walkers(world, client, config, collision_detector)

        logger.info(f'Spawned {len(vehicles_list)} vehicles and {len(walkers_list)} walkers')
        traffic_manager.global_percentage_speed_difference(config.simulation.global_speed_difference)

        # Initialize V2X client
        v2x_client = None
        v2x_last_broadcast = 0.0
        v2x_broadcast_interval = 1.0 / config.v2x.frequency  # Convert Hz to interval

        if config.v2x.enabled:
            #v2x_client = CarlaStepClient(config.v2x.log_file, station_id=config.v2x.station_id)   [PA]
            v2x_client = CarlaStepClient(config.v2x.log_file)
            logger.info(f"V2X client initialized with frequency {config.v2x.frequency} Hz")

        # Wait for initial tick
        if config.simulation.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        logger.info("Starting main simulation loop")
        # Main simulation loop
        elapsed_time = 0
        frame_count = 0
        tempi_steps = []

        try:
            while True:
                start_time = time.perf_counter()

                # Handle both sync and async modes
                if not config.simulation.asynch and synchronous_master:
                    world.tick()
                    snapshot = world.get_snapshot()
                    elapsed_time = snapshot.timestamp.elapsed_seconds
                else:
                    #logger.warning("Start wait_for_tick") # [PA]
                    snapshot = world.wait_for_tick()
                    #logger.warning("End wait_for_tick") # [PA]
                    #logger.info("Start loop")
                    time.sleep(0.05) #perchè è stato inserito questo sleep [PA]
                    #logger.warning("End Sleep wait_for_tick") # [PA]
                    elapsed_time = snapshot.timestamp.elapsed_seconds

                # Trajectory recording
                if config.trajectory_recording.enabled and trajectory_recorder:
                    current_frame = trajectory_recorder.frame_count

                    # Get vehicles using cache
                    all_vehicle_actors = trajectory_recorder.actor_manager.get_vehicles(current_frame)
                    print(f"Get vehicles number: {len(all_vehicle_actors)} ")
                    for actor in all_vehicle_actors:
                        try:
                            if actor.is_alive:
                                trajectory_recorder.record_actor(actor, elapsed_time, actor_type="vehicle")
                                #actor.set_light_state(light_state) # turn on all light of the cars [PA]
                        except Exception as e:
                            print(e)

                    # Get walkers using cache
                    all_walker_actors = trajectory_recorder.actor_manager.get_walkers(current_frame)
                    for actor in all_walker_actors:
                        try:
                            if actor.is_alive:
                                trajectory_recorder.record_actor(actor, elapsed_time, actor_type="walker")
                        except:
                            continue

                    trajectory_recorder.tick()

                # Send V2X CAM messages
                if v2x_client and config.v2x.enabled:
                    current_time = time.time()
                    if current_time - v2x_last_broadcast >= v2x_broadcast_interval:
                        v2x_last_broadcast = current_time

                        # Use existing actor manager cache for efficiency
                        if trajectory_recorder:
                            vehicles = trajectory_recorder.actor_manager.get_vehicles()
                            walkers = trajectory_recorder.actor_manager.get_walkers()
                        else:
                            vehicles = world.get_actors().filter('vehicle.*')
                            walkers = world.get_actors().filter('walker.pedestrian.*')

                        # Send CAMs for vehicles
                        for vehicle in vehicles:
                            if vehicle.is_alive:
                                #print(vehicle.id)
                                send_vehicle_cam(v2x_client, vehicle)

                        for walker in walkers:
                            if walker.is_alive:
                                send_walker_cam(v2x_client, walker)

                # Update collision detection
                if collision_detector and config.collision_detection.enabled:
                    collision_detector.update(current_frame=frame_count)

                # Update actor lists less frequently
                if frame_count % config.performance.actor_cache_interval == 0:
                    if trajectory_recorder:
                        trajectory_recorder.actor_manager.force_update()
                    if collision_detector:
                        collision_detector.actor_manager.force_update()

                    # Update global actor lists
                    vehicles_actors = trajectory_recorder.actor_manager.cached_vehicles if trajectory_recorder else world.get_actors().filter('vehicle.*')
                    vehicles_list = [v.id for v in vehicles_actors if hasattr(v, 'is_alive') and v.is_alive]

                    walker_actors = trajectory_recorder.actor_manager.cached_walkers if trajectory_recorder else world.get_actors().filter('walker.pedestrian.*')
                    controller_actors = collision_detector.actor_manager.cached_controllers if collision_detector else world.get_actors().filter('controller.ai.walker')

                    # Rebuild walkers_list
                    walkers_list = []
                    for walker in walker_actors:
                        try:
                            if walker.is_alive:
                                controller = next((c for c in controller_actors if c.parent and c.parent.id == walker.id), None)
                                walkers_list.append({'id': walker.id, 'con': controller.id if controller else None})
                        except:
                            continue

                frame_count += 1
                end_time = time.perf_counter()
                tempo_ms = (end_time - start_time) * 1000  # Convert to milliseconds
                tempi_steps.append(tempo_ms)


                # Log progress periodically
                if frame_count % 100 == 0:
                    logger.debug(f"Frame {frame_count}, Elapsed time: {elapsed_time:.2f}s, Step time: {tempo_ms:.2f}ms")

        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
            pass

    finally:
        # Print performance statistics
        logger.info("Simulation ended, calculating performance statistics")
        print_performance_stats(frame_count, tempi_steps)

        # Save data
        if (config.trajectory_recording.enabled and trajectory_recorder) or (config.collision_detection.enabled and collision_detector):
            if trajectory_recorder and collision_detector:
                combined_data = save_combined_data(trajectory_recorder, collision_detector, config.trajectory_recording.output_file)
                logger.info(f"Data saved to: {config.trajectory_recording.output_file}")
                logger.info(f"Trajectories: {len(combined_data['trajectories']['vehicles'])} vehicles, {len(combined_data['trajectories']['walkers'])} walkers")
                logger.info(f"Collisions: {len(combined_data['collisions']['events'])} events")

            elif trajectory_recorder:
                trajectory_recorder.save()
                stats = trajectory_recorder.get_stats()
                logger.info(f"Trajectory data saved. {stats['vehicle_count']} vehicles, {stats['walker_count']} walkers")

            elif collision_detector:
                collision_stats = collision_detector.get_collision_statistics()
                collision_data = {
                    'collisions': {
                        'events': collision_detector.collision_events,
                        'statistics': collision_stats
                    }
                }
                import pickle
                with open(config.trajectory_recording.output_file, 'wb') as f:
                    pickle.dump(collision_data, f)
                logger.info(f"Collision data saved. {collision_stats['total_collisions']} collisions detected")

        # Cleanup simulation
        logger.info("Cleaning up simulation resources")
        cleanup_simulation(world, config, synchronous_master, vehicles_list, all_id,
                         trajectory_recorder, collision_detector)


def main():
    """Main entry point"""
    # Load configuration from YAML file
    config_file = "VFA_carla_scripts/config.yaml"
    if len(sys.argv) > 1:
        config_file = sys.argv[1]

    config = load_config(config_file)

    logger.info(f"Starting CARLA traffic simulation with config: {config_file}")
    logger.info(f"Vehicles: {config.vehicles.number}, Walkers: {config.walkers.number}")
    logger.info(f"V2X enabled: {config.v2x.enabled}, Trajectory recording: {config.trajectory_recording.enabled}")
    logger.info(f"Collision detection: {config.collision_detection.enabled}")

    try:
        run_simulation(config)
    except KeyboardInterrupt:
        logger.info("Simulation interrupted")
    except Exception as e:
        logger.error(f"Simulation error: {e}", exc_info=True)
        raise
    finally:
        logger.info('Simulation complete')


if __name__ == '__main__':
    main()
