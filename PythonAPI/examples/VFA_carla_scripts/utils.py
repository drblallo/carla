#!/usr/bin/env python

"""
Utilities for CARLA traffic generation and simulation
"""

import os
import time
import pickle
import math
import datetime
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Any, Dict
from enum import Enum
import numpy
import yaml
import random

import carla  # type: ignore

from trajectory_recorder import TrajectoryRecorder, RecordingMode
from collision_detector import CollisionDetector, CollisionMethod
from carla_step_manager import CarlaStepClient

# Setup logging
logger = logging.getLogger(__name__)


class SimulationMode(Enum):
    SYNCHRONOUS = "synchronous"
    ASYNCHRONOUS = "asynchronous"


@dataclass
class SimulationConfig:
    """Main simulation configuration"""
    host: str = "127.0.0.1"
    port: int = 2000
    tm_port: int = 8000
    timeout: float = 10.0
    asynch: bool = True
    no_rendering: bool = False
    simulation_fps: float = 60.0
    seed: Optional[int] = None
    hybrid: bool = False
    respawn: bool = False
    global_speed_difference: float = 30.0
    distance_to_leading_vehicle: float = 2.5


@dataclass 
class VehicleConfig:
    """Vehicle spawning configuration"""
    number: int = 20
    filter: str = "vehicle.*"
    generation: str = "All"
    safe_mode: bool = False
    car_lights_on: bool = False
    hero: bool = False
    speed_variance: float = 1.0


@dataclass
class WalkerConfig:
    """Walker/pedestrian configuration"""
    number: int = 30
    filter: str = "walker.pedestrian.*"
    generation: str = "2"
    seed: int = 0
    percentage_running: float = 0.2
    percentage_crossing: float = 1.0


@dataclass
class TrajectoryConfig:
    """Trajectory recording configuration"""
    enabled: bool = True
    output_file: str = "output/simulation_data.pkl"
    save_interval: int = 300
    record_bbox: bool = False
    recording_mode: str = "tick_based"
    frame_interval: int = 2
    recording_frequency: int = 10
    enable_gps: bool = True


@dataclass
class CollisionConfig:
    """Collision detection configuration"""
    enabled: bool = True
    method: str = "hybrid"
    distance_threshold: float = 2.0
    update_interval: int = 3


@dataclass
class V2XConfig:
    """V2X communication configuration"""
    enabled: bool = True
    frequency: int = 10
    log_file: str = "00_v2x_carla_traffic.log" 
    station_id: int = 123


@dataclass
class PerformanceConfig:
    """Performance optimization configuration"""
    actor_cache_interval: int = 50


@dataclass
class CarlaConfig:
    """Complete CARLA simulation configuration"""
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    vehicles: VehicleConfig = field(default_factory=VehicleConfig)
    walkers: WalkerConfig = field(default_factory=WalkerConfig)
    trajectory_recording: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    collision_detection: CollisionConfig = field(default_factory=CollisionConfig)
    v2x: V2XConfig = field(default_factory=V2XConfig)
    performance: PerformanceConfig = field(default_factory=PerformanceConfig)


class WalkerWrapper:
    """Wrapper class for walker control"""
    
    def __init__(self, world, walker, walker_ai):
        self.world = world
        self.walker = walker
        self.walker_ai = walker_ai
        self.target_point = None
        self.target_waypoint = None

    def set_relative_target_location(self, x: float, y: float):
        """Set a target location relative to current walker position"""
        self.walker_ai.stop()
        self.walker_ai.start()
        self.target_waypoint = self.world.get_map().get_waypoint(
            carla.Location(self.walker.get_location().x + x, self.walker.get_location().y + y),
            project_to_road=True,
            lane_type=carla.LaneType.Sidewalk
        ).transform.location
        self.walker_ai.go_to_location(self.target_waypoint)

    def walker_reached_destination(self, threshold: float = 2.0) -> bool:
        """Check if walker reached the destination"""
        if self.target_waypoint is None:
            return False
        current_location = self.walker.get_location()
        distance = current_location.distance(self.target_waypoint)
        return distance < threshold

    def tick(self):
        """Update walker state each tick"""
        if self.walker_reached_destination():
            pass


def load_config(config_file: str) -> CarlaConfig:
    """Load configuration from YAML file"""
    try:
        with open(config_file, 'r') as f:
            config_dict = yaml.safe_load(f)
        
        # Convert nested dictionaries to dataclass instances
        config = CarlaConfig(
            simulation=SimulationConfig(**config_dict.get('simulation', {})),
            vehicles=VehicleConfig(**config_dict.get('vehicles', {})),
            walkers=WalkerConfig(**config_dict.get('walkers', {})),
            trajectory_recording=TrajectoryConfig(**config_dict.get('trajectory_recording', {})),
            collision_detection=CollisionConfig(**config_dict.get('collision_detection', {})),
            v2x=V2XConfig(**config_dict.get('v2x', {})),
            performance=PerformanceConfig(**config_dict.get('performance', {}))
        )
        
        logger.info(f"Configuration loaded from {config_file}")
        return config
        
    except FileNotFoundError:
        logger.warning(f"Config file {config_file} not found. Using default configuration.")
        return CarlaConfig()
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML config: {e}. Using default configuration.")
        return CarlaConfig()


def get_actor_blueprints(world, filter_pattern: str, generation: str):
    """Get actor blueprints based on filter and generation"""
    bps = world.get_blueprint_library().filter(filter_pattern)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            return []
    except:
        return []


def compute_actor_heading(actor) -> float:
    """
    Compute WGS84 heading from CARLA actor.
    
    Converts CARLA's yaw (0° = East) to WGS84 heading (0° = North) in degrees.
    Returns heading in range 0.0-360.0 degrees.
    
    Args:
        actor: CARLA actor (vehicle or walker)
        
    Returns:
        float: WGS84 heading in degrees
    """
    carla_yaw = actor.get_transform().rotation.yaw
    return (90.0 - carla_yaw) % 360.0


def send_vehicle_cam(v2x_client: CarlaStepClient, vehicle_actor):
    """Send CAM message for a vehicle"""
    try:
        # Get GPS coordinates
        geo_loc = vehicle_actor.get_geolocation()

        velocity = vehicle_actor.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # m/s

        heading = compute_actor_heading(vehicle_actor)
        
        v2x_client.send_message(
            latitude=geo_loc.latitude,
            longitude=geo_loc.longitude,
            speed=speed,
            heading=heading,
            id=vehicle_actor.id
        )
        
    except Exception as e:
        logger.error(f"Error sending vehicle CAM for actor {vehicle_actor.id}: {e}")


def send_walker_cam(v2x_client: CarlaStepClient, walker_actor):
    """Send CAM message for a walker (pedestrian)"""
    try:
        # Get GPS coordinates  
        geo_loc = walker_actor.get_geolocation()
        
        velocity = walker_actor.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        heading = compute_actor_heading(walker_actor)
        
        v2x_client.send_message(
            latitude=geo_loc.latitude,
            longitude=geo_loc.longitude,
            speed=speed,
            heading=heading,
            id=walker_actor.id
        )
        
    except Exception as e:
        logger.error(f"Error sending walker CAM for actor {walker_actor.id}: {e}")


def save_combined_data(trajectory_recorder, collision_detector, filename: str) -> Dict[str, Any]:
    """Save both trajectory and collision data to a pickle file"""
    
    logger.info(f"Saving combined data to {filename}")
    
    # Get clean trajectory data
    clean_vehicles, clean_walkers = trajectory_recorder.clean_trajectories()
    
    # Base datetime: July 21, 2025 at 00:00:00
    base_datetime = datetime.datetime(2025, 7, 21, 0, 0, 0)
    
    # Process vehicle trajectories to add proper timestamps
    processed_vehicles = {}
    for vehicle_id, trajectory in clean_vehicles.items():
        processed_trajectory = []
        for state in trajectory:
            # Create new state dict with modified timestamps
            new_state = state.copy()
            
            # Rename original timestamp to elapsed_time
            new_state['elapsed_time'] = state['timestamp']
            
            # Create new timestamp with datetime from base + elapsed_time (precision 0.01s)
            elapsed_seconds = round(state['timestamp'], 2)  # 0.01s precision
            new_state['timestamp'] = base_datetime + datetime.timedelta(seconds=elapsed_seconds)
            
            # Create timestamp_real using real_time
            real_seconds = round(state['real_time'], 2)  # 0.01s precision
            new_state['timestamp_real'] = base_datetime + datetime.timedelta(seconds=real_seconds)
            
            processed_trajectory.append(new_state)
        processed_vehicles[vehicle_id] = processed_trajectory
    
    # Process walker trajectories to add proper timestamps
    processed_walkers = {}
    for walker_id, trajectory in clean_walkers.items():
        processed_trajectory = []
        for state in trajectory:
            # Create new state dict with modified timestamps
            new_state = state.copy()
            
            # Rename original timestamp to elapsed_time
            new_state['elapsed_time'] = state['timestamp']
            
            # Create new timestamp with datetime from base + elapsed_time (precision 0.01s)
            elapsed_seconds = round(state['timestamp'], 2)  # 0.01s precision
            new_state['timestamp'] = base_datetime + datetime.timedelta(seconds=elapsed_seconds)
            
            # Create timestamp_real using real_time
            real_seconds = round(state['real_time'], 2)  # 0.01s precision
            new_state['timestamp_real'] = base_datetime + datetime.timedelta(seconds=real_seconds)
            
            processed_trajectory.append(new_state)
        processed_walkers[walker_id] = processed_trajectory
    
    # Get collision statistics and events
    collision_stats = collision_detector.get_collision_statistics() if collision_detector else {}
    collision_events = collision_detector.collision_events if collision_detector else []
    
    # Process collision events to add proper timestamps
    processed_collision_events = []
    for event in collision_events:
        new_event = event.copy()
        if 'timestamp' in event:
            # Rename original timestamp to elapsed_time
            new_event['elapsed_time'] = event['timestamp']
            
            # Create new timestamp with datetime from base + elapsed_time
            elapsed_seconds = round(event['timestamp'], 2)  # 0.01s precision
            new_event['timestamp'] = base_datetime + datetime.timedelta(seconds=elapsed_seconds)
            
            # If there's a real_time field, create timestamp_real
            if 'real_time' in event:
                real_seconds = round(event['real_time'], 2)
                new_event['timestamp_real'] = base_datetime + datetime.timedelta(seconds=real_seconds)
        
        processed_collision_events.append(new_event)
    
    # Combine all data
    combined_data = {
        'trajectories': {
            'vehicles': processed_vehicles,
            'walkers': processed_walkers,
            'frame_count': trajectory_recorder.frame_count,
            'recording_mode': trajectory_recorder.recording_mode.value,
            'original_recording_mode': trajectory_recorder.original_recording_mode.value,
            'auto_switched': trajectory_recorder.auto_switched,
            'recording_frequency': trajectory_recorder.frequency,
            'frame_interval': trajectory_recorder.frame_interval,
            'sim_time_between_records': trajectory_recorder.sim_time_between_records,
        },
        'collisions': {
            'events': processed_collision_events,
            'statistics': collision_stats,
            'detection_method': collision_detector.detection_method.value if collision_detector else None,
            'total_collisions': collision_detector.total_collisions if collision_detector else 0,
            'collisions_by_type': dict(collision_detector.collisions_by_type) if collision_detector else {}
        },
        'metadata': {
            'save_timestamp': time.time(),
            'save_datetime': datetime.datetime.now(),
            'base_simulation_datetime': base_datetime,
            'total_simulation_time': time.time() - trajectory_recorder.start_time,
            'recording_settings': {
                'record_bbox': trajectory_recorder.record_bbox,
                'distance_threshold': getattr(collision_detector, 'distance_threshold', None) if collision_detector else None
            }
        }
    }
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(os.path.abspath(filename)) if os.path.dirname(filename) else '.', exist_ok=True)
    
    # Save to pickle file
    with open(filename, 'wb') as f:
        pickle.dump(combined_data, f)
    
    logger.info(f"Data saved successfully: {len(processed_vehicles)} vehicles, {len(processed_walkers)} walkers, {len(processed_collision_events)} collision events")
    
    return combined_data


def setup_world_settings(world, config: CarlaConfig, synchronous_master: bool = False):
    """Configure CARLA world settings based on configuration"""
    settings = world.get_settings()
    
    if not config.simulation.asynch:
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / config.simulation.simulation_fps
        else:
            synchronous_master = False

    if config.simulation.no_rendering:
        settings.no_rendering_mode = True
        
    world.apply_settings(settings)
    logger.debug(f"World settings applied: sync={settings.synchronous_mode}, no_render={config.simulation.no_rendering}")
    return synchronous_master


def setup_traffic_manager(client, config: CarlaConfig):
    """Setup and configure traffic manager"""
    traffic_manager = client.get_trafficmanager(config.simulation.tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(config.simulation.distance_to_leading_vehicle)
    
    if config.simulation.respawn:
        traffic_manager.set_respawn_dormant_vehicles(True)
    if config.simulation.hybrid:
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_hybrid_physics_radius(70.0)
    if config.simulation.seed is not None:
        traffic_manager.set_random_device_seed(config.simulation.seed)
        
    logger.debug(f"Traffic manager configured on port {config.simulation.tm_port}")
    return traffic_manager


def initialize_recorders(world, config: CarlaConfig, settings):
    """Initialize trajectory recorder and collision detector"""
    trajectory_recorder = None
    collision_detector = None
    
    # Initialize trajectory recorder
    if config.trajectory_recording.enabled:
        trajectory_recorder = TrajectoryRecorder(
            world=world,
            save_interval=config.trajectory_recording.save_interval,
            filename=config.trajectory_recording.output_file.replace('.pkl', '_trajectories_temp.pkl'),
            record_bbox=config.trajectory_recording.record_bbox,
            recording_mode=RecordingMode(config.trajectory_recording.recording_mode),
            frequency=config.trajectory_recording.recording_frequency,
            frame_interval=config.trajectory_recording.frame_interval,
            fixed_delta_seconds=settings.fixed_delta_seconds if not config.simulation.asynch else 0.05,
            enable_gps=config.trajectory_recording.enable_gps
        )
        logger.info(f"Trajectory recorder initialized with mode: {config.trajectory_recording.recording_mode}")

    # Initialize collision detector
    if config.collision_detection.enabled:
        collision_detector = CollisionDetector(
            world=world,
            detection_method=CollisionMethod(config.collision_detection.method),
            distance_threshold=config.collision_detection.distance_threshold,
            collision_update_interval=config.collision_detection.update_interval,
            actor_cache_interval=config.performance.actor_cache_interval
        )
        logger.info(f"Collision detector initialized with method: {config.collision_detection.method}")
        
    return trajectory_recorder, collision_detector


def print_performance_stats(frame_count: int, tempi_steps: List[float]):
    """Print performance statistics"""
    if tempi_steps:
        tempo_medio = sum(tempi_steps) / len(tempi_steps)
        tempo_min = min(tempi_steps)
        tempo_max = max(tempi_steps)
        
        logger.info(f"Performance Statistics:")
        logger.info(f"  Iterations: {frame_count}")
        logger.info(f"  Average step time: {tempo_medio:.6f} ms")
        logger.info(f"  Min time: {tempo_min:.6f} ms")
        logger.info(f"  Max time: {tempo_max:.6f} ms")


def cleanup_simulation(world, config: CarlaConfig, synchronous_master: bool, 
                      vehicles_list: List[int], all_id: List[int],
                      trajectory_recorder=None, collision_detector=None):
    """Clean up simulation resources"""
    
    logger.info("Starting cleanup process")
    
    # Cleanup recorders
    if collision_detector:
        collision_detector.cleanup()
        logger.debug("Collision detector cleaned up")
    if trajectory_recorder:
        trajectory_recorder.cleanup()
        logger.debug("Trajectory recorder cleaned up")
    
    # Restore world settings
    if not config.simulation.asynch and synchronous_master:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        logger.debug("World settings restored")

    # Clean up actors
    client = carla.Client(config.simulation.host, config.simulation.port)
    logger.info(f'Destroying {len(vehicles_list)} vehicles')
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

    logger.info(f'Destroying {len(all_id)} walkers')
    client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

    time.sleep(0.5)
    logger.info("Cleanup complete")

def spawn_vehicles(world, client, config: CarlaConfig, traffic_manager, collision_detector=None):
    """Spawn vehicles in the simulation"""
    logger.info(f"Spawning {config.vehicles.number} vehicles")
    
    blueprints = get_actor_blueprints(world, config.vehicles.filter, config.vehicles.generation)
    if not blueprints:
        logger.error("Couldn't find any vehicles with the specified filters")
        raise ValueError("Couldn't find any vehicles with the specified filters")

    if config.vehicles.safe_mode:
        blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
        logger.debug(f"Safe mode enabled, filtered to {len(blueprints)} car blueprints")

    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if config.vehicles.number < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif config.vehicles.number > number_of_spawn_points:
        logger.warning(f"Requested {config.vehicles.number} vehicles but only {number_of_spawn_points} spawn points available")
        config.vehicles.number = number_of_spawn_points

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # Spawn vehicles
    batch = []
    hero = config.vehicles.hero
    for n, transform in enumerate(spawn_points):
        if n >= config.vehicles.number:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if hero:
            blueprint.set_attribute('role_name', 'hero')
            hero = False
        else:
            blueprint.set_attribute('role_name', 'autopilot')

        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    vehicles_list = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logger.warning(f"Vehicle spawn error: {response.error}")
        else:
            vehicles_list.append(response.actor_id)

    logger.info(f"Successfully spawned {len(vehicles_list)} vehicles")

    # Set automatic vehicle lights and collision sensors
    if config.vehicles.car_lights_on:
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            traffic_manager.update_vehicle_lights(actor, True)
        logger.debug("Vehicle lights enabled")

    if collision_detector:
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            try:
                if actor.is_alive:
                    collision_detector.add_collision_sensor(actor)
                    traffic_manager.vehicle_percentage_speed_difference(
                        actor, numpy.random.normal(loc=0, scale=config.vehicles.speed_variance, size=None)
                    )
            except Exception as e:
                logger.debug(f"Failed to add collision sensor to vehicle {actor.id}: {e}")
                continue

    return vehicles_list


def spawn_walkers(world, client, config: CarlaConfig, collision_detector=None):
    """Spawn walkers (pedestrians) in the simulation"""
    logger.info(f"Spawning {config.walkers.number} walkers")
    
    blueprintsWalkers = get_actor_blueprints(world, config.walkers.filter, config.walkers.generation)
    if not blueprintsWalkers:
        logger.error("Couldn't find any walkers with the specified filters")
        raise ValueError("Couldn't find any walkers with the specified filters")

    if config.walkers.seed:
        world.set_pedestrians_seed(config.walkers.seed)
        random.seed(config.walkers.seed)
        logger.debug(f"Walker seed set to {config.walkers.seed}")

    spawn_points = []
    for i in range(config.walkers.number):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spawn_point.location = loc
            spawn_points.append(spawn_point)

    logger.debug(f"Found {len(spawn_points)} valid spawn points for walkers")

    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        probability = random.randint(0, 101)
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        if walker_bp.has_attribute('can_use_wheelchair') and probability < 11:
            walker_bp.set_attribute('use_wheelchair', 'true')
        if walker_bp.has_attribute('speed'):
            if random.random() > config.walkers.percentage_running:
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            walker_speed.append(0.0)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    walkers_list = []
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logger.warning(f"Walker spawn error: {results[i].error}")
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2

    logger.info(f"Successfully spawned {len(walkers_list)} walkers")

    # Spawn walker controllers
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    
    results = client.apply_batch_sync(batch, True)
    all_id = []
    for i in range(len(results)):
        if results[i].error:
            logger.warning(f"Walker controller spawn error: {results[i].error}")
        else:
            walkers_list[i]["con"] = results[i].actor_id

    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])

    all_actors = world.get_actors(all_id)

    # Add collision sensors to walkers
    if collision_detector:
        all_walker_actors = world.get_actors([w["id"] for w in walkers_list])
        for actor in all_walker_actors:
            try:
                if actor.is_alive:
                    collision_detector.add_collision_sensor(actor)
            except Exception as e:
                logger.debug(f"Failed to add collision sensor to walker {actor.id}: {e}")
                continue

    # Start walker AI
    world.set_pedestrians_cross_factor(config.walkers.percentage_crossing)
    for i in range(0, len(all_id), 2):
        all_actors[i].start()
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    logger.info(f"Walker AI controllers started with {config.walkers.percentage_crossing*100:.0f}% crossing factor")

    return walkers_list, all_id