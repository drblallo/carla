#!/usr/bin/env python

"""
Streamlined trajectory recording module with manual GPS conversion
"""

import os
import time
import pickle
import math
import logging
from collections import defaultdict
from enum import Enum
import numpy as np
import carla # type: ignore

from actor_manager import ActorManager

# Setup logging
logger = logging.getLogger(__name__)


class RecordingMode(Enum):
    SIMULATION_TIME = "simulation_time"
    FRAME_BASED = "frame_based"
    HYBRID = "hybrid"
    TICK_BASED = "tick_based"


def compute_actor_heading(actor):
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


class TrajectoryRecorder:
    """
    Trajectory recorder with manual GPS conversion
    """
    
    def __init__(self, world, save_interval=300, filename="trajectories.pkl", record_bbox=False, 
                 recording_mode=RecordingMode.TICK_BASED, frequency=10.0, frame_interval=1, 
                 fixed_delta_seconds=0.05, actor_cache_interval=30, enable_gps=True):
        self.world = world
        self.vehicle_trajectories = defaultdict(list)
        self.walker_trajectories = defaultdict(list)
        
        # Individual actor tracking
        self.vehicle_last_record_time = {}
        self.walker_last_record_time = {}
        self.vehicle_last_record_frame = {}
        self.walker_last_record_frame = {}
        self.vehicle_last_record_tick = {}
        self.walker_last_record_tick = {}
        
        self.frame_count = 0
        self.save_interval = save_interval
        self.filename = filename
        self.start_time = time.time()
        self.record_bbox = record_bbox
        self.enable_gps = enable_gps
        
        # Initialize ActorManager
        self.actor_manager = ActorManager(world, update_interval=actor_cache_interval)
        
        # Initialize map for GPS conversion
        if self.enable_gps:
            self._map = world.get_map()
        
        # Auto-detect async mode
        self.original_recording_mode = recording_mode
        self.recording_mode, self.auto_switched = self._detect_and_configure_mode(recording_mode)
        
        # Recording strategy configuration
        self.frequency = frequency
        self.frame_interval = frame_interval
        self.sim_time_between_records = 1.0 / frequency if frequency > 0 else 0.1
        self.waiting_ticks = int((1/fixed_delta_seconds)/self.frequency)
        
        # Async mode tolerance
        if self.auto_switched:
            self.time_tolerance = self.sim_time_between_records * 0.1
        else:
            self.time_tolerance = 0.0
        
        # For hybrid mode
        self.last_global_record_time = 0.0
        self.last_global_record_frame = 0
        
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(filename)) if os.path.dirname(filename) else '.', exist_ok=True)
        
        logger.info(f"TrajectoryRecorder initialized: mode={recording_mode.value}, frequency={frequency}Hz, save_interval={save_interval}, GPS={enable_gps}")
    
    def _detect_and_configure_mode(self, requested_mode):
        try:
            settings = self.world.get_settings()
            is_async = not settings.synchronous_mode
            
            if is_async:
                if requested_mode != RecordingMode.SIMULATION_TIME:
                    logger.warning(f"Async mode detected, switching from {requested_mode.value} to SIMULATION_TIME mode")
                    return RecordingMode.SIMULATION_TIME, True
                else:
                    return RecordingMode.SIMULATION_TIME, False
            else:
                return requested_mode, False
                
        except Exception as e:
            logger.error(f"Failed to detect simulation mode: {e}")
            return requested_mode, False
    
    def get_gps_location(self, location):
        if not self.enable_gps:
            return None
        
        try:
            geo_location = self._map.transform_to_geolocation(location)
            return {
                'latitude': geo_location.latitude,
                'longitude': geo_location.longitude,
                'altitude': geo_location.altitude
            }
        except Exception as e:
            logger.debug(f"GPS conversion failed: {e}")
            return None
    
    def should_record_now(self, actor_id, current_sim_time, current_frame, actor_type="vehicle"):
        if self.auto_switched or self.recording_mode == RecordingMode.SIMULATION_TIME:
            return self._should_record_by_time(actor_id, current_sim_time, actor_type)
        elif self.recording_mode == RecordingMode.FRAME_BASED:
            return self._should_record_by_frame(actor_id, current_frame, actor_type)
        elif self.recording_mode == RecordingMode.HYBRID:
            return (self._should_record_by_time(actor_id, current_sim_time, actor_type) or 
                    self._should_record_by_frame(actor_id, current_frame, actor_type))
        elif self.recording_mode == RecordingMode.TICK_BASED:
            return self._should_record_by_ticks(actor_id, self.frame_count, actor_type)
        
        return False
    
    def record_actor(self, actor, elapsed_time, actor_type="vehicle"):
        actor_id = actor.id
        
        if not self.should_record_now(actor_id, elapsed_time, self.frame_count, actor_type):
            return False
        
        # Get actor data
        try:
            transform = actor.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = actor.get_velocity()
            angular_velocity = actor.get_angular_velocity()
        except Exception as e:
            logger.debug(f"Failed to get actor {actor_id} data: {e}")
            return False
        
        # Get GPS coordinates
        geo_loc = actor.get_geolocation()
        geo_location = {
            'latitude': geo_loc.latitude,
            'longitude': geo_loc.longitude,
            'altitude': geo_loc.altitude
        }
        
        # Calculate speed
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        heading = compute_actor_heading(actor)
        
        # Create state dictionary
        state = {
            'timestamp': elapsed_time, # Elapsed sim time
            'frame': self.frame_count,
            'real_time': time.time() - self.start_time, # Elapsed real time
            'type_id': actor.type_id,
            'location': {
                'x': location.x,
                'y': location.y,
                'z': location.z
            },
            'geo_location': geo_location,
            'heading': heading,
            'rotation': {
                'pitch': rotation.pitch,
                'yaw': rotation.yaw,
                'roll': rotation.roll
            },
            'velocity': {
                'x': velocity.x,
                'y': velocity.y,
                'z': velocity.z,
                'speed': speed
            },
            'angular_velocity': {
                'x': angular_velocity.x,
                'y': angular_velocity.y,
                'z': angular_velocity.z
            }
        }
        
        # Add bounding box if requested
        if self.record_bbox:
            try:
                bbox = actor.bounding_box
                state['bbox'] = {
                    'extent': {
                        'x': bbox.extent.x,
                        'y': bbox.extent.y,
                        'z': bbox.extent.z
                    },
                    'location': {
                        'x': bbox.location.x,
                        'y': bbox.location.y,
                        'z': bbox.location.z
                    }
                }
            except Exception as e:
                logger.debug(f"Failed to get bbox for actor {actor_id}: {e}")
        
        # Add to trajectory
        if actor_type == "vehicle":
            self.vehicle_trajectories[actor_id].append(state)
        else:
            self.walker_trajectories[actor_id].append(state)
        
        return True
    
    def _should_record_by_time(self, actor_id, current_sim_time, actor_type):
        last_record_dict = self.vehicle_last_record_time if actor_type == "vehicle" else self.walker_last_record_time
        
        if actor_id not in last_record_dict:
            last_record_dict[actor_id] = current_sim_time
            return True
        
        time_diff = current_sim_time - last_record_dict[actor_id]
        min_time_threshold = self.sim_time_between_records - self.time_tolerance
        
        if time_diff >= min_time_threshold:
            last_record_dict[actor_id] = current_sim_time
            return True
        
        return False
    
    def _should_record_by_ticks(self, actor_id, current_tick, actor_type):
        last_record_dict = self.vehicle_last_record_tick if actor_type == "vehicle" else self.walker_last_record_tick
        
        if actor_id not in last_record_dict or (current_tick-last_record_dict[actor_id]) >= self.waiting_ticks:
            last_record_dict[actor_id] = current_tick
            return True
        return False

    def _should_record_by_frame(self, actor_id, current_frame, actor_type):
        last_record_dict = self.vehicle_last_record_frame if actor_type == "vehicle" else self.walker_last_record_frame
        
        if actor_id not in last_record_dict or (current_frame - last_record_dict[actor_id]) >= self.frame_interval:
            last_record_dict[actor_id] = current_frame
            return True
        return False
    
    def tick(self):
        self.frame_count += 1
        
        # Update actor manager
        self.actor_manager.set_current_frame(self.frame_count)
        
        # Save periodically
        if self.frame_count % self.save_interval == 0:
            logger.debug(f"Auto-saving trajectories at frame {self.frame_count}")
            self.save()
    
    def clean_trajectories(self):
        min_points = 3
        
        # Clean vehicle trajectories
        cleaned_vehicle_trajectories = {}
        #print("numero di veicoli in clean:")
        #print(self.vehicle_trajectories.keys())
        for vehicle_id, traj in self.vehicle_trajectories.items():
            if len(traj) < min_points:
                print(f"elimino {vehicle_id} per mintraj")
                continue
                
            # Find where the vehicle starts moving
            start_idx = 0
            if len(traj) > 1:
                for i in range(len(traj)):
                    if np.sqrt(traj[i]['velocity']['x']**2 + traj[i]['velocity']['y']**2) > 0.1:
                        start_idx = max(0, i-1)
                        break
            
            # Find deletion point (z=0)
            end_idx = 0
            for i in range(len(traj)-1, start_idx, -1):
                if abs(traj[i]['location']['z']) > 0.00001:
                    end_idx = i
                    break
                    
            if end_idx > start_idx + 1 and end_idx - start_idx >= min_points:
                cleaned_vehicle_trajectories[vehicle_id] = traj[start_idx:end_idx]
                #print(vehicle_id)
                #print(f"start: {start_idx}, end: {end_idx}")
            else:
                pass
                #print(f"elimino {vehicle_id} deletion point (z=0), start: {start_idx}, end: {end_idx}")
        
        # Clean walker trajectories
        cleaned_walker_trajectories = {}
        for walker_id, traj in self.walker_trajectories.items():
            if len(traj) < min_points:
                continue
                
            start_idx = 0
            if len(traj) > 1:
                for i in range(len(traj)):
                    if traj[i]['velocity']['speed'] > 0.1:
                        start_idx = max(0, i-1)
                        break

            end_idx = 0
            for i in range(len(traj)-1, start_idx, -1):
                if abs(traj[i]['location']['z']) > 0.00001:
                    end_idx = i
                    break
                    
            if end_idx > start_idx + 1 and end_idx - start_idx >= min_points:
                cleaned_walker_trajectories[walker_id] = traj[start_idx:end_idx]
        
        logger.debug(f"Cleaned trajectories: {len(cleaned_vehicle_trajectories)} vehicles, {len(cleaned_walker_trajectories)} walkers")
        return cleaned_vehicle_trajectories, cleaned_walker_trajectories
            
    def save(self):
        clean_vehicles, clean_walkers = self.clean_trajectories()
        
        data = {
            'vehicles': clean_vehicles,
            'walkers': clean_walkers,
            'frame_count': self.frame_count,
            'total_time': time.time() - self.start_time,
            'recording_mode': self.recording_mode.value,
            'original_recording_mode': self.original_recording_mode.value,
            'auto_switched': self.auto_switched,
            'recording_frequency': self.frequency,
            'frame_interval': self.frame_interval,
            'sim_time_between_records': self.sim_time_between_records,
            'gps_enabled': self.enable_gps,
            'gps_conversion_method': 'carla_api',
            'actor_cache_stats': self.actor_manager.get_cache_stats()
        }
        
        with open(self.filename, 'wb') as f:
            pickle.dump(data, f)
        
        logger.info(f"Saved trajectories to {self.filename}: {len(clean_vehicles)} vehicles, {len(clean_walkers)} walkers")
    
    def get_stats(self):
        clean_vehicles, clean_walkers = self.clean_trajectories()
        vehicle_count = len(clean_vehicles)
        walker_count = len(clean_walkers)
        
        avg_vehicle_len = sum(len(traj) for traj in clean_vehicles.values()) / max(1, vehicle_count)
        avg_walker_len = sum(len(traj) for traj in clean_walkers.values()) / max(1, walker_count)
        
        stats = {
            'vehicle_count': vehicle_count,
            'walker_count': walker_count,
            'frame_count': self.frame_count,
            'total_time': time.time() - self.start_time,
            'avg_vehicle_trajectory_length': avg_vehicle_len,
            'avg_walker_trajectory_length': avg_walker_len,
            'recording_mode': self.recording_mode.value,
            'original_recording_mode': self.original_recording_mode.value,
            'auto_switched': self.auto_switched,
            'recording_frequency': self.frequency,
            'gps_enabled': self.enable_gps,
            'gps_conversion_method': 'carla_api',
            'actor_cache_stats': self.actor_manager.get_cache_stats()
        }
        
        return stats
    
    def reset(self):
        logger.info("Resetting trajectory recorder")
        self.vehicle_trajectories = defaultdict(list)
        self.walker_trajectories = defaultdict(list)
        self.vehicle_last_record_time = {}
        self.walker_last_record_time = {}
        self.vehicle_last_record_frame = {}
        self.walker_last_record_frame = {}
        self.vehicle_last_record_tick = {}
        self.walker_last_record_tick = {}
        self.frame_count = 0
        self.start_time = time.time()
        
        if hasattr(self, 'actor_manager'):
            self.actor_manager.reset_stats()
    
    def cleanup(self):
        if self.frame_count > 0:
            try:
                self.save()
            except Exception as e:
                logger.error(f"Failed to save trajectories during cleanup: {e}")
        
        if hasattr(self, 'actor_manager'):
            self.actor_manager.cleanup()
        
        logger.info("Trajectory recorder cleanup complete")


def load_trajectories(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)


def extract_vehicle_paths(trajectory_data):
    paths = {}
    for vehicle_id, states in trajectory_data['vehicles'].items():
        paths[vehicle_id] = {
            'x': [state['location']['x'] for state in states],
            'y': [state['location']['y'] for state in states],
            'speed': [state['velocity']['speed'] for state in states],
            'time': [state['timestamp'] for state in states]
        }
    return paths


def extract_geo_paths(trajectory_data):
    geo_paths = {}
    for vehicle_id, states in trajectory_data['vehicles'].items():
        geo_paths[vehicle_id] = {
            'latitude': [state['geo_location']['latitude'] for state in states if state['geo_location']],
            'longitude': [state['geo_location']['longitude'] for state in states if state['geo_location']],
            'altitude': [state['geo_location']['altitude'] for state in states if state['geo_location']],
            'speed': [state['velocity']['speed'] for state in states if state['geo_location']]
        }
    return geo_paths