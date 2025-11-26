#!/usr/bin/env python

"""
Streamlined Collision Detection Module for CARLA Simulator
"""

import time
import weakref
import logging
from collections import defaultdict
from enum import Enum
import numpy as np
import carla # type: ignore

from actor_manager import ActorManager

try:
    from scipy.spatial.distance import pdist, squareform
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Setup logging
logger = logging.getLogger(__name__)


class CollisionMethod(Enum):
    SENSOR_BASED = "sensor_based"
    BBOX_BASED = "bbox_based"
    DISTANCE_BASED = "distance_based"
    HYBRID = "hybrid"


class CollisionSeverity(Enum):
    MINOR = "minor"
    MODERATE = "moderate"
    SEVERE = "severe"


class CollisionDetector:
    """
    Streamlined collision detection system for CARLA simulation
    """
    
    def __init__(self, world, detection_method=CollisionMethod.SENSOR_BASED, 
                 distance_threshold=2.0, ignore_static_road_elements=True, 
                 collision_update_interval=3, actor_cache_interval=30):
        self.world = world
        self.detection_method = detection_method
        self.distance_threshold = distance_threshold
        self.ignore_static_road_elements = ignore_static_road_elements
        
        # Frequency control
        self.collision_update_interval = collision_update_interval
        self.last_collision_check_frame = 0
        
        # Initialize ActorManager
        self.actor_manager = ActorManager(world, update_interval=actor_cache_interval)
        
        # Define static elements to ignore
        self.ignored_static_types = {
            'static.road', 'static.unknown', 'static.terrain',
            'static.roadline', 'static.road.line', 'static.road.marking',
            'static.road.surface'
        } if ignore_static_road_elements else set()
        
        # Static elements that should be recorded
        self.meaningful_static_types = {
            'static.prop.streetsign', 'static.prop.streetlight',
            'static.prop.trafficlight', 'static.building',
            'static.prop.barrier', 'static.prop.fence',
            'static.prop.pole', 'static.prop.mailbox',
            'static.prop.bench', 'static.prop.table',
            'static.prop.chair', 'static.prop.dumpster',
            'static.prop.container', 'static.prop.billboard',
            'static.prop.bus_stop', 'static.prop.phone_booth',
            'static.prop.atm', 'static.prop.fountain',
            'static.prop.statue', 'static.prop.vegetation',
            'static.prop.tree', 'static.prop.bush'
        }
        
        # Collision tracking
        self.collision_events = []
        self.collision_sensors = {}
        self.actor_collisions = defaultdict(list)
        self.collision_pairs = set()
        self.collision_cooldown = {}
        self.cooldown_time = 1.0
        
        # Statistics
        self.total_collisions = 0
        self.collisions_by_type = defaultdict(int)
        self.start_time = time.time()
        
        logger.info(f"CollisionDetector initialized: method={detection_method.value}, threshold={distance_threshold}m, update_interval={collision_update_interval}")
        
        # Initialize detection method
        if detection_method in [CollisionMethod.SENSOR_BASED, CollisionMethod.HYBRID]:
            self._setup_sensor_detection()
    
    def _setup_sensor_detection(self):
        vehicles = self.actor_manager.get_vehicles(0)
        sensor_count = 0
        for vehicle in vehicles:
            if self.add_collision_sensor(vehicle):
                sensor_count += 1
        logger.debug(f"Added collision sensors to {sensor_count} vehicles")
    
    def add_collision_sensor(self, actor):
        if actor.id in self.collision_sensors:
            return False
        
        # Check if actor is still alive and valid
        try:
            if not actor.is_alive:
                return False
        except:
            return False
            
        try:
            blueprint = self.world.get_blueprint_library().find('sensor.other.collision')
            sensor = self.world.spawn_actor(blueprint, carla.Transform(), attach_to=actor)
            
            weak_self = weakref.ref(self)
            sensor.listen(lambda event: CollisionDetector._on_collision(weak_self, event, actor.id))
            
            self.collision_sensors[actor.id] = sensor
            return True
        except RuntimeError as e:
            # Actor might have been destroyed between validation and sensor creation
            logger.debug(f"Failed to add collision sensor to actor {actor.id}: {e}")
            return False
    
    @staticmethod
    def _on_collision(weak_self, collision_event, actor_id):
        self = weak_self()
        if self is None:
            return
        self._handle_collision_event(collision_event, actor_id)
    
    def _should_ignore_collision(self, actor_type):
        if not self.ignore_static_road_elements:
            return False
        
        for ignored_type in self.ignored_static_types:
            if actor_type.startswith(ignored_type):
                return True
        
        return False
    
    def _is_meaningful_static_collision(self, actor_type):
        if not actor_type.startswith('static.'):
            return True
        
        for meaningful_type in self.meaningful_static_types:
            if actor_type.startswith(meaningful_type):
                return True
        
        return not self._should_ignore_collision(actor_type)
    
    def _handle_collision_event(self, collision_event, actor_id):
        other_actor = collision_event.other_actor
        other_actor_type = other_actor.type_id
        
        if self._should_ignore_collision(other_actor_type):
            return
        
        if not self._is_meaningful_static_collision(other_actor_type):
            return
        
        impulse = collision_event.normal_impulse
        impulse_magnitude = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        
        severity = self._classify_collision_severity(impulse_magnitude)
        
        collision_pair = tuple(sorted([actor_id, other_actor.id]))
        current_time = time.time()
        
        # Check cooldown
        if collision_pair in self.collision_cooldown:
            if current_time - self.collision_cooldown[collision_pair] < self.cooldown_time:
                return
        
        self.collision_cooldown[collision_pair] = current_time
        
        collision_data = {
            'timestamp': current_time - self.start_time,
            'simulation_time': self.world.get_snapshot().timestamp.elapsed_seconds,
            'actor1_id': actor_id,
            'actor2_id': other_actor.id,
            'actor1_type': self._get_actor_type(actor_id),
            'actor2_type': other_actor.type_id,
            'location': {
                'x': collision_event.transform.location.x,
                'y': collision_event.transform.location.y,
                'z': collision_event.transform.location.z
            },
#            'geo_location':{
#            'latitude': collision_event.transform.get_geolocation().latitude,
#            'longitude': collision_event.transform.get_geolocation().longitude,
#            'altitude': collision_event.transform.get_geolocation().altitude
#            },
            'impulse': {
                'x': impulse.x,
                'y': impulse.y,
                'z': impulse.z,
                'magnitude': impulse_magnitude
            },
            'severity': severity.value,
            'detection_method': 'sensor',
            'static_object': other_actor.type_id.startswith('static.')
        }
        
        self._record_collision(collision_data)
        logger.debug(f"Collision detected: Actor {actor_id} ({self._get_actor_type(actor_id)}) vs Actor {other_actor.id} ({other_actor_type}), Severity: {severity.value}")
    
    def _get_actor_type(self, actor_id):
        try:
            actor = self.actor_manager.get_actor_by_id(actor_id)
            return actor.type_id if actor else 'unknown'
        except:
            return 'unknown'
    
    def _classify_collision_severity(self, impulse_magnitude):
        if impulse_magnitude < 100:
            return CollisionSeverity.MINOR
        elif impulse_magnitude < 500:
            return CollisionSeverity.MODERATE
        else:
            return CollisionSeverity.SEVERE
    
    def check_bbox_collisions(self, actors=None):
        if actors is None:
            current_frame = getattr(self, '_current_frame', 0)
            vehicles = self.actor_manager.get_vehicles(current_frame)
            walkers = self.actor_manager.get_walkers(current_frame)
            actors = vehicles + walkers
        
        # Get meaningful static objects
        static_actors = []
        if self.ignore_static_road_elements:
            current_frame = getattr(self, '_current_frame', 0)
            all_static = self.actor_manager.get_static_actors(current_frame)
            static_actors = [actor for actor in all_static 
                           if self._is_meaningful_static_collision(actor.type_id)]
        
        all_actors_to_check = actors + static_actors
        current_time = time.time()
        
        if len(all_actors_to_check) < 2:
            return
        
        # Extract actor data
        actor_data = self._extract_actor_data_vectorized(all_actors_to_check)
        if actor_data is None:
            return
        
        positions, geo_positions, bboxes, actor_ids, actor_types = actor_data
        
        # Perform vectorized collision detection
        collision_pairs = self._vectorized_bbox_intersections(positions, bboxes)
        
        # Process detected collisions
        collision_count = 0
        for i, j in zip(*collision_pairs):
            actor1_idx, actor2_idx = int(i), int(j)
            
            if self._should_ignore_collision(actor_types[actor2_idx]):
                continue
            
            if actor1_idx == actor2_idx:
                continue
            
            collision_pair = tuple(sorted([actor_ids[actor1_idx], actor_ids[actor2_idx]]))
            
            if collision_pair in self.collision_cooldown:
                if current_time - self.collision_cooldown[collision_pair] < self.cooldown_time:
                    continue
            
            self.collision_cooldown[collision_pair] = current_time
            
            collision_location = {
                'x': (positions[actor1_idx, 0] + positions[actor2_idx, 0]) / 2,
                'y': (positions[actor1_idx, 1] + positions[actor2_idx, 1]) / 2,
                'z': (positions[actor1_idx, 2] + positions[actor2_idx, 2]) / 2
            }

            geo_location = {
                'latitude': (geo_positions[actor1_idx, 0] + geo_positions[actor2_idx, 0]) / 2,
                'longitude': (geo_positions[actor1_idx, 1] + geo_positions[actor2_idx, 1]) / 2,
                'altitude': (geo_positions[actor1_idx, 2] + geo_positions[actor2_idx, 2]) / 2
            }


            collision_data = {
                'timestamp': current_time - self.start_time,
                'simulation_time': self.world.get_snapshot().timestamp.elapsed_seconds,
                'actor1_id': actor_ids[actor1_idx],
                'actor2_id': actor_ids[actor2_idx],
                'actor1_type': actor_types[actor1_idx],
                'actor2_type': actor_types[actor2_idx],
                'location': collision_location,
                'geo_location': geo_location,
                'impulse': None,
                'severity': CollisionSeverity.MINOR.value,
                'detection_method': 'bbox',
                'static_object': actor_types[actor2_idx].startswith('static.')
            }
            
            self._record_collision(collision_data)
            collision_count += 1
        
        if collision_count > 0:
            logger.debug(f"Detected {collision_count} bbox collisions")
    
    def _extract_actor_data_vectorized(self, actors):
        try:
            positions = []
            geo_positions = []   #[PA]
            bboxes = []
            actor_ids = []
            actor_types = []
            
            for actor in actors:
                try:
                    transform = actor.get_transform()
                    bbox = actor.bounding_box
                    
                    loc = transform.location
                    positions.append([loc.x, loc.y, loc.z])

                    geo_loc = actor.get_geolocation()                                               #[PA]
                    geo_positions.append( [geo_loc.latitude,geo_loc.longitude,geo_loc.altitude])    #[PA]

                    
                    extent = bbox.extent
                    bbox_min = [loc.x - extent.x, loc.y - extent.y, loc.z - extent.z]
                    bbox_max = [loc.x + extent.x, loc.y + extent.y, loc.z + extent.z]
                    bboxes.append([bbox_min, bbox_max])
                    
                    actor_ids.append(actor.id)
                    actor_types.append(actor.type_id)
                    
                except Exception:
                    continue
            
            if len(positions) < 2:
                return None
            
            positions = np.array(positions, dtype=np.float32)
            geo_positions = np.array(geo_positions, dtype=np.float32)
            bboxes = np.array(bboxes, dtype=np.float32)
            
            return positions, geo_positions, bboxes, actor_ids, actor_types
            
        except Exception as e:
            logger.error(f"Failed to extract actor data: {e}")
            return None
    
    def _vectorized_bbox_intersections(self, positions, bboxes):
        n = len(bboxes)
        
        min_coords = bboxes[:, 0, :]
        max_coords = bboxes[:, 1, :]
        
        # Quick distance-based culling
        max_extents = np.max(max_coords - min_coords, axis=1)
        
        pos_diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
        distances = np.linalg.norm(pos_diff, axis=2)
        
        max_extent_sums = max_extents[:, np.newaxis] + max_extents[np.newaxis, :]
        quick_reject_mask = distances > max_extent_sums
        
        # Detailed bbox intersection test
        min1 = min_coords[:, np.newaxis, :]
        max1 = max_coords[:, np.newaxis, :]
        min2 = min_coords[np.newaxis, :, :]
        max2 = max_coords[np.newaxis, :, :]
        
        intersects_per_axis = (min1 <= max2) & (max1 >= min2)
        bbox_intersections = np.all(intersects_per_axis, axis=2)
        
        # Apply quick rejection mask
        bbox_intersections = bbox_intersections & ~quick_reject_mask
        
        # Remove self-intersections
        np.fill_diagonal(bbox_intersections, False)
        
        return np.where(bbox_intersections)
    
    def check_distance_collisions(self, actors=None):
        if actors is None:
            current_frame = getattr(self, '_current_frame', 0)
            vehicles = self.actor_manager.get_vehicles(current_frame)
            walkers = self.actor_manager.get_walkers(current_frame)
            actors = vehicles + walkers
        
        # Get meaningful static objects
        static_actors = []
        if self.ignore_static_road_elements:
            current_frame = getattr(self, '_current_frame', 0)
            all_static = self.actor_manager.get_static_actors(current_frame)
            static_actors = [actor for actor in all_static 
                           if self._is_meaningful_static_collision(actor.type_id)]
        
        all_actors_to_check = actors + static_actors
        current_time = time.time()
        
        if SCIPY_AVAILABLE:
            self._check_distance_collisions_vectorized(all_actors_to_check, current_time)
        else:
            logger.warning("SciPy not available, using fallback distance collision detection")
            self._check_distance_collisions_fallback(all_actors_to_check, current_time)
    
    def _check_distance_collisions_vectorized(self, all_actors_to_check, current_time):
        try:
            positions = []
            actor_metadata = []
            
            for actor in all_actors_to_check:
                try:
                    loc = actor.get_location()
                    positions.append([loc.x, loc.y, loc.z])
                    actor_metadata.append((actor.id, actor.type_id))
                except:
                    continue
            
            if len(positions) < 2:
                return
            
            positions = np.array(positions)
            distances = squareform(pdist(positions))
            
            collision_indices = np.where((distances < self.distance_threshold) & (distances > 0))
            
            collision_count = 0
            for i, j in zip(*collision_indices):
                if i >= j:
                    continue
                
                actor1_id, actor1_type = actor_metadata[i]
                actor2_id, actor2_type = actor_metadata[j]
                
                if self._should_ignore_collision(actor2_type):
                    continue
                
                collision_pair = tuple(sorted([actor1_id, actor2_id]))
                
                if collision_pair in self.collision_cooldown:
                    if current_time - self.collision_cooldown[collision_pair] < self.cooldown_time:
                        continue
                
                self.collision_cooldown[collision_pair] = current_time
                
                collision_data = {
                    'timestamp': current_time - self.start_time,
                    'simulation_time': self.world.get_snapshot().timestamp.elapsed_seconds,
                    'actor1_id': actor1_id,
                    'actor2_id': actor2_id,
                    'actor1_type': actor1_type,
                    'actor2_type': actor2_type,
                    'location': {
                        'x': (positions[i, 0] + positions[j, 0]) / 2,
                        'y': (positions[i, 1] + positions[j, 1]) / 2,
                        'z': (positions[i, 2] + positions[j, 2]) / 2
                    },
                    'distance': distances[i, j],
                    'impulse': None,
                    'severity': CollisionSeverity.MINOR.value,
                    'detection_method': 'distance',
                    'static_object': actor2_type.startswith('static.')
                }
                
                self._record_collision(collision_data)
                collision_count += 1
            
            if collision_count > 0:
                logger.debug(f"Detected {collision_count} distance-based collisions")
        
        except Exception as e:
            logger.error(f"Vectorized distance collision check failed: {e}")
            self._check_distance_collisions_fallback(all_actors_to_check, current_time)
    
    def _check_distance_collisions_fallback(self, all_actors_to_check, current_time):
        collision_count = 0
        for i in range(len(all_actors_to_check)):
            for j in range(i + 1, len(all_actors_to_check)):
                actor1, actor2 = all_actors_to_check[i], all_actors_to_check[j]
                
                if self._should_ignore_collision(actor2.type_id):
                    continue
                
                try:
                    loc1 = actor1.get_location()
                    loc2 = actor2.get_location()
                    distance = np.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2 + (loc1.z - loc2.z)**2)
                    
                    if distance < self.distance_threshold:
                        collision_pair = tuple(sorted([actor1.id, actor2.id]))
                        
                        if collision_pair in self.collision_cooldown:
                            if current_time - self.collision_cooldown[collision_pair] < self.cooldown_time:
                                continue
                        
                        self.collision_cooldown[collision_pair] = current_time
                        
                        collision_data = {
                            'timestamp': current_time - self.start_time,
                            'simulation_time': self.world.get_snapshot().timestamp.elapsed_seconds,
                            'actor1_id': actor1.id,
                            'actor2_id': actor2.id,
                            'actor1_type': actor1.type_id,
                            'actor2_type': actor2.type_id,
                            'location': {
                                'x': (loc1.x + loc2.x) / 2,
                                'y': (loc1.y + loc2.y) / 2,
                                'z': (loc1.z + loc2.z) / 2
                            },
                            'distance': distance,
                            'impulse': None,
                            'severity': CollisionSeverity.MINOR.value,
                            'detection_method': 'distance',
                            'static_object': actor2.type_id.startswith('static.')
                        }
                        
                        self._record_collision(collision_data)
                        collision_count += 1
                except:
                    continue
        
        if collision_count > 0:
            logger.debug(f"Detected {collision_count} distance-based collisions (fallback)")
    
    def update(self, current_frame=None):
        if current_frame is not None:
            self._current_frame = current_frame
            self.actor_manager.set_current_frame(current_frame)
        
        if not self._should_check_collisions():
            return
        
        self.last_collision_check_frame = getattr(self, '_current_frame', 0)
        
        if self.detection_method == CollisionMethod.BBOX_BASED:
            self.check_bbox_collisions()
        elif self.detection_method == CollisionMethod.DISTANCE_BASED:
            self.check_distance_collisions()
        elif self.detection_method == CollisionMethod.HYBRID:
            self.check_bbox_collisions()
    
    def _should_check_collisions(self):
        current_frame = getattr(self, '_current_frame', 0)
        return (current_frame - self.last_collision_check_frame) >= self.collision_update_interval
    
    def _record_collision(self, collision_data):
        self.collision_events.append(collision_data)
        self.total_collisions += 1
        
        collision_type = f"{collision_data['actor1_type']} vs {collision_data['actor2_type']}"
        self.collisions_by_type[collision_type] += 1
    
    def get_collision_statistics(self):
        current_time = time.time() - self.start_time
        
        return {
            'total_collisions': self.total_collisions,
            'collisions_by_type': dict(self.collisions_by_type),
            'collision_rate': self.total_collisions / max(current_time, 1),
            'active_sensors': len(self.collision_sensors),
            'detection_method': self.detection_method.value,
            'simulation_time': current_time,
            'recent_collisions': len([c for c in self.collision_events if current_time - c['timestamp'] < 60]),
            'static_collisions': len([c for c in self.collision_events if c.get('static_object', False)]),
            'dynamic_collisions': len([c for c in self.collision_events if not c.get('static_object', False)]),
            'actor_cache_stats': self.actor_manager.get_cache_stats()
        }
    
    def get_recent_collisions(self, time_window=30):
        current_time = time.time() - self.start_time
        return [c for c in self.collision_events if current_time - c['timestamp'] < time_window]
    
    def cleanup(self):
        logger.info("Cleaning up collision detector")
        sensor_count = len(self.collision_sensors)
        
        for sensor in self.collision_sensors.values():
            try:
                if sensor.is_alive:
                    sensor.destroy()
            except:
                pass
        
        self.collision_sensors.clear()
        
        if hasattr(self, 'actor_manager'):
            self.actor_manager.cleanup()
        
        logger.info(f"Collision detector cleanup complete. Removed {sensor_count} sensors, detected {self.total_collisions} total collisions")


def analyze_collision_hotspots(collision_events, grid_size=10):
    hotspots = defaultdict(int)
    
    for collision in collision_events:
        loc = collision['location']
        grid_x = int(loc['x'] // grid_size)
        grid_y = int(loc['y'] // grid_size)
        hotspots[(grid_x, grid_y)] += 1
    
    return dict(hotspots)


def filter_collisions_by_severity(collision_events, min_severity=CollisionSeverity.MODERATE):
    severity_order = {
        CollisionSeverity.MINOR.value: 0,
        CollisionSeverity.MODERATE.value: 1,
        CollisionSeverity.SEVERE.value: 2
    }
    
    min_level = severity_order[min_severity.value]
    
    return [c for c in collision_events if severity_order.get(c['severity'], 0) >= min_level]