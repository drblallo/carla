#!/usr/bin/env python

"""
Streamlined Actor Manager for CARLA Simulator - Centralized actor caching system
"""

import logging
from typing import List, Dict, Optional, Any

# Setup logging
logger = logging.getLogger(__name__)


class ActorManager:
    """
    Centralized cache manager for CARLA actor queries
    """
    
    def __init__(self, world, update_interval: int = 30):
        self.world = world
        self.update_interval = update_interval
        self.last_update_frame = 0
        self._current_frame = 0
        
        # Cached actor lists
        self.cached_vehicles = []
        self.cached_walkers = []
        self.cached_static = []
        self.cached_all_actors = []
        self.cached_controllers = []
        
        # Actor lookup dictionaries
        self._vehicle_lookup = {}
        self._walker_lookup = {}
        self._static_lookup = {}
        
        # Performance tracking
        self.cache_hits = 0
        self.cache_misses = 0
        self.total_queries = 0
        
        # Initial population
        self._update_cache()
        
        logger.info(f"ActorManager initialized with update_interval={update_interval} frames")
        logger.debug(f"Initial cache: {len(self.cached_vehicles)} vehicles, {len(self.cached_walkers)} walkers, {len(self.cached_static)} static actors")
    
    def get_vehicles(self, current_frame: Optional[int] = None) -> List[Any]:
        if current_frame is not None:
            self.set_current_frame(current_frame)
        
        self.total_queries += 1
        
        if self._should_update_cache():
            self._update_cache()
            self.cache_misses += 1
        else:
            self.cache_hits += 1
        
        return self.cached_vehicles.copy()
    
    def get_walkers(self, current_frame: Optional[int] = None) -> List[Any]:
        if current_frame is not None:
            self.set_current_frame(current_frame)
        
        self.total_queries += 1
        
        if self._should_update_cache():
            self._update_cache()
            self.cache_misses += 1
        else:
            self.cache_hits += 1
        
        return self.cached_walkers.copy()
    
    def get_static_actors(self, current_frame: Optional[int] = None) -> List[Any]:
        if current_frame is not None:
            self.set_current_frame(current_frame)
        
        self.total_queries += 1
        
        if self._should_update_cache():
            self._update_cache()
            self.cache_misses += 1
        else:
            self.cache_hits += 1
        
        return self.cached_static.copy()
    
    def get_controllers(self, current_frame: Optional[int] = None) -> List[Any]:
        if current_frame is not None:
            self.set_current_frame(current_frame)
        
        self.total_queries += 1
        
        if self._should_update_cache():
            self._update_cache()
            self.cache_misses += 1
        else:
            self.cache_hits += 1
        
        return self.cached_controllers.copy()
    
    def get_all_actors(self, current_frame: Optional[int] = None) -> List[Any]:
        if current_frame is not None:
            self.set_current_frame(current_frame)
        
        self.total_queries += 1
        
        if self._should_update_cache():
            self._update_cache()
            self.cache_misses += 1
        else:
            self.cache_hits += 1
        
        return self.cached_all_actors.copy()
    
    def get_actor_by_id(self, actor_id: int, actor_type: str = "any") -> Optional[Any]:
        if actor_type == "vehicle":
            return self._vehicle_lookup.get(actor_id)
        elif actor_type == "walker":
            return self._walker_lookup.get(actor_id)
        elif actor_type == "static":
            return self._static_lookup.get(actor_id)
        else:
            return (self._vehicle_lookup.get(actor_id) or 
                   self._walker_lookup.get(actor_id) or 
                   self._static_lookup.get(actor_id))
    
    def get_actors_by_filter(self, filter_pattern: str, current_frame: Optional[int] = None) -> List[Any]:
        all_actors = self.get_all_actors(current_frame)
        return [actor for actor in all_actors if self._matches_filter(actor.type_id, filter_pattern)]
    
    def _matches_filter(self, type_id: str, filter_pattern: str) -> bool:
        if filter_pattern.endswith('*'):
            return type_id.startswith(filter_pattern[:-1])
        else:
            return type_id == filter_pattern
    
    def set_current_frame(self, frame: int):
        self._current_frame = frame
    
    def force_update(self):
        logger.debug("Forcing cache update")
        self._update_cache()
    
    def _should_update_cache(self) -> bool:
        return (self._current_frame - self.last_update_frame) >= self.update_interval
    
    def _update_cache(self):
        try:
            all_actors = self.world.get_actors()
            
            # Clear previous lookups
            self._vehicle_lookup.clear()
            self._walker_lookup.clear()
            self._static_lookup.clear()
            
            # Filter and cache different actor types
            vehicles = []
            walkers = []
            static_actors = []
            controllers = []
            
            for actor in all_actors:
                actor_type = actor.type_id
                actor_id = actor.id
                
                if 'vehicle' in actor_type:
                    vehicles.append(actor)
                    self._vehicle_lookup[actor_id] = actor
                elif 'walker.pedestrian' in actor_type:
                    walkers.append(actor)
                    self._walker_lookup[actor_id] = actor
                elif 'controller.ai.walker' in actor_type:
                    controllers.append(actor)
                elif actor_type.startswith('static.'):
                    static_actors.append(actor)
                    self._static_lookup[actor_id] = actor
            
            # Update cached lists
            self.cached_vehicles = vehicles
            self.cached_walkers = walkers
            self.cached_static = static_actors
            self.cached_controllers = controllers
            self.cached_all_actors = list(all_actors)
            
            # Update tracking info
            self.last_update_frame = self._current_frame
            
            logger.debug(f"Cache updated at frame {self._current_frame}: {len(vehicles)} vehicles, {len(walkers)} walkers, {len(static_actors)} static, {len(controllers)} controllers")
            
        except Exception as e:
            # Keep existing cache if update fails
            logger.error(f"Failed to update actor cache: {e}")
            pass
    
    def get_cache_stats(self) -> Dict[str, Any]:
        hit_rate = (self.cache_hits / max(self.total_queries, 1)) * 100
        
        stats = {
            'cache_hits': self.cache_hits,
            'cache_misses': self.cache_misses,
            'total_queries': self.total_queries,
            'hit_rate_percent': hit_rate,
            'update_interval': self.update_interval,
            'last_update_frame': self.last_update_frame,
            'current_frame': self._current_frame,
            'cached_vehicles': len(self.cached_vehicles),
            'cached_walkers': len(self.cached_walkers),
            'cached_static': len(self.cached_static),
            'cached_controllers': len(self.cached_controllers),
            'total_cached_actors': len(self.cached_all_actors),
            'frames_since_update': self._current_frame - self.last_update_frame
        }
        
        if self.total_queries > 0 and self.total_queries % 100 == 0:
            logger.info(f"Cache performance: {hit_rate:.1f}% hit rate ({self.cache_hits}/{self.total_queries} queries)")
        
        return stats
    
    def reset_stats(self):
        logger.debug("Resetting actor manager statistics")
        self.cache_hits = 0
        self.cache_misses = 0
        self.total_queries = 0
    
    def get_alive_actors(self, actor_list: List[Any]) -> List[Any]:
        alive_actors = []
        for actor in actor_list:
            try:
                if actor.is_alive:
                    alive_actors.append(actor)
            except:
                continue
        return alive_actors
    
    def cleanup(self):
        logger.info(f"ActorManager cleanup - Final stats: {self.cache_hits} hits, {self.cache_misses} misses, {self.total_queries} total queries")
        pass


def create_actor_manager(world, update_interval: int = 30) -> ActorManager:
    logger.info(f"Creating ActorManager with update interval of {update_interval} frames")
    return ActorManager(world, update_interval)


def get_actors_by_type(world, actor_type: str, use_cache: bool = True, 
                      actor_manager: Optional[ActorManager] = None) -> List[Any]:
    if use_cache and actor_manager:
        logger.debug(f"Getting actors of type '{actor_type}' from cache")
        if actor_type == 'vehicles':
            return actor_manager.get_vehicles()
        elif actor_type == 'walkers':
            return actor_manager.get_walkers()
        elif actor_type == 'static':
            return actor_manager.get_static_actors()
        elif actor_type == 'controllers':
            return actor_manager.get_controllers()
        elif actor_type == 'all':
            return actor_manager.get_all_actors()
    
    # Fallback to direct world query
    logger.debug(f"Getting actors of type '{actor_type}' directly from world")
    if actor_type == 'vehicles':
        return list(world.get_actors().filter('vehicle.*'))
    elif actor_type == 'walkers':
        return list(world.get_actors().filter('walker.pedestrian.*'))
    elif actor_type == 'static':
        return list(world.get_actors().filter('static.*'))
    elif actor_type == 'controllers':
        return list(world.get_actors().filter('controller.ai.walker'))
    elif actor_type == 'all':
        return list(world.get_actors())
    else:
        return []