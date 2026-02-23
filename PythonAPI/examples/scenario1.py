#!/usr/bin/env python3
"""
triggered_walker.py

Spawn a pedestrian that only starts crossing when the ego vehicle comes close.

Assumptions:
- CARLA server is already running.
- Another client is advancing the simulation (world.tick()), OR the server is in
  default asynchronous mode. This script never calls world.tick().
"""

import argparse
import time
import logging
import random
import threading
import sys
import glob
import sub_data
import os

SCREEN_OFF = 0
ACCIDENT = 1
SLOW_VEHICHLE= 2
ROADWORKS = 3
TRAFFIC = 4
HUMAN_PERSENCE = 5
EMERGENCY_VEHICLE = 6
INCIDENT_NERBY = 7

DURATA_VISIVA = 1

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import carla  # Make sure CARLA's Python API is on PYTHONPATH.
except ImportError:
    raise RuntimeError(
        "Could not import 'carla'. Make sure you have added "
        "<CARLA_ROOT>/PythonAPI/carla/dist/carla-*.egg to PYTHONPATH."
    )


# ---------------------------------------------------------------------------
# Scenario API (scenario-specific code should live behind these)
# ---------------------------------------------------------------------------

def configure_triggered_walker_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """
    Add arguments specific to the 'triggered_walker' scenario.
    Keep all generic/framework args (host/port/timeout/logging/etc.) outside.
    """
    parser.add_argument("--ego-role-name", default="hero",
                        help="role_name of the ego vehicle (default: 'hero')")

    parser.add_argument("--trigger-distance", type=float, default=60.0,
                        help="Distance (m) at which the walker starts crossing (default: 60.0)")

    parser.add_argument("--walker-speed", type=float, default=1.4,
                        help="Walker max speed in m/s when crossing (default: 1.4)")

    parser.add_argument("--arrival-threshold", type=float, default=0.5,
                        help="Distance (m) to target at which the walker is considered arrived "
                             "(default: 0.5)")

    # Spawn / target positions in world coordinates
    parser.add_argument("--spawn", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        required=False,
                        help="Spawn location of the walker (X Y Z in world coordinates)",
                        default=(441, 214, 120))
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        required=False,
                        help="Target location across the street (X Y Z in world coordinates)",
                        default=(441, 200, 120))

    return parser




# ---------------------------------------------------------------------------
# Helpers (scenario-internal helpers; keep as-is unless you want to further modularize)
# ---------------------------------------------------------------------------

def find_ego_vehicle(world: "carla.World", role_name: str):
    """Return the first vehicle actor with the given role_name, or None."""
    vehicles = world.get_actors().filter("vehicle.*")
    for v in vehicles:
        if v.attributes.get("role_name") == role_name:
            return v
    return None

import subprocess

def play_sound(path: str):
    subprocess.Popen(["aplay", path],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL)
def _pick_vehicle_blueprint(blueprint_library: "carla.BlueprintLibrary", blueprint_name = None) -> "carla.ActorBlueprint":
    if blueprint_name != None:
        return blueprint_library.filter(blueprint_name)[0]

    vehicle_bps = blueprint_library.filter("vehicle.*")
    if not vehicle_bps:
        raise RuntimeError("No vehicle blueprints found (filter 'vehicle.*').")

    bp = random.choice(vehicle_bps)

    # Optional: make them easier to spot / deterministic
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "scenario_vehicle")

    return bp

def find_closest_spawn_point(world: "carla.World",
                             location: "carla.Location") -> "carla.Transform | None":
    """
    Return the closest map spawn point to the given location.
    """
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        return None

    closest = min(
        spawn_points,
        key=lambda sp: sp.location.distance(location)
    )
    return closest

def _advance_waypoint_random(wp: "carla.Waypoint", steps: int, step_m: float) -> "carla.Waypoint | None":
    """
    Advance `steps` times from waypoint `wp`, each time calling wp.next(step_m) (meters).
    If multiple candidates are returned, choose one at random.
    Returns the final waypoint, or None if we hit a dead-end.
    """
    current = wp
    for _ in range(steps):
        next_wps = current.next(step_m)
        if not next_wps:
            return None
        current = random.choice(next_wps)
    return current


def spawn_stopped_vehicle_queue(client: "carla.Client", world: "carla.World", num=10, blueprint=None, base_loc = carla.Location(413, 562, 123)) -> list["carla.Actor"]:
    """
    Spawn X vehicles one in front of the other on the road, stopped.

    - Use the nearest MAP SPAWN POINT (from get_spawn_points()) to a reference location as the start.
    - Spawn the first car at that spawn point transform.
    - Convert that spawn transform to a driving waypoint, then for each next car:
      advance `gap_waypoints` waypoint-steps ahead (each step is wp.next(step_m) meters),
      choosing randomly when there are multiple candidates.
    """

    gap_waypoints = 8   # number of waypoint-steps between cars
    step_m = 1.2        # meters per waypoint step (used by wp.next)

    if num <= 0:
        return []

    blueprint_library = world.get_blueprint_library()

    # base_loc = carla.Location(413, 562, 123)

    # 1) Nearest spawn point to base_loc
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("Map has no spawn points.")

    start_spawn = min(spawn_points, key=lambda sp: sp.location.distance(base_loc))

    # 2) Driving waypoint corresponding to the spawn point (used to step forward)
    wp = world.get_map().get_waypoint(
        start_spawn.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    wp = _advance_waypoint_random(wp, steps=gap_waypoints, step_m=step_m)
    if wp is None:
        raise RuntimeError(f"Could not find a driving waypoint near spawn point {start_spawn.location}")

    vehicles: list["carla.Actor"] = []

    for i in range(num):
        # First car: spawn at the spawn point transform (reliable alignment).
        # Others: spawn at the waypoint transforms derived from stepping forward.

        car_bp = _pick_vehicle_blueprint(blueprint_library, blueprint)
        v = world.try_spawn_actor(car_bp, start_spawn)
        if v is None:
            continue

        transform = wp.transform
        v.set_transform(transform)

        hold_vehicle_stopped(v)
        vehicles.append(v)

        logging.info(
            "Spawned queued car %d id=%d type=%s at (%.2f, %.2f, %.2f)",
            i, v.id, v.type_id,
            transform.location.x, transform.location.y, transform.location.z
        )

        # For the next vehicle position, advance from the CURRENT waypoint
        next_wp = _advance_waypoint_random(wp, steps=gap_waypoints, step_m=step_m)
        if next_wp is None:
            logging.warning(
                "No further waypoint found when advancing %d waypoint-steps; stopping at %d cars.",
                gap_waypoints, len(vehicles)
            )
            break

        wp = next_wp

    return vehicles



def hold_vehicle_stopped(vehicle: "carla.Actor") -> None:
    """Force a vehicle to remain stopped (no TM control)."""
    try:
        vehicle.set_autopilot(False)
    except Exception:
        pass

    try:
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
    except Exception as e:
        logging.warning("Failed to apply stop control to vehicle id=%s: %s", getattr(vehicle, "id", "?"), e)


def release_vehicle_queue_to_traffic_manager(client: "carla.Client", vehicles: list["carla.Actor"], args) -> None:
    """Enable Traffic Manager autopilot for the queued vehicles."""
    if not vehicles:
        return

    # You can set TM global behaviors here if desired (optional):
    # tm.global_percentage_speed_difference(0.0)
    # tm.set_global_distance_to_leading_vehicle(2.5)

    for v in vehicles:
        if v is None or not v.is_alive:
            continue

        # Remove forced stop, then yield control to TM.
        try:
            v.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
        except Exception:
            pass

        v.set_autopilot(True, 8000)

    logging.info("Released %d queued cars to Traffic Manager (port=%d).", len(vehicles), 8000)

def scenario7(client: "carla.Client",
              world: "carla.World",
              ego_vehicle: "carla.Actor",
              args,
              subscriber,
              night=False,
              location=carla.Location(-198, 146, 152)):
    # Spawn the stopped car queue first
    queued_vehicles = spawn_stopped_vehicle_queue(client, world, 1, "vehicle.carlamotors.european_hgv", location)

    trigger_distance = 40

    while True:
        next_sun(world)
        world_snapshot = world.wait_for_tick()
        ego_loc = ego_vehicle.get_location()
        distance = queued_vehicles[0].get_location().distance(ego_loc)
        if trigger_distance > distance:
            break

    client.set_vodafone_alert_image(HUMAN_PERSENCE)
    release_vehicle_queue_to_traffic_manager(client, queued_vehicles, args)
    wait_for(world, 1)
    play_sound("./sounds/ADAS_Test_Package/alert_slow_vehicle_FAR.wav")
    client.set_vodafone_alert_image(SCREEN_OFF)
    wait_for(world, 1)
    if night:
        play_sound("./sounds/ALERT VOCALI NOTTE/veicolo_lento_notte_2.wav")
    else:
        play_sound("./sounds/day/veicolo_lento_giorno_2.wav")


def scenario6(client: "carla.Client",
              world: "carla.World",
              ego_vehicle: "carla.Actor",
              args,
              subscriber,
              night=False,
              trigger_point=carla.Location(88, 181, 143),
              spawn_point=carla.Location(350, 300, 128)):

    while True:
        next_sun(world)
        world_snapshot = world.wait_for_tick()
        loc = ego_vehicle.get_location()
        if 10 > loc.distance(trigger_point):
            break


    spawn_points = world.get_map().get_spawn_points()
    start_spawn = min(spawn_points, key=lambda sp: sp.location.distance(spawn_point))
    if night:
        bp = [x for x in world.get_blueprint_library().filter("vehicle.carlamotors.firetruck")][0]
    else:
        bp = [x for x in world.get_blueprint_library().filter("vehicle.ford.ambulance")][0]

    ambulance = world.try_spawn_actor(bp, start_spawn)
    ambulance.set_light_state(
    carla.VehicleLightState(
        carla.VehicleLightState.Special1 |
        carla.VehicleLightState.Special2 |
        carla.VehicleLightState.Position |
        carla.VehicleLightState.LowBeam
    )
    )
    tm = client.get_trafficmanager(8000)
    tm.vehicle_percentage_speed_difference(ambulance, -40.0)
    tm.distance_to_leading_vehicle(ambulance, 1.0)
    tm.ignore_lights_percentage(ambulance, 100.0)
    tm.ignore_signs_percentage(ambulance, 100.0)
    tm.auto_lane_change(ambulance, True)
    ambulance.set_autopilot(True)

    wait_for(world, 5)

    client.set_vodafone_alert_image(EMERGENCY_VEHICLE)
    wait_for(world, 1)
    client.set_vodafone_alert_image(SCREEN_OFF)
    play_sound("./sounds/ADAS_Test_Package/alert_emergency_MID.wav")
    wait_for(world, 1)
    if night:
        play_sound("./sounds/ALERT VOCALI NOTTE/mezzo_emergenza_notte_1.wav")
    else:
        play_sound("./sounds/day/Mezzo_emergenza_giorno_1.wav")
    wait_for(world, 1)

def scenario5(client: "carla.Client",
              world: "carla.World",
              ego_vehicle: "carla.Actor",
              args,
              subscriber,
              day=True,
              location=carla.Location(309, 449, 135)):
    # Spawn the stopped car queue first

    while True:
        next_sun(world)
        world_snapshot = world.wait_for_tick()

        ego_loc = ego_vehicle.get_location()
        print(ego_loc.distance( location))
        if 10 > ego_loc.distance( location) :
            break


    client.set_vodafone_alert_image(INCIDENT_NERBY)
    wait_for(world, 1)
    client.set_vodafone_alert_image(SCREEN_OFF)
    play_sound("./sounds/ADAS_Test_Package/alert_incident_MID.wav")
    wait_for(world, 1)
    if day:
        play_sound("./sounds/day/Incidente_giorno_3.wav")
    else:
        play_sound("./sounds/ALERT VOCALI NOTTE/Incidente_notte_4.wav")
    wait_for(world, 1)





def scenario4(client: "carla.Client",
              world: "carla.World",
              ego_vehicle: "carla.Actor",
              args,
              subscriber, night=False, location=carla.Location(263, 444, 138), queue_location=carla.Location(413, 562, 123)):
    # Spawn the stopped car queue first
    queued_vehicles = spawn_stopped_vehicle_queue(client, world, base_loc=queue_location)

    waypoint = world.get_map().get_waypoint(
        location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    tm = client.get_trafficmanager(8000)
    for vehicle in queued_vehicles:
        tm.set_path(vehicle, [waypoint.transform.location])

    trigger_distance = 40

    while True:
        next_sun(world)
        world_snapshot = world.wait_for_tick()
        ego_loc = ego_vehicle.get_location()
        distance = queued_vehicles[0].get_location().distance(ego_loc)
        if trigger_distance > distance:
            break

    client.set_vodafone_alert_image(TRAFFIC)
    wait_for(world, 1)
    client.set_vodafone_alert_image(SCREEN_OFF)
    play_sound("./sounds/ADAS_Test_Package/alert_traffic_MID.wav")
    wait_for(world, 1)
    if night:
        play_sound("./sounds/ALERT VOCALI NOTTE/Traffico_intenso_notte_3.wav")
    else:
        play_sound("./sounds/day/Traffico_intenso_giorno_1.wav")

    wait_for(world, 4)
    release_vehicle_queue_to_traffic_manager(client, queued_vehicles, args)
    return queued_vehicles




def scenario2(client: "carla.Client",
                     world: "carla.World",
                     ego_vehicle: "carla.Actor",
                     args,
                     subscriber, night=False, location=carla.Location(655.899414, 419.032440, 117.766655)):

    while True:
        next_sun(world)
        world_snapshot = world.wait_for_tick()

        ego_loc = ego_vehicle.get_location()
        if 10 > ego_loc.distance(location) :
            client.set_vodafone_alert_image(ROADWORKS)
            wait_for(world, 1)
            play_sound("sounds/ADAS_Test_Package/alert_roadworks_MID.wav")
            wait_for(world, 1)
            if night:
                play_sound("sounds/ALERT VOCALI NOTTE/Lavori_in_corso_notte_3.wav")
            else:
                play_sound("sounds/day/Lavori_in_corso_giorno_1.wav")
            break

    wait_for(world, 1.5)
    client.set_vodafone_alert_image(SCREEN_OFF)


def spawn_walker_with_controller(client: "carla.Client", world: "carla.World", args, location):
    """Spawn a single walker and attach a WalkerAIController to it."""
    blueprint_library = world.get_blueprint_library()

    walker_bps = blueprint_library.filter("walker.pedestrian.*")
    if not walker_bps:
        raise RuntimeError("No walker blueprints found (filter 'walker.pedestrian.*').")

    walker_bp = random.choice(walker_bps)

    # Make sure the pedestrian can actually be hit if needed (no invincibility).
    if walker_bp.has_attribute("is_invincible"):
        walker_bp.set_attribute("is_invincible", "false")

    spawn_location = location
    spawn_transform = carla.Transform(spawn_location)

    walker = world.try_spawn_actor(walker_bp, spawn_transform)
    if walker is None:
        raise RuntimeError(f"Failed to spawn walker at {spawn_location} (spot blocked?).")

    # AI controller blueprint
    controller_bp = blueprint_library.find("controller.ai.walker")

    # Attach controller to walker (third argument is parent actor)
    controller = world.spawn_actor(controller_bp, carla.Transform(), walker)

    logging.info("Spawned walker id=%d type=%s at (%.2f, %.2f, %.2f)",
                 walker.id, walker.type_id,
                 spawn_location.x, spawn_location.y, spawn_location.z)
    logging.info("Spawned walker controller id=%d", controller.id)

    # We do NOT call controller.start() yet: walker will stand still until triggered.
    return walker, controller

def wait_for(world, quantity):
    start_message = time.time()
    current = time.time()
    while quantity > current - start_message:
        world_snapshot = world.wait_for_tick()
        current = time.time()

current_sun_value = 0
target_sun_value = 0

def next_sun(world):
    global current_sun_value
    global target_sun_value
    while current_sun_value > target_sun_value:
        current_sun_value = current_sun_value - 0.1

        weather = world.get_weather()
        weather.sun_altitude_angle = current_sun_value
        weather.sun_azimuth_angle = 60
        weather.fog_density = 2
        weather.fog_distance = 0.75
        weather.fog_falloff = 0.1
        world.set_weather(weather)
        world_snapshot = world.wait_for_tick()


def scenario1(client: "carla.Client",
                     world: "carla.World",
                     ego_vehicle: "carla.Actor",
                     args,
                     subscriber,
                     night=False,
                     location=carla.Location(441, 214, 120),
                     target=carla.Location(441, 200, 120)):
    """Main loop: wait for ego within distance, then send walker to target."""
    world.set_pedestrians_cross_factor(0)
    walker, controller = spawn_walker_with_controller(client, world, args, location)
    # trans = walker.get_transform()
    controller.set_max_speed(float(0))
    trigger_distance = float(args.trigger_distance)
    arrival_threshold = float(args.arrival_threshold)

    target_location = target

    triggered = False

    logging.info("Waiting for ego vehicle with role_name='%s'...", args.ego_role_name)

    sun_altitude = 0.0
    sun_azimuth = 0.0

    snapshot = world.get_snapshot()
    ts = snapshot.platform_timestamp
    # unix_now = time.time()
    # offset = unix_now - ts


    while not triggered:
        # Wait for the next server tick; does not create a tick.
        next_sun(world)
        world_snapshot = world.wait_for_tick()

        #subscriber.inject_marker((world_snapshot.platform_timestamp +offset) * 1000, world_snapshot.frame, "frame")
        # weather = world.get_weather()
        # sun_altitude += 0.03      # vertical movement speed
        # sun_azimuth += 0.05       # horizontal movement speed
        # if sun_altitude > 180:
            # return
            # sun_altitude = -180

        # weather.sun_altitude_angle = sun_altitude
        # weather.sun_azimuth_angle = sun_azimuth
        #world.set_weather(weather)

        walker_loc = walker.get_location()
        ego_loc = ego_vehicle.get_location()

        distance = walker_loc.distance(ego_loc)
        print(distance)

        if not triggered and distance <= trigger_distance:
            logging.info("Ego within %.1f m (threshold=%.1f). Starting crossing.",
                         distance, trigger_distance)

            controller.start()
            controller.go_to_location(target_location)
            controller.set_max_speed(float(args.walker_speed))
            world.set_pedestrians_cross_factor(1.0)
            triggered = True

    client.set_vodafone_alert_image(HUMAN_PERSENCE)
    wait_for(world, DURATA_VISIVA)
    client.set_vodafone_alert_image(SCREEN_OFF)
    wait_for(world, 1)
    play_sound("./sounds/ADAS_Test_Package/alert_pedoni_NEAR(1).wav")
    wait_for(world, 1)
    if night:
        play_sound("./sounds/ALERT VOCALI NOTTE/Attenzione_pedoni_notte1.wav")
    else:
        play_sound("./sounds/day/Attenzione_pedoni_giorno2.wav")
    wait_for(world, 8)

    cleanup(walker, controller)

def cleanup(walker: "carla.Actor", controller: "carla.Actor"):
    """Safely destroy walker and controller."""
    actors = [controller, walker]
    for actor in actors:
        if actor is None:
            continue
        try:
            if hasattr(actor, "stop"):
                try:
                    actor.stop()
                except RuntimeError:
                    pass
            actor_id = actor.id
            actor.destroy()
            logging.info("Destroyed actor id=%d", actor_id)
        except Exception as e:
            logging.warning("Error while destroying actor: %s", e)




# ---------------------------------------------------------------------------
# Generic CLI + Main (not scenario-specific)
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run CARLA scenarios."
    )

    # Generic/common args (keep outside scenario)
    parser.add_argument("--host", default="127.0.0.1",
                        help="CARLA server IP (default: 127.0.0.1)")
    parser.add_argument("-p", "--port", type=int, default=2000,
                        help="CARLA TCP port (default: 2000)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Client connection timeout in seconds (default: 10.0)")

    # Scenario-specific args
    configure_triggered_walker_args(parser)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    cortex_subscriber = sub_data.start() if False else None
    time.sleep(2)

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    # Connection management (generic; keep outside scenario)
    logging.info("Connecting to CARLA at %s:%d", args.host, args.port)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    logging.info("Connected. Current map: %s", world.get_map().name)

    ego_vehicle = None
    while ego_vehicle is None:
        world_snapshot = world.wait_for_tick()

        if world_snapshot is None:
            continue

        # acquire ego vehicle if needed.
        ego_vehicle = find_ego_vehicle(world, args.ego_role_name)
        if ego_vehicle is not None:
            logging.info("Found ego vehicle id=%d type=%s",
                            ego_vehicle.id, ego_vehicle.type_id)

    # Scenario execution
    light_manager = world.get_lightmanager()
    for light in light_manager.get_all_lights():
        light.turn_on()
    # tm = client.get_trafficmanager(8000)
    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_global_vehicle_lights(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)
    global target_sun_value
    global current_sun_value
    target_sun_value = 45
    current_sun_value = 46
    print(carla.WeatherParameters.ClearNight)
    print(carla.WeatherParameters.ClearNoon)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    next_sun(world)

    scenario1(client, world, ego_vehicle, args, cortex_subscriber)
    target_sun_value = 45
    scenario2(client, world, ego_vehicle, args, cortex_subscriber)
    target_sun_value = 35
    spawned = scenario4(client, world, ego_vehicle, args, cortex_subscriber)
    target_sun_value = 25
    scenario5(client, world, ego_vehicle, args, cortex_subscriber)
    for vehichle in spawned:
        vehichle.destroy()
    target_sun_value = 15
    scenario6(client, world, ego_vehicle, args, cortex_subscriber)
    target_sun_value = 10
    scenario7(client, world, ego_vehicle, args, cortex_subscriber)

    # cortex_subscriber.stop_record()
    # cortex_subscriber.join()


    scenario5(client, world, ego_vehicle, args, cortex_subscriber, False, carla.Location(-302, 81, 149))

    scenario1(client, world, ego_vehicle, args, cortex_subscriber, True, carla.Location(-41, -60, 126), carla.Location(-43, -73, 126))

    scenario7(client, world, ego_vehicle, args, cortex_subscriber, True, carla.Location(-26, -226, 127) )

    scenario2(client, world, ego_vehicle, args, cortex_subscriber, True, carla.Location(-266, -441, 138) )

    spawned = scenario4(client, world, ego_vehicle, args, cortex_subscriber, True, queue_location=carla.Location(105, -480, 153), location=carla.Location(31, -541, 158))

    scenario6(client, world, ego_vehicle, args, cortex_subscriber, True, trigger_point=carla.Location(714, 39, 134), spawn_point=carla.Location(676, 5.4, 135))

if __name__ == "__main__":
    main()

