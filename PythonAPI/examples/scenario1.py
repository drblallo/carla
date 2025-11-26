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
import logging
import random
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

try:
    import carla  # Make sure CARLA's Python API is on PYTHONPATH.
except ImportError:
    raise RuntimeError(
        "Could not import 'carla'. Make sure you have added "
        "<CARLA_ROOT>/PythonAPI/carla/dist/carla-*.egg to PYTHONPATH."
    )


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Spawn a walker that crosses when the ego vehicle gets close."
    )

    parser.add_argument("--host", default="127.0.0.1",
                        help="CARLA server IP (default: 127.0.0.1)")
    parser.add_argument("-p", "--port", type=int, default=2000,
                        help="CARLA TCP port (default: 2000)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Client connection timeout in seconds (default: 10.0)")

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
                        help="Spawn location of the walker (X Y Z in world coordinates)", default=(441, 214, 120))
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        required=False,
                        help="Target location across the street (X Y Z in world coordinates)", default=(441, 200, 120))

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_ego_vehicle(world: "carla.World", role_name: str):
    """Return the first vehicle actor with the given role_name, or None."""
    vehicles = world.get_actors().filter("vehicle.*")
    for v in vehicles:
        if v.attributes.get("role_name") == role_name:
            return v
    return None


def spawn_walker_with_controller(world: "carla.World", args):
    """Spawn a single walker and attach a WalkerAIController to it."""
    blueprint_library = world.get_blueprint_library()

    walker_bps = blueprint_library.filter("walker.pedestrian.*")
    if not walker_bps:
        raise RuntimeError("No walker blueprints found (filter 'walker.pedestrian.*').")

    walker_bp = random.choice(walker_bps)

    # Make sure the pedestrian can actually be hit if needed (no invincibility).
    if walker_bp.has_attribute("is_invincible"):
        walker_bp.set_attribute("is_invincible", "false")

    spawn_location = carla.Location(
        x=args.spawn[0],
        y=args.spawn[1],
        z=args.spawn[2],
    )
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


def run_trigger_loop(world: "carla.World",
                     walker: "carla.Actor",
                     controller: "carla.Actor",
                     args):
    """Main loop: wait for ego within distance, then send walker to target."""
    trigger_distance = float(args.trigger_distance)
    arrival_threshold = float(args.arrival_threshold)

    target_location = carla.Location(
        x=args.target[0],
        y=args.target[1],
        z=args.target[2],
    )

    ego_vehicle = None
    triggered = False

    logging.info("Waiting for ego vehicle with role_name='%s'...", args.ego_role_name)

    while True:
        # Wait for the next server tick; does not create a tick.
        world_snapshot = world.wait_for_tick()
        if world_snapshot is None:
            # Very unlikely, but just continue if it happens.
            continue

        # (Re)acquire ego vehicle if needed.
        if ego_vehicle is None or not ego_vehicle.is_alive:
            ego_vehicle = find_ego_vehicle(world, args.ego_role_name)
            if ego_vehicle is not None:
                logging.info("Found ego vehicle id=%d type=%s",
                             ego_vehicle.id, ego_vehicle.type_id)

        if ego_vehicle is None:
            # No ego yet: keep waiting and checking.
            continue

        walker_loc = walker.get_location()
        ego_loc = ego_vehicle.get_location()

        distance = walker_loc.distance(ego_loc)

        if not triggered and distance <= trigger_distance:
            logging.info("Ego within %.1f m (threshold=%.1f). Starting crossing.",
                         distance, trigger_distance)

            controller.start()
            controller.go_to_location(target_location)
            controller.set_max_speed(float(args.walker_speed))

            triggered = True

        if triggered:
            # Update walker position after trigger
            walker_loc = walker.get_location()
            if walker_loc.distance(target_location) <= arrival_threshold:
                logging.info("Walker reached target (|Δ| ≤ %.2f m). Stopping controller.",
                             arrival_threshold)
                controller.stop()
                break


def cleanup(walker: "carla.Actor", controller: "carla.Actor"):
    """Safely destroy walker and controller."""
    actors = [controller, walker]
    for actor in actors:
        if actor is None:
            continue
        try:
            if hasattr(actor, "stop"):
                # For the controller, stop navigation before destroy.
                try:
                    actor.stop()
                except RuntimeError:
                    # Some actors (walker itself) won't have a meaningful stop()
                    pass
            actor_id = actor.id
            actor.destroy()
            logging.info("Destroyed actor id=%d", actor_id)
        except Exception as e:
            logging.warning("Error while destroying actor: %s", e)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    logging.info("Connecting to CARLA at %s:%d", args.host, args.port)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    logging.info("Connected. Current map: %s", world.get_map().name)

    walker = None
    controller = None

    try:
        walker, controller = spawn_walker_with_controller(world, args)
        run_trigger_loop(world, walker, controller, args)
    except KeyboardInterrupt:
        logging.info("Interrupted by user.")
    finally:
        cleanup(walker, controller)


if __name__ == "__main__":
    main()

