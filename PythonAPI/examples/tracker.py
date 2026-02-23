#!/usr/bin/env python3
import argparse
import math
import glob
import os
import signal
import sys
from typing import Any, Optional

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def custom_print(kind: str, **kwargs: Any) -> None:
    print(str, str(kwargs))


def kmh_from_velocity(v: carla.Vector3D) -> float:
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class HeroTelemetry:
    def __init__(self, world: carla.World):
        self.world = world
        self.hero: Optional[carla.Vehicle] = None
        self.collision_sensor: Optional[carla.Actor] = None

        self.start_frame: Optional[int] = None
        self._last_collision_frame: Optional[int] = None
        self._colliding_recently: bool = False
        self._shutdown = False

    def shutdown(self) -> None:
        self._shutdown = True
        self._destroy_collision_sensor()

    def _destroy_collision_sensor(self) -> None:
        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
            except Exception:
                pass
            try:
                self.collision_sensor.destroy()
            except Exception:
                pass
            self.collision_sensor = None

    def _find_hero(self) -> Optional[carla.Vehicle]:
        for actor in self.world.get_actors().filter("vehicle.*"):
            try:
                if actor.attributes.get("role_name", "") == "hero":
                    return actor
            except Exception:
                continue
        return None

    def _attach_collision_sensor(self, vehicle: carla.Vehicle) -> None:
        self._destroy_collision_sensor()

        bp_lib = self.world.get_blueprint_library()
        col_bp = bp_lib.find("sensor.other.collision")
        sensor = self.world.spawn_actor(col_bp, carla.Transform(), attach_to=vehicle)
        self.collision_sensor = sensor

        def _on_collision(event: carla.CollisionEvent) -> None:
            frame_abs = int(event.frame)

            # "collision start" if we didn't have a collision in the last 2 frames
            recently = (
                self._colliding_recently
                and self._last_collision_frame is not None
                and (frame_abs - self._last_collision_frame) <= 2
            )

            if not recently and self.start_frame is not None and self.hero is not None:
                other = event.other_actor
                other_id = int(other.id) if other is not None else -1
                other_type = str(other.type_id) if other is not None else "unknown"

                impulse = event.normal_impulse
                impulse_mag = float(math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2))

                custom_print(
                    "collision_start",
                    frame_abs=frame_abs,
                    frame_elapsed=frame_abs - self.start_frame,
                    vehicle_id=int(self.hero.id),
                    other_actor_id=other_id,
                    other_actor_type=other_type,
                    impulse=impulse_mag,
                )

            self._last_collision_frame = frame_abs
            self._colliding_recently = True

        sensor.listen(_on_collision)

    def _ensure_hero(self, snapshot: carla.WorldSnapshot) -> None:
        hero_ok = self.hero is not None
        if hero_ok:
            try:
                _ = self.hero.get_location()
            except Exception:
                hero_ok = False

        if not hero_ok:
            new_hero = self._find_hero()
            if new_hero is not None:
                self.hero = new_hero
                self.start_frame = int(snapshot.frame)
                self._last_collision_frame = None
                self._colliding_recently = False
                self._attach_collision_sensor(new_hero)
                custom_print(
                    "status",
                    message=f"Tracking HERO id={new_hero.id} type={new_hero.type_id} start_frame={self.start_frame}",
                )

    def on_tick(self, snapshot: carla.WorldSnapshot) -> None:
        if self._shutdown:
            return

        self._ensure_hero(snapshot)
        if self.hero is None or self.start_frame is None:
            return

        frame_abs = int(snapshot.frame)

        # decay "colliding_recently"
        if self._last_collision_frame is not None and (frame_abs - self._last_collision_frame) > 2:
            self._colliding_recently = False

        try:
            speed_kmh = kmh_from_velocity(self.hero.get_velocity())
            ctrl = self.hero.get_control()

            throttle = float(ctrl.throttle)  # 0..1
            brake = float(ctrl.brake)        # 0..1
            steer = float(ctrl.steer)        # -1..1

            steer_deg_0_360 = (steer + 1.0) * 180.0
            frame_elapsed = frame_abs - self.start_frame

            custom_print(
                "frame",
                frame_abs=frame_abs,
                frame_elapsed=frame_elapsed,
                vehicle_id=int(self.hero.id),
                vehicle_type=str(self.hero.type_id),
                speed_kmh=float(speed_kmh),
                brake=brake,
                throttle=throttle,
                steer_deg_0_360=float(steer_deg_0_360),
            )
        except Exception:
            # hero despawned mid-tick
            self.hero = None
            self.start_frame = None
            self._destroy_collision_sensor()
            custom_print("status", message="Lost hero vehicle; waiting to reacquire...")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--sync", action="store_true", help="Tick the world from this script (sync mode).")
    ap.add_argument("--fixed-dt", type=float, default=0.05, help="Fixed delta seconds if --sync.")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    original_settings = world.get_settings()
    applied_sync = False

    telem = HeroTelemetry(world)

    def _sigint_handler(sig: Any, frame: Any) -> None:
        telem.shutdown()
        if applied_sync:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint_handler)

    if args.sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.fixed_dt
        world.apply_settings(settings)
        applied_sync = True

    try:
        while True:
            if args.sync:
                world.tick()
                snapshot = world.get_snapshot()
            else:
                snapshot = world.wait_for_tick()
            telem.on_tick(snapshot)
    finally:
        telem.shutdown()
        if applied_sync:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass


if __name__ == "__main__":
    main()
