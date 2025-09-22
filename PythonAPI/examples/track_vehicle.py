#!/usr/bin/env python3
"""
CARLA: Track an existing vehicle by index (no manual control)

Usage:
    python carla_track_existing_vehicle.py --vehicle-index 0 [--host 127.0.0.1 --port 2000 --res 1920x1024 --sync --gamma 2.2]

Notes:
- This script does NOT spawn a vehicle and does NOT accept driving input.
- It simply finds an existing vehicle by index, attaches a camera that follows it, and renders the view.
- Use TAB to change camera positions, N to cycle sensors, F1 to toggle HUD, and ESC to quit.
"""
from __future__ import print_function

import glob
import os
import sys
import argparse
import logging
import math
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_F1, K_TAB, K_BACKQUOTE, K_n
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# ----------------------------- Utilities ------------------------------------

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# ----------------------------- HUD ------------------------------------------

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            'Location:% 20s' % ('(% 5.1f, % 5.1f, % 4.1f)' % (t.location.x, t.location.y, t.location.z)),
        ]

    def toggle_info(self):
        self._show_info = not self._show_info

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((260, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                surface = self._font_mono.render(item, True, (255, 255, 255))
                display.blit(surface, (8, v_offset))
                v_offset += 18

# ----------------------------- Camera ---------------------------------------

class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.images_panel = None
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        # A few nice cinematic views
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-6.0*bound_x, z=2.5*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),  # chase far
            (carla.Transform(carla.Location(x=-2.0*bound_x, z=1.5*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),  # chase near
            (carla.Transform(carla.Location(x=0.8*bound_x, z=1.3*bound_z)), Attachment.Rigid),  # hood
            (carla.Transform(carla.Location(x=0, y=-0.37, z=1.2), carla.Rotation(pitch=-15.0)), Attachment.Rigid),  # dash
            (carla.Transform(carla.Location(x=0, y=-6.0, z=3.0), carla.Rotation(yaw=90.0, pitch=-10.0)), Attachment.SpringArmGhost),  # side
        ]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Semantic Segmentation', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Depth (Log)', {}],
        ]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        self.index = None

        # spawn default sensor
        self.set_sensor(0, notify=False)

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

# ----------------------------- World -----------------------------------------

class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.hud = hud
        self.sync = args.sync
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            sys.exit(1)

        self.player = self._get_tracked_vehicle(args.vehicle_index)
        if self.player is None:
            print('No vehicle found at index %d. Exiting.' % args.vehicle_index)
            sys.exit(1)

        self.camera_manager = CameraManager(self.player, self.hud, args.gamma)
        self.world.on_tick(hud.on_world_tick)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _get_tracked_vehicle(self, index):
        vehicles = list(self.world.get_actors().filter('vehicle.*'))
        if not vehicles:
            print('No vehicles found in the simulation.')
            return None
        vehicles.sort(key=lambda v: v.id)  # deterministic ordering
        if index < 0 or index >= len(vehicles):
            print('Vehicle index out of range. There are %d vehicles (0..%d).' % (len(vehicles), len(vehicles)-1))
            return None
        tracked = vehicles[index]
        print('Tracking vehicle #%d: id=%s, type=%s' % (index, tracked.id, get_actor_display_name(tracked)))
        return tracked

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

# ----------------------------- Game Loop -------------------------------------

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        sim_world = client.get_world()

        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        display = pygame.display.set_mode((args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0, 0, 0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        running = True
        while running:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYUP:
                    if event.key == K_ESCAPE:
                        running = False
                    elif event.key == K_F1:
                        hud.toggle_info()
                    elif event.key == K_TAB:
                        world.camera_manager.toggle_camera()
                    elif event.key == K_BACKQUOTE or event.key == K_n:
                        world.camera_manager.next_sensor()

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)
        if world is not None and world.player is not None:
            # Do not destroy the tracked actor!
            pass
        pygame.quit()

# ----------------------------- Main -----------------------------------------

def main():
    parser = argparse.ArgumentParser(description='CARLA: Track existing vehicle (no control)')
    parser.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    parser.add_argument('-p', '--port', default=2000, type=int, help='TCP port (default: 2000)')
    parser.add_argument('--res', metavar='WIDTHxHEIGHT', default='1920x1024', help='window resolution (default: 1920x1024)')
    parser.add_argument('--gamma', default=2.2, type=float, help='Gamma correction of the camera (default: 2.2)')
    parser.add_argument('--sync', action='store_true', help='Activate synchronous mode execution')
    parser.add_argument('--vehicle-index', type=int, default=0, help='Index of the existing vehicle to track (sorted by actor id)')
    args = parser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    logging.info('connecting to server %s:%s', args.host, args.port)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
