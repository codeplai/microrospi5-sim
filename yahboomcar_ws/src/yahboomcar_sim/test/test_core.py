"""Pruebas del nucleo del simulador (sin ROS): pistas, fisica, carrera, camara, piloto.

Ejecutar desde la carpeta del paquete:   python -m unittest discover -s test -v
"""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from yahboomcar_sim.camera import Camera                    # noqa: E402
from yahboomcar_sim.pilot import PID, PIDFollower, PurePursuit               # noqa: E402
from yahboomcar_sim.race import RaceTracker                 # noqa: E402
from yahboomcar_sim.track import Track, roughen_floor        # noqa: E402
from yahboomcar_sim.track_gen import build_tracks, write_arena   # noqa: E402
from yahboomcar_sim.vehicle import Vehicle                  # noqa: E402
from yahboomcar_sim.world import WorldMap                   # noqa: E402

WORLDS = os.path.join(PKG, 'worlds')
NAMES = ['pista_basica', 'pista_intermedia', 'pista_avanzada']   # las 3 de 3x2 m (por dificultad)


def load(name):
    return (Track.load(os.path.join(WORLDS, name + '.track.yaml')),
            WorldMap(os.path.join(WORLDS, name + '.yaml')))


def drive_laps(name, laps=2, speed=0.6, limit=150.0):
    track, world = load(name)
    sx, sy, syaw = track.start_pose
    veh = Vehicle(world, sx, sy, syaw, radius=track.robot_radius)
    race = RaceTracker(track, laps)
    race.reset((sx, sy))
    pilot = PurePursuit(track, v_max=speed)
    t, dt = 0.0, 0.02
    while t < limit and race.status != 'finished':
        veh.step(pilot.command(veh.x, veh.y, veh.yaw), dt)
        t += dt
        race.update(veh.x, veh.y, veh.yaw, t, veh.speed, veh.colliding)
    return race


class TestPID(unittest.TestCase):
    def test_pid_converges_and_limits_output(self):
        pid = PID(2.0, 0.5, 0.1, out_limit=1.0)
        x = 1.0
        for _ in range(400):
            x += pid.update(0.0 - x, 0.02) * 0.02 * 5.0       # planta integradora simple
        self.assertLess(abs(x), 0.02)
        big = PID(100.0, 1.0, 0.0, out_limit=0.5)
        self.assertLessEqual(abs(big.update(10.0, 0.02)), 0.5)
        for _ in range(500):                                   # anti-windup: integral acotada
            big.update(10.0, 0.02)
        self.assertLessEqual(abs(big.integral * big.ki), big.i_limit + 1e-9)

    def test_pid_follower_laps_every_track_with_rough_floor(self):
        for name in NAMES + ['pista_cuadrado']:
            track, world = load(name)
            sx, sy, syaw = track.start_pose
            veh = Vehicle(world, sx, sy, syaw, radius=track.robot_radius, floor_noise=0.10,
                          odom_noise=0.05, seed=2)
            race = RaceTracker(track, 2)
            race.reset((sx, sy))
            pid = PIDFollower(track, v_max=0.6)
            t, dt, worst = 0.0, 0.02, 0.0
            while t < 150 and race.status != 'finished':
                veh.step(pid.command(veh.x, veh.y, veh.yaw, dt), dt)
                t += dt
                worst = max(worst, abs(pid.lateral))
                race.update(veh.x, veh.y, veh.yaw, t, veh.speed, veh.colliding)
            self.assertEqual(race.status, 'finished', name)
            self.assertEqual(race.collisions, 0, name)
            self.assertLess(worst, track.lane_width / 2.0, name)


class TestFloorNoise(unittest.TestCase):
    def test_square_track_laps_with_noisy_floor(self):
        track, world = load('pista_cuadrado')
        self.assertEqual(track.box, (2.0, 2.0))
        sx, sy, syaw = track.start_pose
        veh = Vehicle(world, sx, sy, syaw, radius=track.robot_radius, floor_noise=0.05,
                      odom_noise=0.01, seed=3)
        race = RaceTracker(track, 2)
        race.reset((sx, sy))
        pilot = PurePursuit(track, v_max=0.6)
        t, dt = 0.0, 0.02
        drift = 0.0
        while t < 120 and race.status != 'finished':
            veh.step(pilot.command(veh.x, veh.y, veh.yaw), dt)
            t += dt
            race.update(veh.x, veh.y, veh.yaw, t, veh.speed, veh.colliding)
        self.assertEqual(race.status, 'finished')
        self.assertEqual(race.collisions, 0)

    def test_floor_pushes_robot_and_odom_does_not_see_it(self):
        track, world = load('pista_cuadrado')
        sx, sy, syaw = track.start_pose
        veh = Vehicle(world, sx, sy, syaw, radius=track.robot_radius, floor_noise=0.1, seed=1)
        ref = Vehicle(world, sx, sy, syaw, radius=track.robot_radius, floor_noise=0.0)
        for _ in range(40):
            veh.step((0.5, 0.0, 0.0), 0.02)
            ref.step((0.5, 0.0, 0.0), 0.02)
        self.assertGreater(abs(veh.y - ref.y) + abs(veh.yaw - ref.yaw), 0.005)  # se desvia
        self.assertAlmostEqual(veh.ox, ref.ox, delta=0.01)                       # ruedas: igual
        self.assertAlmostEqual(veh.oy, 0.0, delta=0.01)

    def test_roughen_floor_changes_pixels_but_not_shape(self):
        track, _ = load('pista_cuadrado')
        clean = track.render_floor()
        dirty = roughen_floor(clean, 0.05, seed=1)
        self.assertEqual(clean.shape, dirty.shape)
        self.assertGreater(float(np.abs(dirty.astype(int) - clean.astype(int)).mean()), 1.0)
        self.assertTrue((roughen_floor(clean, 0.0) == clean).all())


class TestArena(unittest.TestCase):
    """Arena abierta 4 x 4 m del RC-1: el cuadrado de 2 x 2 m queda con 1 m de margen."""

    def test_arena_is_open_with_no_track_and_fits_a_2m_square(self):
        yaml_path = os.path.join(WORLDS, 'arena_rc1.yaml')
        world = WorldMap(yaml_path)
        self.assertAlmostEqual(world.width * world.resolution, 4.0)
        self.assertAlmostEqual(world.height * world.resolution, 4.0)
        self.assertIsNone(Track.for_map(yaml_path))              # sin carril ni carrera
        self.assertTrue(world.free[world.height // 2, world.width // 2])
        self.assertTrue(world.solid[0, :].all() and world.solid[:, -1].all())
        veh = Vehicle(world, 1.0, 1.0, 0.0, radius=0.14)         # radio que barre el MicroROS-Pi5
        for x, y in [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]:
            self.assertFalse(veh.collides(x, y))                 # esquinas del cuadrado de 2 m

    def test_saved_arena_matches_generator(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            base = write_arena(d)
            with open(base + '.pgm', 'rb') as a, \
                    open(os.path.join(WORLDS, 'arena_rc1.pgm'), 'rb') as b:
                self.assertEqual(a.read(), b.read())


class TestTracks(unittest.TestCase):
    def test_tracks_fit_the_3x2_table(self):
        for t in build_tracks():
            self.assertEqual(t.box, (2.0, 2.0) if t.name == 'pista_cuadrado' else (3.0, 2.0))
            t.validate()                       # no se sale ni se pisa a si misma

    def test_difficulty_increases_with_curves(self):
        stats = [load(n)[0].stats() for n in NAMES]
        radii = [s['min_radius_m'] for s in stats]
        self.assertGreater(radii[0], radii[1])
        self.assertGreater(radii[1], radii[2])
        turning = [s['total_turning_deg'] for s in stats]
        self.assertLess(turning[0], turning[1])
        self.assertLess(turning[1], turning[2])
        lanes = [load(n)[0].lane_width for n in NAMES]
        self.assertGreaterEqual(lanes[0], lanes[1])
        self.assertGreaterEqual(lanes[1], lanes[2])

    def test_saved_files_match_generator(self):
        for built in build_tracks():
            saved, _ = load(built.name)
            self.assertAlmostEqual(saved.length, built.length, delta=0.05)

    def test_map_has_lane_walls_and_start_is_free(self):
        for n in NAMES:
            track, world = load(n)
            self.assertEqual((world.width, world.height), (300, 200))
            sx, sy, _ = track.start_pose
            veh = Vehicle(world, sx, sy, 0.0, radius=track.robot_radius)
            self.assertFalse(veh.collides(sx, sy))
            self.assertTrue(world.is_solid(np.array([0.05]), np.array([0.05]))[0])

    def test_lane_fits_the_robot(self):
        for n in NAMES:
            track, _ = load(n)
            self.assertGreater(track.lane_width, 2 * track.robot_radius + 0.08)

    def test_project_returns_lateral_offset_sign(self):
        track, _ = load('pista_basica')
        x, y, yaw = track.start_pose
        _, lat_left = track.project(x - 0.0, y + 0.1, hint=0)
        _, lat_right = track.project(x - 0.0, y - 0.1, hint=0)
        self.assertGreater(lat_left, 0.0)
        self.assertLess(lat_right, 0.0)


class TestVehicle(unittest.TestCase):
    def setUp(self):
        self.track, self.world = load('pista_basica')
        x, y, yaw = self.track.start_pose
        self.veh = Vehicle(self.world, x, y, yaw, radius=self.track.robot_radius)

    def test_acceleration_is_limited(self):
        self.veh.step((1.0, 0.0, 0.0), 0.02)
        self.assertLess(self.veh.vx, 0.1)                 # no salta a 1 m/s de golpe
        for _ in range(25):            # 0.5 s: llega a 1 m/s sin alcanzar la primera curva
            self.veh.step((1.0, 0.0, 0.0), 0.02)
        self.assertGreater(self.veh.cvx, 0.9)

    def test_wall_blocks_and_flags_collision(self):
        # Sale de la pista contra la pared exterior (hacia -y).
        for _ in range(300):
            self.veh.step((0.0, -0.6, 0.0), 0.02)
        self.assertTrue(self.veh.colliding)
        self.assertFalse(self.veh.collides(self.veh.x, self.veh.y))   # nunca atraviesa
        self.assertGreater(self.veh.y, 0.1)

    def test_strafing_works(self):
        y0 = self.veh.y
        for _ in range(25):
            self.veh.step((0.0, 0.3, 0.0), 0.02)
        self.assertGreater(self.veh.y, y0 + 0.02)

    def test_odometry_matches_truth_without_noise(self):
        for _ in range(100):
            self.veh.step((0.5, 0.0, 0.3), 0.02)
        self.assertAlmostEqual(self.veh.oyaw, self.veh.yaw - self.track.start_pose[2], places=6)

    def test_odometry_noise_drifts(self):
        x, y, yaw = self.track.start_pose
        noisy = Vehicle(self.world, x, y, yaw, radius=0.13, odom_noise=0.05, seed=1)
        clean = Vehicle(self.world, x, y, yaw, radius=0.13)
        for _ in range(100):
            noisy.step((0.5, 0.0, 0.0), 0.02)
            clean.step((0.5, 0.0, 0.0), 0.02)
        self.assertNotAlmostEqual(noisy.ox, clean.ox, places=4)


class TestRace(unittest.TestCase):
    def test_autopilot_completes_laps_cleanly_on_every_track(self):
        for n in NAMES:
            race = drive_laps(n, laps=2)
            self.assertEqual(race.status, 'finished', n)
            self.assertEqual(race.lap, 2, n)
            self.assertEqual(race.collisions, 0, n)

    def test_harder_tracks_take_longer(self):
        times = [np.mean(drive_laps(n, laps=1).lap_times) for n in NAMES]
        self.assertLess(times[0], times[1])
        self.assertLess(times[1], times[2])

    def test_reverse_does_not_count_laps(self):
        track, world = load('pista_basica')
        sx, sy, syaw = track.start_pose
        race = RaceTracker(track, 0)
        race.reset((sx, sy))
        # Recorre la pista hacia atras con un "robot" fantasma sobre la linea central.
        t = 0.0
        for k in range(0, 2 * track.n, 3):
            i = (-k) % track.n
            x, y = track.center[i]
            t += 0.02
            race.update(x, y, track.heading[i] + math.pi, t, 0.5, False)
        self.assertEqual(race.lap, 0)
        self.assertTrue(race.wrong_way)

    def test_forward_ghost_counts_exact_laps(self):
        track, _ = load('pista_intermedia')
        race = RaceTracker(track, 0)
        race.reset(tuple(track.center[0]))
        t = 0.0
        for k in range(0, int(2.5 * track.n), 3):
            i = k % track.n
            x, y = track.center[i]
            t += 0.02
            race.update(x, y, track.heading[i], t, 0.5, False)
        self.assertEqual(race.lap, 2)
        self.assertEqual(len(race.lap_times), 2)


class TestCamera(unittest.TestCase):
    def test_image_shows_floor_and_walls(self):
        track, world = load('pista_avanzada')
        cam = Camera(world, track.render_floor())
        x, y, yaw = track.start_pose
        img = cam.render(x, y, yaw)
        self.assertEqual(img.shape, (120, 160, 3))
        self.assertEqual(img.dtype, np.uint8)
        # Debe haber asfalto gris en la parte baja y algo mas que un solo color.
        bottom = img[-1, :, :].astype(int)
        self.assertGreater((np.abs(bottom - np.array([78, 80, 86])).sum(axis=1) < 40).sum(), 40)
        self.assertGreater(len({tuple(p) for p in img.reshape(-1, 3)}), 5)

    def test_dashed_center_line_is_visible_ahead(self):
        track, world = load('pista_basica')
        cam = Camera(world, track.render_floor())
        x, y, yaw = track.start_pose
        img = cam.render(x - 0.3, y, yaw).astype(int)
        yellow = (np.abs(img - np.array([235, 200, 40])).sum(axis=2) < 60)
        self.assertGreater(yellow.sum(), 10)
        cols = np.nonzero(yellow)[1]
        self.assertLess(abs(cols.mean() - 80), 12)        # centrada en la imagen


class TestPilot(unittest.TestCase):
    def test_pilot_slows_down_for_tighter_curves(self):
        speeds = {}
        for n in NAMES:
            track, _ = load(n)
            pp = PurePursuit(track, v_max=1.0, a_lat=0.8)
            vs = []
            for i in range(0, track.n, 20):
                x, y = track.center[i]
                vs.append(pp.command(x, y, track.heading[i])[0])
                pp.reset()
            speeds[n] = min(vs)
        self.assertGreater(speeds['pista_basica'], speeds['pista_avanzada'])


if __name__ == '__main__':
    unittest.main()
