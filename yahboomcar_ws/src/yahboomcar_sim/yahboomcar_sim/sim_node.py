#!/usr/bin/env python3
"""Simulador 2D del robot Yahboom (X3, mecanum) para RViz, con pistas de carrera.

Reemplaza al hardware del robot real:
  suscribe   /cmd_vel        geometry_msgs/Twist (vx, vy, wz en el marco del robot)
             /initialpose    (RViz "2D Pose Estimate") teletransporta el robot [standalone]
             /beep           std_msgs/Bool (solo se registra en el log)
  publica    /odom           nav_msgs/Odometry            odom -> base_footprint
             TF              odom -> base_footprint
             /scan           sensor_msgs/LaserScan        lidar MS200 simulado
             /imu/data_raw   sensor_msgs/Imu
             /vel_raw        geometry_msgs/Twist          velocidad real medida
             /voltage        std_msgs/Float32             bateria
             /camera/image_raw, /camera/camera_info       camara frontal simulada
             /sim/pose       geometry_msgs/PoseStamped    pose real (ground truth) en `map`
             /sim/collision  std_msgs/Bool
             /race/status    std_msgs/String (JSON)       vueltas, tiempos, choques
             /race/lap_time  std_msgs/Float32             tiempo de cada vuelta completada
             /race/track_markers, /race/hud               marcadores de RViz de la pista
  servicios  /sim/reset      std_srvs/Empty               robot a la salida y carrera a cero
  (modo standalone) /map y TF estatico map -> odom

El mundo es un mapa de ocupacion (.yaml + .pgm). Si junto al mapa hay un
<mapa>.track.yaml, el nodo conoce la pista (salida, vueltas, camara con piso).
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .camera import Camera, floor_from_world
from .race import RaceTracker
from .track import Track, roughen_floor
from .vehicle import Vehicle
from .world import WorldMap

CAMERA_HFOV = 1.2          # rad
CAMERA_PITCH = 0.35        # rad, hacia abajo


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def occupancy_grid(world, frame_id, stamp):
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.header.stamp = stamp
    info = MapMetaData()
    info.resolution = world.resolution
    info.width = world.width
    info.height = world.height
    info.origin.position.x = world.origin[0]
    info.origin.position.y = world.origin[1]
    info.origin.orientation.w = 1.0
    grid.info = info
    data = np.full(world.occupied.shape, -1, dtype=np.int8)
    data[world.free] = 0
    data[world.occupied] = 100
    grid.data = data.flatten().tolist()
    return grid


# Escenarios de ruido: valen para cualquier mundo (pistas o mapas propios).
NOISE_LEVELS = {
    'none':   {'floor_noise': 0.0,  'floor_grain': 0.0,  'odom_noise': 0.0,  'scan_noise': 0.0,   'imu_noise': 0.0},
    'low':    {'floor_noise': 0.02, 'floor_grain': 0.02, 'odom_noise': 0.01, 'scan_noise': 0.005, 'imu_noise': 0.005},
    'medium': {'floor_noise': 0.04, 'floor_grain': 0.04, 'odom_noise': 0.02, 'scan_noise': 0.01,  'imu_noise': 0.01},
    'high':   {'floor_noise': 0.10, 'floor_grain': 0.10, 'odom_noise': 0.05, 'scan_noise': 0.02,  'imu_noise': 0.03},
}


class SimRobot(Node):
    def __init__(self):
        super().__init__('yahboomcar_sim')

        # ---------------------------------------------------------- parametros
        P = self.declare_parameter
        P('map_yaml', '')
        P('track_yaml', '')                 # '' = <mapa>.track.yaml si existe
        P('start', 'auto')                  # 'auto' o 'x,y,yaw' en el marco map
        P('unknown_as_wall', True)
        P('publish_map', False)             # standalone: publicar /map y map->odom
        P('publish_map_to_odom', False)
        P('odom_frame', 'odom')
        P('base_frame', 'base_footprint')
        P('laser_frame', 'laser_frame')
        P('imu_frame', 'imu_Link')
        P('camera_frame', 'camera_link')
        P('laser_x', -0.0046412)
        P('laser_y', 0.0)
        P('robot_radius', -1.0)             # <0: el de la pista, o 0.20 en mapas sin pista
        P('update_rate', 50.0)
        P('scan_rate', 10.0)
        P('scan_samples', 450)              # MS200: 360 grados, 0.8 grados/punto
        P('range_min', 0.03)
        P('range_max', 12.0)
        P('scan_noise', 0.005)
        P('cmd_timeout', 0.5)
        # dinamica
        P('max_vx', 1.0)
        P('max_vy', 1.0)
        P('max_wz', 3.0)
        P('accel_lin', 2.5)
        P('accel_ang', 8.0)
        P('odom_noise', 0.01)               # error multiplicativo de la odometria
        P('noise_level', 'custom')         # custom | none | low | medium | high (ruido de todo el entorno)
        P('floor_noise', 0.0)               # m/s rms: empuje aleatorio del piso (no lo miden las ruedas)
        P('floor_grain', 0.0)               # suciedad visual del piso en la camara (0.05 = leve)
        P('odom_align', 0.0)                # 1/s: map->odom realinea la odometria con la pose real (0 = no)
        # sensores extra
        P('imu_rate', 50.0)
        P('imu_noise', 0.01)
        P('battery_voltage', 12.4)
        P('camera', True)
        P('camera_rate', 15.0)
        P('camera_width', 160)
        P('camera_height', 120)
        # carrera
        P('laps', 0)                        # 0 = sin limite
        P('seed', 0)

        gp = self.get_parameter
        level = str(gp('noise_level').value).lower()
        if level in ('off', 'false'):            # ROS/YAML convierte `off` en booleano false
            level = 'none'
        if level != 'custom' and level not in NOISE_LEVELS:
            raise RuntimeError(f"noise_level='{level}' invalido: {['custom'] + list(NOISE_LEVELS)}")
        if level in NOISE_LEVELS:
            vals = NOISE_LEVELS[level]
            self.set_parameters([Parameter(k, value=v) for k, v in vals.items()])
            self.get_logger().info(f'Nivel de ruido "{level}": {vals}')
        map_yaml = gp('map_yaml').value
        if not map_yaml or not os.path.isfile(map_yaml):
            raise RuntimeError(f"Parametro map_yaml invalido: '{map_yaml}'")
        self.world = WorldMap(map_yaml, gp('unknown_as_wall').value)

        track_yaml = gp('track_yaml').value
        self.track = Track.load(track_yaml) if track_yaml else Track.for_map(map_yaml)
        self.get_logger().info(
            f'Mundo: {os.path.basename(map_yaml)} '
            f'({self.world.width}x{self.world.height} celdas, {self.world.resolution} m/celda)'
            + (f' | pista "{self.track.name}" ({self.track.level}), '
               f'{self.track.length:.2f} m' if self.track else ''))

        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.laser_frame = gp('laser_frame').value
        self.imu_frame = gp('imu_frame').value
        self.camera_frame = gp('camera_frame').value
        self.laser_xy = (gp('laser_x').value, gp('laser_y').value)
        self.range_min = gp('range_min').value
        self.range_max = gp('range_max').value
        self.noise = gp('scan_noise').value
        self.imu_noise = gp('imu_noise').value
        self.cmd_timeout = gp('cmd_timeout').value
        self.n_samples = int(gp('scan_samples').value)
        self.standalone = bool(gp('publish_map').value)
        seed = int(gp('seed').value)
        self.rng = np.random.default_rng(seed if seed else None)

        radius = gp('robot_radius').value
        if radius <= 0.0:
            radius = self.track.robot_radius if self.track else 0.20
        self.radius = radius

        # ---------------------------------------------------------- pose inicial
        start = str(gp('start').value).strip().lower()
        if start == 'auto':
            if self.track:
                sx, sy, syaw = self.track.start_pose
            else:
                sx, sy = self.world.auto_start(self.radius + 0.10)
                syaw = 0.0
        else:
            try:
                sx, sy, syaw = (float(v) for v in start.split(','))
            except ValueError:
                raise RuntimeError(f"start='{start}' invalido: use 'auto' o 'x,y,yaw'")
        self.start_pose = (sx, sy, syaw)

        self.veh = Vehicle(
            self.world, sx, sy, syaw, radius=self.radius,
            max_vx=gp('max_vx').value, max_vy=gp('max_vy').value, max_wz=gp('max_wz').value,
            accel_lin=gp('accel_lin').value, accel_ang=gp('accel_ang').value,
            odom_noise=gp('odom_noise').value, floor_noise=gp('floor_noise').value,
            seed=seed if seed else None)
        if self.veh.collides(sx, sy):
            self.get_logger().warn('La pose inicial esta dentro de una pared u obstaculo')
        self.get_logger().info(
            f'Robot: radio {self.radius:.2f} m | pose inicial en map: '
            f'x={sx:.2f} y={sy:.2f} yaw={syaw:.2f}')

        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.prev_v = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0)
        self.energy_used = 0.0

        self.race = RaceTracker(self.track, int(gp('laps').value)) if self.track else None
        if self.race:
            self.race.reset((sx, sy), self._now_s())

        # ---------------------------------------------------------- lidar / camara
        inc = 2.0 * math.pi / self.n_samples
        self.angle_min = -math.pi
        self.angle_inc = inc
        self.beam_angles = self.angle_min + inc * np.arange(self.n_samples)
        self.ray_t = np.arange(self.range_min, self.range_max, self.world.resolution / 2.0)

        self.camera = None
        if gp('camera').value:
            floor = self.track.render_floor() if self.track else floor_from_world(self.world)
            floor = roughen_floor(floor, gp('floor_grain').value, seed=seed or 1)
            self.camera = Camera(self.world, floor, width=int(gp('camera_width').value),
                                 height=int(gp('camera_height').value),
                                 hfov=CAMERA_HFOV, pitch=CAMERA_PITCH)
            self.cam_info = self._make_camera_info()

        # ---------------------------------------------------------- comunicacion
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, 'beep', self._on_beep, 10)
        if self.standalone:
            self.create_subscription(PoseWithCovarianceStamped, 'initialpose',
                                     self._on_initialpose, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.vel_pub = self.create_publisher(Twist, 'vel_raw', 10)
        self.volt_pub = self.create_publisher(Float32, 'voltage', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'sim/pose', 10)
        self.coll_pub = self.create_publisher(Bool, 'sim/collision', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._static_tfs = {}
        self.create_service(Empty, 'sim/reset', self._on_reset)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        if self.standalone:
            self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
            self.map_pub.publish(occupancy_grid(self.world, 'map', self.get_clock().now().to_msg()))
        self.odom_align = float(gp('odom_align').value)
        self.mo = (sx, sy, syaw)                 # map->odom actual (x, y, yaw)
        self.dynamic_mo = bool(gp('publish_map_to_odom').value) and self.odom_align > 0.0
        if gp('publish_map_to_odom').value and not self.dynamic_mo:
            self._send_map_to_odom(sx, sy, syaw)

        if self.camera:
            self.image_pub = self.create_publisher(Image, 'camera/image_raw', 5)
            self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 5)
            self.create_timer(1.0 / gp('camera_rate').value, self._publish_camera)
            self._send_camera_tf()
        if self.race:
            self.status_pub = self.create_publisher(String, 'race/status', 10)
            self.lap_pub = self.create_publisher(Float32, 'race/lap_time', 10)
            self.hud_pub = self.create_publisher(Marker, 'race/hud', 10)
            self.marker_pub = self.create_publisher(MarkerArray, 'race/track_markers', latched)
            self.marker_pub.publish(self._track_markers())
            self.create_timer(0.2, self._publish_race)

        self.dt = 1.0 / gp('update_rate').value
        self.last_update = self.get_clock().now()
        self.create_timer(self.dt, self._update)
        self.create_timer(1.0 / gp('scan_rate').value, self._publish_scan)
        self.create_timer(1.0 / gp('imu_rate').value, self._publish_imu)
        self.create_timer(1.0, self._publish_voltage)

    # ------------------------------------------------------------------ util
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_beep(self, msg):
        self.get_logger().info('BEEP' if msg.data else 'beep off')

    def _send_map_to_odom(self, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = yaw_to_quat(yaw)
        self._send_static(t)

    def _send_static(self, transform):
        """Envia un TF estatico junto con todos los anteriores.

        En Humble el broadcaster estatico solo retiene el ultimo mensaje para los
        suscriptores tardios; si se enviaran por separado, uno pisaria al otro.
        """
        self._static_tfs[transform.child_frame_id] = transform
        self.static_tf.sendTransform(list(self._static_tfs.values()))

    def _teleport(self, x, y, yaw):
        """Coloca el robot y reinicia odometria y carrera (map->odom se reenvia)."""
        self.veh.reset(x, y, yaw)
        self.cmd = Twist()
        self.prev_v = (0.0, 0.0, 0.0)
        if self.race:
            self.race.reset((x, y), self._now_s())
        self.mo = (x, y, yaw)
        if self.standalone and not self.dynamic_mo:
            self._send_map_to_odom(x, y, yaw)
        if self.veh.collides(x, y):
            self.get_logger().warn('La nueva pose esta dentro de una pared u obstaculo')

    def _on_reset(self, request, response):
        self.get_logger().info('Reinicio: robot a la salida, carrera a cero')
        self._teleport(*self.start_pose)
        return response

    def _on_initialpose(self, msg):
        if msg.header.frame_id not in ('map', ''):
            self.get_logger().warn(f'initialpose en marco "{msg.header.frame_id}" ignorada')
            return
        p = msg.pose.pose
        self.get_logger().info(f'Teletransporte a x={p.position.x:.2f} y={p.position.y:.2f}')
        self._teleport(p.position.x, p.position.y, quat_to_yaw(p.orientation))

    # ------------------------------------------------------------ cinematica
    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now
        if dt <= 0.0 or dt > 1.0:
            dt = self.dt

        stale = (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout
        cmd = (0.0, 0.0, 0.0) if stale else (
            self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z)
        v = self.veh
        v.step(cmd, dt)
        self.energy_used += (0.5 + 3.0 * v.speed) * dt          # consumo ficticio -> bateria
        self.accel = ((v.vx - self.prev_v[0]) / dt, (v.vy - self.prev_v[1]) / dt)
        self.prev_v = (v.vx, v.vy, v.wz)

        stamp = now.to_msg()
        if self.dynamic_mo:
            self._align_odom(dt)
        self._publish_odom(stamp)
        self._publish_truth(stamp)
        if self.race:
            for ev in self.race.update(v.x, v.y, v.yaw, now.nanoseconds * 1e-9,
                                       v.speed, v.colliding):
                if ev == 'lap':
                    lt = self.race.lap_times[-1]
                    self.lap_pub.publish(Float32(data=float(lt)))
                    self.get_logger().info(
                        f'Vuelta {self.race.lap} completada: {lt:.2f} s '
                        f'(mejor {self.race.best_lap:.2f} s)')
                elif ev == 'finish':
                    self.get_logger().info(
                        f'CARRERA TERMINADA: {self.race.total_time:.2f} s, '
                        f'{self.race.collisions} choques')

    def _align_odom(self, dt):
        """Acerca map->odom a la pose real, como lo haria un localizador (AMCL / SLAM).

        La odometria (/odom) sigue acumulando error, pero map->base_footprint se va
        alineando con la realidad a razon de `odom_align` (1/s) mientras el robot recorre.
        """
        v = self.veh
        # map->odom ideal = T_map_base * inv(T_odom_base)
        c, s_ = math.cos(v.oyaw), math.sin(v.oyaw)
        iyaw = v.yaw - v.oyaw
        ci, si = math.cos(iyaw), math.sin(iyaw)
        tx = v.x - (ci * v.ox - si * v.oy)
        ty = v.y - (si * v.ox + ci * v.oy)
        k = 1.0 - math.exp(-self.odom_align * dt)
        mx, my, myaw = self.mo
        dyaw = (iyaw - myaw + math.pi) % (2.0 * math.pi) - math.pi
        self.mo = (mx + k * (tx - mx), my + k * (ty - my), myaw + k * dyaw)

    def _publish_odom(self, stamp):
        v = self.veh
        qx, qy, qz, qw = yaw_to_quat(v.oyaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = v.ox
        odom.pose.pose.position.y = v.oy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v.vx
        odom.twist.twist.linear.y = v.vy
        odom.twist.twist.angular.z = v.wz
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e-3
            odom.twist.covariance[i] = 1e-3
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = v.ox
        t.transform.translation.y = v.oy
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        if self.dynamic_mo:
            m = TransformStamped()
            m.header.stamp = stamp
            m.header.frame_id = 'map'
            m.child_frame_id = self.odom_frame
            m.transform.translation.x = self.mo[0]
            m.transform.translation.y = self.mo[1]
            (m.transform.rotation.x, m.transform.rotation.y,
             m.transform.rotation.z, m.transform.rotation.w) = yaw_to_quat(self.mo[2])
            self.tf_broadcaster.sendTransform([t, m])
        else:
            self.tf_broadcaster.sendTransform(t)

        vel = Twist()
        vel.linear.x, vel.linear.y, vel.angular.z = v.vx, v.vy, v.wz
        self.vel_pub.publish(vel)

    def _publish_truth(self, stamp):
        v = self.veh
        p = PoseStamped()
        p.header.stamp = stamp
        p.header.frame_id = 'map'
        p.pose.position.x = v.x
        p.pose.position.y = v.y
        p.pose.orientation.z = math.sin(v.yaw / 2.0)
        p.pose.orientation.w = math.cos(v.yaw / 2.0)
        self.pose_pub.publish(p)
        self.coll_pub.publish(Bool(data=bool(v.colliding)))

    # ----------------------------------------------------------------- lidar
    def _publish_scan(self):
        v = self.veh
        c, s = math.cos(v.yaw), math.sin(v.yaw)
        lx = v.x + c * self.laser_xy[0] - s * self.laser_xy[1]
        ly = v.y + s * self.laser_xy[0] + c * self.laser_xy[1]
        ranges = self.world.raycast(lx, ly, self.beam_angles + v.yaw, self.ray_t)
        if self.noise > 0.0:
            hit = np.isfinite(ranges)
            ranges = np.where(hit, ranges + self.rng.normal(0.0, self.noise, ranges.shape),
                              ranges)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_min + self.angle_inc * (self.n_samples - 1))
        scan.angle_increment = float(self.angle_inc)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)

    # ------------------------------------------------------------------- IMU
    def _publish_imu(self):
        v = self.veh
        n = self.imu_noise
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        _, _, qz, qw = yaw_to_quat(v.oyaw)
        msg.orientation.z, msg.orientation.w = qz, qw
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 1e-3
            msg.angular_velocity_covariance[i] = 1e-4
            msg.linear_acceleration_covariance[i] = 1e-2
        msg.angular_velocity.x = float(self.rng.normal(0.0, n * 0.05))
        msg.angular_velocity.y = float(self.rng.normal(0.0, n * 0.05))
        msg.angular_velocity.z = v.wz + float(self.rng.normal(0.0, n * 0.2))
        msg.linear_acceleration.x = self.accel[0] + float(self.rng.normal(0.0, n))
        msg.linear_acceleration.y = self.accel[1] + float(self.rng.normal(0.0, n))
        msg.linear_acceleration.z = 9.81 + float(self.rng.normal(0.0, n))
        self.imu_pub.publish(msg)

    def _publish_voltage(self):
        volts = self.get_parameter('battery_voltage').value - self.energy_used * 2e-5
        self.volt_pub.publish(Float32(data=float(max(volts, 9.0))))

    # ---------------------------------------------------------------- camara
    def _make_camera_info(self):
        w, h = self.camera.width, self.camera.height
        f = (w / 2.0) / math.tan(CAMERA_HFOV / 2.0)
        info = CameraInfo()
        info.header.frame_id = self.camera_frame
        info.width, info.height = w, h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [f, 0.0, w / 2.0, 0.0, f, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, w / 2.0, 0.0, 0.0, f, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _send_camera_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = self.camera.cam_x
        t.transform.translation.z = self.camera.cam_h
        t.transform.rotation.y = math.sin(CAMERA_PITCH / 2.0)      # cabeceo hacia abajo
        t.transform.rotation.w = math.cos(CAMERA_PITCH / 2.0)
        self._send_static(t)

    def _publish_camera(self):
        img = self.camera.render(self.veh.x, self.veh.y, self.veh.yaw)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'rgb8'
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.image_pub.publish(msg)
        self.cam_info.header.stamp = stamp
        self.info_pub.publish(self.cam_info)

    # ---------------------------------------------------------------- carrera
    def _publish_race(self):
        d = self.race.as_dict()
        self.status_pub.publish(String(data=json.dumps(d)))

        hud = Marker()
        hud.header.frame_id = 'map'
        hud.header.stamp = self.get_clock().now().to_msg()
        hud.ns, hud.id = 'hud', 0
        hud.type, hud.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        bx, by = self.track.box
        hud.pose.position.x, hud.pose.position.y, hud.pose.position.z = bx / 2.0, by / 2.0, 0.05
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.11
        hud.color.r = hud.color.g = hud.color.b = hud.color.a = 1.0
        laps = f"/{d['total_laps']}" if d['total_laps'] else ''
        best = f"{d['best_lap']:.2f} s" if d['best_lap'] is not None else '--'
        last = f"{d['last_lap']:.2f} s" if d['last_lap'] is not None else '--'
        state = {'ready': 'LISTO', 'running': 'EN CARRERA', 'finished': 'TERMINADA'}[d['status']]
        wrong = '\n! CONTRAMANO !' if d['wrong_way'] else ''
        hud.text = (f"{self.track.name.replace('pista_', '').upper()}  {state}\n"
                    f"Vuelta {d['lap'] + (d['status'] == 'running')}{laps}  "
                    f"{d['lap_time']:.1f} s\nUltima {last}  Mejor {best}\n"
                    f"Choques {d['collisions']}{wrong}")
        self.hud_pub.publish(hud)

    def _track_markers(self):
        tr = self.track
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id, line.header.stamp = 'map', stamp
        line.ns, line.id = 'track', 0
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.012
        line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.9, 0.3, 0.9
        line.pose.orientation.w = 1.0
        for k in range(0, tr.n + 1, 4):
            p = tr.center[k % tr.n]
            line.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.01))
        arr.markers.append(line)

        sx, sy, syaw = tr.start_pose
        gate = Marker()
        gate.header.frame_id, gate.header.stamp = 'map', stamp
        gate.ns, gate.id = 'track', 1
        gate.type, gate.action = Marker.CUBE, Marker.ADD
        gate.pose.position.x, gate.pose.position.y, gate.pose.position.z = sx, sy, 0.012
        gate.pose.orientation.z, gate.pose.orientation.w = math.sin(syaw / 2), math.cos(syaw / 2)
        gate.scale.x, gate.scale.y, gate.scale.z = 0.04, tr.lane_width, 0.004
        gate.color.r = gate.color.g = gate.color.b = gate.color.a = 1.0
        arr.markers.append(gate)

        for i, k in enumerate(tr.checkpoints[1:], start=2):
            cp = Marker()
            cp.header.frame_id, cp.header.stamp = 'map', stamp
            cp.ns, cp.id = 'track', i
            cp.type, cp.action = Marker.CYLINDER, Marker.ADD
            cp.pose.position.x = float(tr.center[k, 0])
            cp.pose.position.y = float(tr.center[k, 1])
            cp.pose.position.z = 0.012
            cp.pose.orientation.w = 1.0
            cp.scale.x = cp.scale.y = 0.05
            cp.scale.z = 0.004
            cp.color.r, cp.color.g, cp.color.b, cp.color.a = 1.0, 0.8, 0.1, 0.9
            arr.markers.append(cp)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
