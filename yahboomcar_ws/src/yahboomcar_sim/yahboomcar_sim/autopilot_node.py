#!/usr/bin/env python3
"""Piloto automatico: recorre la pista con control PID (o pure pursuit).

Usa la pose real que publica el simulador (/sim/pose) y publica /cmd_vel. Sirve para
ver una vuelta de referencia, calibrar tiempos y comprobar que la pista es recorrible.

  ros2 run yahboomcar_sim autopilot --ros-args -p map_yaml:=<ruta>.yaml -p speed:=0.6

controller:=pid (por defecto) usa dos PID (desvio lateral y rumbo) que corrigen el empuje del
piso; controller:=pursuit es el piloto de referencia anterior. Publica los errores en
/pid/cross_track (m) y /pid/heading_error (rad) para graficarlos con rqt_plot.
pose_source:=truth usa /sim/pose; pose_source:=tf usa el TF map->base_footprint (la pose que
ve el robot, con la odometria realineada por odom_align).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32

from .pilot import PIDFollower, PurePursuit
from .track import Track


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Autopilot(Node):
    def __init__(self):
        super().__init__('yahboomcar_autopilot')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('track_yaml', '')
        self.declare_parameter('speed', 0.6)         # m/s maxima en rectas
        self.declare_parameter('lateral_accel', 0.8)  # m/s^2: limita la velocidad en curvas
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('controller', 'pid')      # pid | pursuit
        self.declare_parameter('pose_source', 'truth')   # truth | tf
        for name, val in (('kp_lat', 3.5), ('ki_lat', 0.3), ('kd_lat', 0.4),
                          ('kp_head', 3.0), ('ki_head', 0.0), ('kd_head', 0.15)):
            self.declare_parameter(name, val)

        gp = self.get_parameter
        track_yaml = gp('track_yaml').value
        track = Track.load(track_yaml) if track_yaml else (
            Track.for_map(gp('map_yaml').value) if gp('map_yaml').value else None)
        if track is None:
            raise RuntimeError('Se necesita track_yaml o un map_yaml con su .track.yaml al lado')
        self.controller = str(gp('controller').value).lower()
        if self.controller == 'pid':
            self.pilot = PIDFollower(
                track, v_max=gp('speed').value, a_lat=gp('lateral_accel').value,
                kp_lat=gp('kp_lat').value, ki_lat=gp('ki_lat').value, kd_lat=gp('kd_lat').value,
                kp_head=gp('kp_head').value, ki_head=gp('ki_head').value,
                kd_head=gp('kd_head').value)
        elif self.controller == 'pursuit':
            self.pilot = PurePursuit(track, v_max=gp('speed').value,
                                     a_lat=gp('lateral_accel').value)
        else:
            raise RuntimeError(f"controller='{self.controller}' invalido: use 'pid' o 'pursuit'")
        self.pose_source = str(gp('pose_source').value).lower()
        if self.pose_source == 'tf':
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        elif self.pose_source != 'truth':
            raise RuntimeError(f"pose_source='{self.pose_source}' invalido: use 'truth' o 'tf'")
        self.last_t = self.get_clock().now()
        self.err_lat_pub = self.create_publisher(Float32, 'pid/cross_track', 10)
        self.err_head_pub = self.create_publisher(Float32, 'pid/heading_error', 10)
        self.pose = None
        self.create_subscription(PoseStamped, 'sim/pose', self._on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / gp('rate').value, self._tick)
        self.get_logger().info(
            f'Piloto automatico ({self.controller}, pose={self.pose_source}) en "{track.name}" '
            f'a {gp("speed").value} m/s')

    def _on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, quat_to_yaw(msg.pose.orientation))

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if self.pose_source == 'tf':
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                    rclpy.time.Time()).transform
            except Exception:
                return
            self.pose = (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
        if self.pose is None:
            return
        if not 0.0 < dt < 0.5:
            dt = 1.0 / 30.0
        if self.controller == 'pid':
            vx, vy, wz = self.pilot.command(*self.pose, dt=dt)
            self.err_lat_pub.publish(Float32(data=float(self.pilot.lateral)))
            self.err_head_pub.publish(Float32(data=float(self.pilot.heading_err)))
        else:
            vx, vy, wz = self.pilot.command(*self.pose)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Autopilot()
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
