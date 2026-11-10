#!/usr/bin/env python3
"""RC-1 · cierre_real.py — la "cinta metrica virtual" del simulador.

Compara lo que dice /odom (lo que cree el robot) con la pose REAL del simulador (/sim/pose).
Solo funciona con yahboomcar_sim: en el robot real /sim/pose no existe, ahi mide la cinta.

Uso (terminal 3, ANTES de lanzar cuadrado_controller.py; Ctrl+C cuando el nodo diga FIN):
    python3 cierre_real.py --csv exp3b_real.csv --png exp3b_real.png --lado 2.0

Imprime el cierre (distancia inicio-final) y el yaw acumulado, segun /odom y segun la pose real.
--lado L dibuja ademas el cuadrado ideal de lado L (opcional). El PNG necesita matplotlib.
"""
import argparse
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Cinta(Node):
    def __init__(self, path):
        super().__init__('cierre_real')
        self.real = self.odom = None
        self.rows = []
        self.f = open(path, 'w', newline='') if path else None
        self.w = csv.writer(self.f) if self.f else None
        if self.w:
            self.w.writerow(['t', 'x_real', 'y_real', 'yaw_real', 'x_odom', 'y_odom', 'yaw_odom'])
        self.create_subscription(PoseStamped, '/sim/pose', self.on_real, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Registrando /sim/pose y /odom... (Ctrl+C para terminar)')

    def on_real(self, m):
        p = m.pose
        self.real = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def tick(self):
        if self.real is None or self.odom is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        row = (t, *self.real, *self.odom)
        self.rows.append(row)
        if self.w:
            self.w.writerow([f'{v:.4f}' for v in row])


def resumen(rows):
    """Cierre (cm) y yaw acumulado (grados), segun /odom y segun la pose real."""
    out = {}
    for name, (ix, iy, iyaw) in (('odom', (4, 5, 6)), ('real', (1, 2, 3))):
        x0, y0 = rows[0][ix], rows[0][iy]
        total, prev = 0.0, rows[0][iyaw]
        for r in rows:
            total += wrap(r[iyaw] - prev)
            prev = r[iyaw]
        out[name] = (100.0 * math.hypot(rows[-1][ix] - x0, rows[-1][iy] - y0), math.degrees(total))
    return out


def grafica(rows, png, lado):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def rel(ix, iy, iyaw):
        x0, y0, th0 = rows[0][ix], rows[0][iy], rows[0][iyaw]
        c, s = math.cos(th0), math.sin(th0)
        return ([c * (r[ix] - x0) + s * (r[iy] - y0) for r in rows],
                [-s * (r[ix] - x0) + c * (r[iy] - y0) for r in rows])

    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    if lado > 0:
        ax.plot([0, lado, lado, 0, 0], [0, 0, lado, lado, 0], '--', c='gray', lw=1.2, label='cuadrado ideal')
    ax.plot(*rel(4, 5, 6), c='#0277BD', lw=2, label='según /odom (lo que cree el robot)')
    ax.plot(*rel(1, 2, 3), c='#0B7A43', lw=2, label='REAL (/sim/pose)')
    ax.plot(0, 0, 'o', c='#DB4437', label='inicio')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Odometría vs. pose real')
    fig.tight_layout()
    fig.savefig(png, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default='', help='archivo CSV de salida')
    ap.add_argument('--png', default='', help='PNG con las dos trayectorias')
    ap.add_argument('--lado', type=float, default=0.0, help='dibuja el cuadrado ideal de este lado (m)')
    a = ap.parse_args()
    rclpy.init()
    node = Cinta(a.csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):    # Humble avisa el Ctrl+C asi
        pass
    rows = node.rows
    if node.f:
        node.f.close()
    node.destroy_node()
    if len(rows) < 2:
        sys.exit('No llegaron datos: ¿está corriendo el simulador (sim_robot)?')
    r = resumen(rows)
    print(f"muestras: {len(rows)}  ({rows[-1][0] - rows[0][0]:.1f} s)")
    print(f"cierre según /odom : {r['odom'][0]:6.1f} cm    yaw acumulado: {r['odom'][1]:7.1f} deg")
    print(f"cierre REAL        : {r['real'][0]:6.1f} cm    yaw acumulado: {r['real'][1]:7.1f} deg")
    if a.png:
        grafica(rows, a.png, a.lado)
        print('grafica            :', a.png)


if __name__ == '__main__':
    main()
