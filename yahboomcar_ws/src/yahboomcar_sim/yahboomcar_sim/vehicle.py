"""Cinematica del robot Yahboom X3 (mecanum, holonomico) sin dependencias de ROS."""
import math

import numpy as np


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def slew(current, target, max_delta):
    """Acerca `current` a `target` cambiando como maximo `max_delta`."""
    return current + clamp(target - current, -max_delta, max_delta)


class Vehicle:
    """Robot holonomico con limites de velocidad/aceleracion, colision y odometria ruidosa.

    Estado real (marco del mapa): x, y, yaw.  Velocidad real en el marco del robot:
    vx, vy, wz (ya descontando lo que bloquea una pared).  La odometria (ox, oy, oyaw)
    integra la velocidad real con un pequeno error multiplicativo, como las ruedas reales.
    """

    def __init__(self, world, x, y, yaw, radius=0.13, max_vx=1.0, max_vy=1.0, max_wz=3.0,
                 accel_lin=2.5, accel_ang=8.0, odom_noise=0.0, floor_noise=0.0, floor_tau=0.4,
                 seed=None):
        self.world = world
        self.radius = radius
        self.max_vx, self.max_vy, self.max_wz = max_vx, max_vy, max_wz
        self.accel_lin, self.accel_ang = accel_lin, accel_ang
        self.odom_noise = odom_noise
        self.floor_noise = floor_noise      # m/s rms del empuje aleatorio del piso
        self.floor_tau = floor_tau          # s, que tan rapido cambia ese empuje
        self.rng = np.random.default_rng(seed)
        ang = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
        self._ring = (radius * np.cos(ang), radius * np.sin(ang))
        self.reset(x, y, yaw)

    def reset(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.ox = self.oy = self.oyaw = 0.0
        self.vx = self.vy = self.wz = 0.0           # velocidad real
        self.cvx = self.cvy = self.cwz = 0.0        # velocidad ordenada tras limitar aceleracion
        self.dv = np.zeros(3)                        # perturbacion del piso (marco del robot)
        self.colliding = False
        self.distance = 0.0                          # metros recorridos

    # ------------------------------------------------------------ colisiones
    def collides(self, x, y):
        px = np.concatenate(([x], x + self._ring[0]))
        py = np.concatenate(([y], y + self._ring[1]))
        return bool(self.world.is_solid(px, py).any())

    # ------------------------------------------------------------ paso
    def step(self, cmd, dt):
        """Avanza dt segundos con cmd=(vx, vy, wz) en el marco del robot."""
        tx = clamp(cmd[0], -self.max_vx, self.max_vx)
        ty = clamp(cmd[1], -self.max_vy, self.max_vy)
        tw = clamp(cmd[2], -self.max_wz, self.max_wz)
        da = self.accel_lin * dt
        self.cvx = slew(self.cvx, tx, da)
        self.cvy = slew(self.cvy, ty, da)
        self.cwz = slew(self.cwz, tw, self.accel_ang * dt)

        # Piso irregular: empuje aleatorio (proceso Ornstein-Uhlenbeck) que las ruedas NO
        # miden, asi el robot se desvia de lo ordenado y hay que corregirlo con sensores.
        sx = sy = sw = 0.0
        if self.floor_noise > 0.0:
            a = dt / self.floor_tau
            self.dv += -self.dv * a + self.floor_noise * math.sqrt(2.0 * a)                 * self.rng.standard_normal(3)
            f = min(1.0, (math.hypot(self.cvx, self.cvy) + 0.3 * abs(self.cwz)) / 0.3)
            sx, sy, sw = self.dv[0] * f, self.dv[1] * f, self.dv[2] * f * 2.0

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bvx, bvy = self.cvx + sx, self.cvy + sy
        want_dx = (bvx * c - bvy * s) * dt
        want_dy = (bvx * s + bvy * c) * dt
        nx, ny = self.x + want_dx, self.y + want_dy
        nyaw = self.yaw + (self.cwz + sw) * dt

        # Colision con deslizamiento sobre la pared.
        if not self.collides(nx, ny):
            ax, ay = nx, ny
        elif not self.collides(nx, self.y):
            ax, ay = nx, self.y
        elif not self.collides(self.x, ny):
            ax, ay = self.x, ny
        else:
            ax, ay = self.x, self.y

        dxw, dyw = ax - self.x, ay - self.y
        want = math.hypot(want_dx, want_dy)
        got = math.hypot(dxw, dyw)
        self.colliding = want > 1e-4 and got < 0.6 * want
        if self.colliding:
            # Al chocar se pierde la velocidad que la pared impide.
            k = got / want if want > 0 else 0.0
            self.cvx *= k
            self.cvy *= k

        dxb = c * dxw + s * dyw
        dyb = -s * dxw + c * dyw
        dyaw = nyaw - self.yaw
        self.vx, self.vy, self.wz = dxb / dt, dyb / dt, dyaw / dt
        self.distance += got
        self.x, self.y, self.yaw = ax, ay, nyaw

        # Odometria: lo que miden las ruedas (sin el empuje del piso) con error multiplicativo.
        k = (got / want) if want > 1e-9 else 1.0
        odx, ody, ody_aw = dxb - sx * dt * k, dyb - sy * dt * k, dyaw - sw * dt
        if self.odom_noise > 0.0:
            fl = 1.0 + self.rng.normal(0.0, self.odom_noise, 3)
            odx, ody, ody_aw = odx * fl[0], ody * fl[1], ody_aw * fl[2]
        co, so = math.cos(self.oyaw), math.sin(self.oyaw)
        self.ox += odx * co - ody * so
        self.oy += odx * so + ody * co
        self.oyaw += ody_aw

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)
