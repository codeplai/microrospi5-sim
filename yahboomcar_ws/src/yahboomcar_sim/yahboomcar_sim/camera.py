"""Camara frontal simulada (sin dependencias de ROS).

Renderiza una vista en perspectiva: piso proyectado (asfalto, lineas, damero de meta) y
paredes por raycast. Permite probar seguimiento de linea con la misma imagen que veria
una camara real mirando hacia adelante y un poco hacia abajo.
"""
import math

import numpy as np

FLOOR_FREE = (78, 80, 86)
FLOOR_BLOCKED = (35, 35, 40)
SKY = (120, 150, 190)
WALL = (205, 60, 55)


def floor_from_world(world):
    """Piso plano para mundos sin pista: libre = asfalto, lo demas = oscuro."""
    floor = np.empty((world.height, world.width, 3), dtype=np.uint8)
    floor[:] = FLOOR_BLOCKED
    floor[world.free] = FLOOR_FREE
    return floor


class Camera:
    def __init__(self, world, floor, width=160, height=120, hfov=1.2, cam_x=0.10, cam_h=0.12,
                 pitch=0.35, wall_h=0.15, max_range=4.0):
        self.world, self.floor = world, floor
        self.width, self.height = width, height
        self.cam_x, self.cam_h, self.wall_h = cam_x, cam_h, wall_h
        f = (width / 2.0) / math.tan(hfov / 2.0)
        u = (np.arange(width) + 0.5 - width / 2.0) / f             # derecha +
        v = (np.arange(height) + 0.5 - height / 2.0) / f           # abajo +
        a = u[None, :]
        b = v[:, None]
        sp, cp = math.sin(pitch), math.cos(pitch)
        fwd = cp - b * sp                                          # comp. horizontal hacia adelante
        up = -(sp + b * cp)                                        # comp. vertical (arriba +)
        self._up = up
        self._hn = np.sqrt(fwd ** 2 + a ** 2)                      # (H, W) norma horizontal
        sky_row = up >= 0.0                                        # (H, 1) rayos sobre el horizonte
        t_g = np.where(sky_row, 0.0, cam_h / np.where(sky_row, 1.0, -up))   # (H, 1) finito
        self._ground_dist = np.where(sky_row, np.inf, t_g * self._hn)       # dist. horizontal al piso
        self._g_fwd = np.broadcast_to(t_g * fwd, self._hn.shape)
        self._g_left = -t_g * a                                    # (H, W)
        self._col_az = np.arctan2(-u, cp)                          # azimut por columna (izq +)
        self._t_vals = np.arange(0.02, max_range, world.resolution)

    def render(self, x, y, yaw):
        w = self.world
        cx = x + math.cos(yaw) * self.cam_x
        cy = y + math.sin(yaw) * self.cam_x
        dist = w.raycast(cx, cy, yaw + self._col_az, self._t_vals)  # (W,)
        d = dist[None, :]
        up = self._up
        with np.errstate(invalid='ignore'):
            t_wall = d / self._hn
            z_wall = up * t_wall                                    # altura (rel. camara) en la pared
        is_ground = (up < 0.0) & (self._ground_dist < d)
        is_wall = ~is_ground & np.isfinite(d) & (z_wall <= self.wall_h - self.cam_h)

        img = np.empty((self.height, self.width, 3), dtype=np.uint8)
        img[:] = SKY

        # Piso: punto del suelo en el marco del mapa -> color de la imagen del piso.
        c, s = math.cos(yaw), math.sin(yaw)
        gf = self._g_fwd + self.cam_x
        gx = x + c * gf - s * self._g_left
        gy = y + s * gf + c * self._g_left
        rows, cols = w.cells(gx, gy)
        inside = (rows >= 0) & (rows < w.height) & (cols >= 0) & (cols < w.width) & is_ground
        floor_px = np.empty_like(img)
        floor_px[:] = FLOOR_BLOCKED
        floor_px[inside] = self.floor[rows[inside], cols[inside]]
        img[is_ground] = floor_px[is_ground]

        # Paredes: sombreado segun distancia.
        shade = np.clip(1.3 / (0.7 + np.where(np.isfinite(dist), dist, 4.0)), 0.3, 1.0)
        wall_rgb = (np.array(WALL)[None, None, :] * shade[None, :, None]).astype(np.uint8)
        wall_rgb = np.broadcast_to(wall_rgb, img.shape)
        img[is_wall] = wall_rgb[is_wall]
        return img
