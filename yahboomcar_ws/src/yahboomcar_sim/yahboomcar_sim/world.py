"""Mundo de ocupacion (sin dependencias de ROS): lee mapas estilo map_server."""
import math
import os
import re

import numpy as np
import yaml


def read_pgm(path):
    """Lee un PGM binario (P5) de 8 bits y devuelve un array (alto, ancho) uint8."""
    with open(path, 'rb') as f:
        data = f.read()
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', data)
    if m is None:
        raise ValueError(f'{path}: no es un PGM binario (P5)')
    w, h, maxval = (int(g) for g in m.groups())
    if maxval > 255:
        raise ValueError(f'{path}: solo se soportan PGM de 8 bits')
    pix = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=m.end())
    return pix.reshape(h, w)


def write_pgm(path, img):
    """Escribe un array (alto, ancho) uint8 como PGM binario (P5)."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(img.tobytes())


class WorldMap:
    """Mapa de ocupacion. Los indices (fila, col) crecen con y y x del mapa."""

    def __init__(self, yaml_path, unknown_as_wall=True):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = read_pgm(img_path)
        self.resolution = float(meta['resolution'])
        self.origin = [float(v) for v in meta['origin']]
        occ_th = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))

        p = img / 255.0 if negate else (255.0 - img) / 255.0   # prob. de ocupacion
        occupied = p >= occ_th
        free = (p < 0.1) & ~occupied
        unknown = ~occupied & ~free

        # PGM: fila 0 = arriba (y maximo). Se voltea para que fila 0 = y minimo.
        self.occupied = np.flipud(occupied)
        self.unknown = np.flipud(unknown)
        self.free = np.flipud(free)
        self.height, self.width = self.occupied.shape
        self.solid = self.occupied | self.unknown if unknown_as_wall else self.occupied.copy()

    @property
    def extent(self):
        """(xmin, xmax, ymin, ymax) del mapa en metros."""
        return (self.origin[0], self.origin[0] + self.width * self.resolution,
                self.origin[1], self.origin[1] + self.height * self.resolution)

    def cells(self, xs, ys):
        cols = np.floor((np.asarray(xs) - self.origin[0]) / self.resolution).astype(np.int64)
        rows = np.floor((np.asarray(ys) - self.origin[1]) / self.resolution).astype(np.int64)
        return rows, cols

    def is_solid(self, xs, ys):
        """True donde hay pared o esta fuera del mapa."""
        rows, cols = self.cells(xs, ys)
        inside = (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)
        out = np.ones(rows.shape, dtype=bool)
        out[inside] = self.solid[rows[inside], cols[inside]]
        return out

    def raycast(self, x, y, angles, t_values):
        """Distancia al primer obstaculo para cada angulo; inf si no hay ninguno.

        angles: (N,) rad en el marco del mapa; t_values: (T,) distancias a muestrear.
        """
        px = x + np.cos(angles)[:, None] * t_values[None, :]
        py = y + np.sin(angles)[:, None] * t_values[None, :]
        hit = self.is_solid(px, py)
        first = np.argmax(hit, axis=1)
        return np.where(hit.any(axis=1), t_values[first], np.inf)

    def auto_start(self, clearance):
        """Celda libre con holgura 'clearance' (m) mas cercana al centro de la zona libre."""
        r = int(math.ceil(clearance / self.resolution))
        solid = self.solid
        padded = np.pad(solid, r, constant_values=True)
        blocked = np.zeros_like(solid)
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc <= r * r:
                    blocked |= padded[r + dr:r + dr + self.height, r + dc:r + dc + self.width]
        ok = self.free & ~blocked
        rows, cols = np.nonzero(ok)
        if rows.size == 0:
            raise RuntimeError('El mapa no tiene zona libre donde quepa el robot')
        cr, cc = rows.mean(), cols.mean()
        k = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
        x = self.origin[0] + (cols[k] + 0.5) * self.resolution
        y = self.origin[1] + (rows[k] + 0.5) * self.resolution
        return float(x), float(y)
