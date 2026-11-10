"""Pistas de carrera (sin dependencias de ROS).

Una pista es un circuito cerrado definido por su linea central (spline Catmull-Rom
centripeta por puntos de control) y un ancho de carril. De ella se derivan:
  * la linea central remuestreada cada `ds` metros (rumbo, curvatura, longitud),
  * el mapa de ocupacion (carril libre + paredes delgadas),
  * la imagen del piso (para la camara simulada),
  * la proyeccion de una pose sobre la pista (progreso, desvio lateral).
"""
import math
import os

import numpy as np
import yaml

WALL_THICKNESS = 0.03     # m, grosor de la pared que bordea el carril
DS = 0.01                 # m, paso de remuestreo de la linea central


def wrap_pi(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def _catmull_rom_closed(points, per_segment=80, alpha=0.5):
    """Spline Catmull-Rom centripeta cerrada que pasa por todos los puntos."""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = pts[(i - 1) % n], pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), 1e-9) ** alpha
        t2 = t1 + max(np.linalg.norm(p2 - p1), 1e-9) ** alpha
        t3 = t2 + max(np.linalg.norm(p3 - p2), 1e-9) ** alpha
        t = np.linspace(t1, t2, per_segment, endpoint=False)[:, None]
        a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.vstack(out)


def _resample_closed(curve, ds):
    """Remuestrea una curva cerrada a paso uniforme `ds`. Devuelve (M,2) y el largo real."""
    closed = np.vstack([curve, curve[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    total = cum[-1]
    m = int(round(total / ds))
    s_new = np.linspace(0.0, total, m, endpoint=False)
    x = np.interp(s_new, cum, closed[:, 0])
    y = np.interp(s_new, cum, closed[:, 1])
    return np.column_stack([x, y]), total


def roughen_floor(floor, grain=0.05, seed=1):
    """Ensucia la imagen del piso: grano fino, manchas suaves y rayones (como una mesa real).

    grain = amplitud relativa (0 = piso limpio, 0.05 = leve, 0.15 = muy sucio).
    """
    if grain <= 0.0:
        return floor
    rng = np.random.default_rng(seed)
    h, w = floor.shape[:2]
    img = floor.astype(np.float32)
    img += rng.normal(0.0, 255.0 * grain, (h, w, 1))                 # grano fino
    # Manchas suaves: ruido de baja resolucion ampliado.
    gh, gw = max(2, h // 25), max(2, w // 25)
    blob = rng.normal(0.0, 255.0 * grain * 1.5, (gh, gw))
    blob = np.kron(blob, np.ones((-(-h // gh), -(-w // gw))))[:h, :w]
    img += blob[:, :, None]
    # Rayones finos aleatorios.
    for _ in range(int(0.05 * (h + w) * min(1.0, grain * 8))):
        x0, y0 = rng.integers(0, w), rng.integers(0, h)
        ln = int(rng.integers(5, 25))
        ang = rng.uniform(0, math.pi)
        for t in range(ln):
            xx, yy = int(x0 + t * math.cos(ang)), int(y0 + t * math.sin(ang))
            if 0 <= xx < w and 0 <= yy < h:
                img[yy, xx] += rng.choice([-1, 1]) * 255.0 * grain * 2.0
    return np.clip(img, 0, 255).astype(np.uint8)


class Track:
    def __init__(self, name, control_points, lane_width, level='', robot_radius=0.13,
                 box=(3.0, 2.0), resolution=0.01, n_checkpoints=8, description=''):
        self.name = name
        self.level = level
        self.description = description
        self.control_points = [list(map(float, p)) for p in control_points]
        self.lane_width = float(lane_width)
        self.robot_radius = float(robot_radius)
        self.box = (float(box[0]), float(box[1]))
        self.resolution = float(resolution)
        self.n_checkpoints = int(n_checkpoints)

        dense = _catmull_rom_closed(self.control_points)
        self.center, self.length = _resample_closed(dense, DS)
        self.n = len(self.center)
        self.ds = self.length / self.n
        self.s = np.arange(self.n) * self.ds

        # Rumbo y curvatura (suavizada en ~6 cm para quitar el ruido del remuestreo).
        d = np.roll(self.center, -1, axis=0) - np.roll(self.center, 1, axis=0)
        self.heading = np.arctan2(d[:, 1], d[:, 0])
        dth = wrap_pi(np.roll(self.heading, -1) - np.roll(self.heading, 1))
        kappa = dth / (2.0 * self.ds)
        k = max(1, int(round(0.06 / self.ds)))
        ker = np.ones(2 * k + 1) / (2 * k + 1)
        padded = np.concatenate([kappa[-k:], kappa, kappa[:k]])
        self.curvature = np.convolve(padded, ker, mode='valid')
        self.checkpoints = [int(i * self.n / self.n_checkpoints)
                            for i in range(self.n_checkpoints)]

    # ------------------------------------------------------------ propiedades
    @property
    def start_pose(self):
        return float(self.center[0, 0]), float(self.center[0, 1]), float(self.heading[0])

    @property
    def min_radius(self):
        return float(1.0 / max(np.max(np.abs(self.curvature)), 1e-9))

    def stats(self):
        """Resumen de las curvas (sirve para clasificar la dificultad)."""
        kap = self.curvature
        sign = np.sign(np.where(np.abs(kap) > 0.5, kap, 0.0))      # curvas con radio < 2 m
        nz = sign[sign != 0]
        turns = int(np.sum(np.abs(np.diff(np.concatenate([nz, nz[:1]]))) > 0)) if nz.size else 0
        # Una "curva" = tramo contiguo con |kappa| > 1/1.0 m
        tight = np.abs(kap) > 1.0
        starts = np.flatnonzero(tight & ~np.roll(tight, 1))
        return {
            'length_m': round(float(self.length), 2),
            'min_radius_m': round(self.min_radius, 3),
            'total_turning_deg': round(float(math.degrees(np.sum(np.abs(kap)) * self.ds)), 1),
            'curves': int(len(starts)),
            'left_curves': int(sum(1 for i in starts if kap[(i + 5) % self.n] > 0)),
            'right_curves': int(sum(1 for i in starts if kap[(i + 5) % self.n] < 0)),
            'direction_changes': turns,
        }

    # ------------------------------------------------------------ geometria
    def project(self, x, y, hint=None, window=80):
        """Punto de la linea central mas cercano. Devuelve (indice, desvio lateral).

        El desvio es positivo hacia la izquierda de la direccion de avance.
        Con `hint` solo se busca en +-window muestras (evita saltar a otro tramo
        que pase cerca).
        """
        if hint is None:
            idx = np.arange(self.n)
        else:
            idx = (hint + np.arange(-window, window + 1)) % self.n
        dx = self.center[idx, 0] - x
        dy = self.center[idx, 1] - y
        k = int(np.argmin(dx * dx + dy * dy))
        i = int(idx[k])
        th = self.heading[i]
        lateral = -math.sin(th) * (x - self.center[i, 0]) + math.cos(th) * (y - self.center[i, 1])
        return i, float(lateral)

    def point_ahead(self, i, dist):
        j = (i + int(round(dist / self.ds))) % self.n
        return float(self.center[j, 0]), float(self.center[j, 1])

    # ------------------------------------------------------------ validacion
    def validate(self, margin=0.02):
        """Lanza ValueError si la pista no cabe en la caja o se pisa a si misma."""
        half = self.lane_width / 2.0 + WALL_THICKNESS
        if self.min_radius < self.lane_width / 2.0:
            raise ValueError(
                f'{self.name}: radio minimo {self.min_radius:.3f} m < medio carril '
                f'{self.lane_width / 2:.3f} m (el borde interior se cruza)')
        lo = self.center.min(axis=0) - half
        hi = self.center.max(axis=0) + half
        if lo[0] < margin or lo[1] < margin or hi[0] > self.box[0] - margin \
                or hi[1] > self.box[1] - margin:
            raise ValueError(
                f'{self.name}: se sale de la caja {self.box}: '
                f'x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}]')
        # Tramos no vecinos deben quedar separados al menos por dos carriles + paredes.
        sub = self.center[::2]
        ssub = self.s[::2]
        dist = np.linalg.norm(sub[:, None, :] - sub[None, :, :], axis=2)
        arc = np.abs(ssub[:, None] - ssub[None, :])
        arc = np.minimum(arc, self.length - arc)
        far = arc > 2.5 * self.lane_width
        need = self.lane_width + 2 * WALL_THICKNESS + 0.03
        if far.any() and dist[far].min() < need:
            raise ValueError(
                f'{self.name}: dos tramos quedan a {dist[far].min():.3f} m '
                f'(minimo {need:.3f} m)')

    # ------------------------------------------------------------ rasterizado
    def _distance_to_center(self):
        """Distancia de cada celda al segmento mas cercano de la linea central."""
        res = self.resolution
        w = int(round(self.box[0] / res))
        h = int(round(self.box[1] / res))
        gx = (np.arange(w) + 0.5) * res
        gy = (np.arange(h) + 0.5) * res
        cx, cy = np.meshgrid(gx, gy)                    # (h, w), fila 0 = y minimo
        px, py = cx.ravel(), cy.ravel()
        best = np.full(px.shape, np.inf)
        best_i = np.zeros(px.shape, dtype=np.int64)
        c = self.center[::2]
        for a in range(0, len(px), 6000):
            ddx = px[a:a + 6000, None] - c[None, :, 0]
            ddy = py[a:a + 6000, None] - c[None, :, 1]
            d2 = ddx * ddx + ddy * ddy
            k = np.argmin(d2, axis=1)
            best[a:a + 6000] = np.sqrt(d2[np.arange(len(k)), k])
            best_i[a:a + 6000] = k * 2
        return best.reshape(h, w), best_i.reshape(h, w)

    def render_map(self):
        """Imagen PGM (fila 0 = arriba): negro=pared, blanco=carril, gris=fuera (no explorado)."""
        d, _ = self._distance_to_center()
        half = self.lane_width / 2.0
        img = np.full(d.shape, 205, dtype=np.uint8)
        img[d < half + WALL_THICKNESS] = 0
        img[d < half] = 254
        return np.flipud(img)

    def render_floor(self, line_color=(235, 200, 40)):
        """Imagen RGB del piso (alto, ancho, 3), fila 0 = y minimo (misma orientacion que WorldMap)."""
        d, idx = self._distance_to_center()
        half = self.lane_width / 2.0
        floor = np.empty(d.shape + (3,), dtype=np.uint8)
        floor[:] = (35, 35, 40)                              # fuera del carril
        lane = d < half
        floor[lane] = (78, 80, 86)                           # asfalto
        edge = lane & (d > half - 0.015)                     # lineas de borde
        floor[edge] = (235, 235, 235)
        # Linea central discontinua (trazos de 6 cm cada 12 cm).
        sarc = self.s[idx % self.n]
        dash = (np.mod(sarc, 0.12) < 0.06) & (d < 0.009)
        floor[dash & lane] = line_color
        # Linea de salida/meta: damero perpendicular a la pista en s=0.
        near = lane & ((idx < 4) | (idx > self.n - 4))
        res = self.resolution
        h, w = d.shape
        gx = (np.arange(w) + 0.5) * res
        gy = (np.arange(h) + 0.5) * res
        cx, cy = np.meshgrid(gx, gy)
        lat = np.hypot(cx - self.center[0, 0], cy - self.center[0, 1])
        checker = ((np.floor(lat / 0.04) + np.floor(sarc / 0.02)) % 2) == 0
        floor[near & checker] = (245, 245, 245)
        floor[near & ~checker] = (15, 15, 15)
        return floor

    # ------------------------------------------------------------ E/S
    def to_dict(self):
        return {
            'name': self.name,
            'level': self.level,
            'description': self.description,
            'lane_width': self.lane_width,
            'robot_radius': self.robot_radius,
            'box': list(self.box),
            'resolution': self.resolution,
            'checkpoints': self.n_checkpoints,
            'control_points': [[round(v, 4) for v in p] for p in self.control_points],
        }

    def save(self, path):
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# Pista de carreras (ver yahboomcar_sim/track.py). Generada por gen_tracks.\n')
            yaml.safe_dump(self.to_dict(), f, sort_keys=False, allow_unicode=True,
                           default_flow_style=None)

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            d = yaml.safe_load(f)
        return cls(d['name'], d['control_points'], d['lane_width'], level=d.get('level', ''),
                   robot_radius=d.get('robot_radius', 0.13), box=d.get('box', (3.0, 2.0)),
                   resolution=d.get('resolution', 0.01), n_checkpoints=d.get('checkpoints', 8),
                   description=d.get('description', ''))

    @classmethod
    def for_map(cls, map_yaml):
        """Carga `<mapa>.track.yaml` junto al mapa si existe; si no, devuelve None."""
        base = os.path.splitext(os.path.abspath(map_yaml))[0]
        path = base + '.track.yaml'
        return cls.load(path) if os.path.isfile(path) else None
