"""Genera las pistas de carrera de 3 x 2 m (basica, intermedia, avanzada).

  ros2 run yahboomcar_sim gen_tracks [--out DIR] [--preview]

Cada pista se define con vertices y un radio de curva por vertice (poligono con
esquinas redondeadas), asi el radio de cada curva es exacto. La dificultad sale de:
  * radio minimo de curva (mas pequeno = mas dificil),
  * cantidad de curvas y cambios de sentido (chicanes / eses),
  * ancho del carril (mas angosto = menos margen).
"""
import argparse
import math
import os

import numpy as np

from .track import Track
from .world import write_pgm

BOX = (3.0, 2.0)           # m (largo x ancho de la mesa)
RESOLUTION = 0.01          # m/celda -> mapa de 300 x 200 celdas


def rounded_polygon(vertices, radii, step=0.04):
    """Polilinea cerrada con esquinas redondeadas. radii[i]=0 deja el vertice recto."""
    v = np.asarray(vertices, dtype=float)
    n = len(v)
    pts = []
    tang = []
    for i in range(n):
        p_prev, p, p_next = v[i - 1], v[i], v[(i + 1) % n]
        u_in = (p - p_prev) / np.linalg.norm(p - p_prev)
        u_out = (p_next - p) / np.linalg.norm(p_next - p)
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        theta = math.atan2(cross, float(np.dot(u_in, u_out)))        # giro con signo
        r = radii[i]
        if r <= 0.0 or abs(theta) < 1e-6:
            tang.append(0.0)
            continue
        tang.append(r * math.tan(abs(theta) / 2.0))
    for i in range(n):
        seg = np.linalg.norm(v[(i + 1) % n] - v[i])
        if tang[i] + tang[(i + 1) % n] > seg + 1e-9:
            raise ValueError(f'vertices {i}-{(i + 1) % n}: las curvas no caben en el tramo '
                             f'({tang[i] + tang[(i + 1) % n]:.3f} > {seg:.3f} m)')

    for i in range(n):
        p_prev, p, p_next = v[i - 1], v[i], v[(i + 1) % n]
        u_in = (p - p_prev) / np.linalg.norm(p - p_prev)
        u_out = (p_next - p) / np.linalg.norm(p_next - p)
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        theta = math.atan2(cross, float(np.dot(u_in, u_out)))
        if tang[i] == 0.0:
            a = b = p
            arc = np.array([p])
        else:
            r = radii[i]
            a = p - u_in * tang[i]
            normal = np.array([-u_in[1], u_in[0]]) * (1.0 if theta > 0 else -1.0)
            c = a + normal * r
            m = max(3, int(math.ceil(abs(theta) * r / step)))
            phis = np.linspace(0.0, theta, m + 1)
            rel = a - c
            ca, sa = np.cos(phis), np.sin(phis)
            arc = c + np.column_stack([rel[0] * ca - rel[1] * sa, rel[0] * sa + rel[1] * ca])
            b = arc[-1]
        # Recta desde el final de la curva anterior (ya agregado) hasta el inicio de esta.
        if pts:
            last = pts[-1]
            first = arc[0]
            k = int(np.linalg.norm(first - last) / step)
            for j in range(1, k):
                pts.append(last + (first - last) * j / k)
        pts.extend(list(arc))
    # Recta de cierre hacia el primer punto.
    last, first = pts[-1], pts[0]
    k = int(np.linalg.norm(first - last) / step)
    for j in range(1, k):
        pts.append(last + (first - last) * j / k)
    out = np.array(pts)
    # Quitar duplicados consecutivos
    keep = np.concatenate(([True], np.linalg.norm(np.diff(out, axis=0), axis=1) > 1e-4))
    return out[keep]


# ---------------------------------------------------------------------------
# Definicion de las tres pistas. Marco del mapa: x a lo largo (0..3), y a lo ancho (0..2).
# El primer vertice esta en un tramo recto: ahi esta la salida/meta, mirando a +x.
# ---------------------------------------------------------------------------
def _basica():
    # Ovalo-rectangulo con 4 curvas amplias de 90 grados, todas hacia la izquierda.
    v = [(1.5, 0.38), (2.62, 0.38), (2.62, 1.62), (0.38, 1.62), (0.38, 0.38)]
    r = [0.0, 0.58, 0.58, 0.58, 0.58]
    return dict(
        name='pista_basica', level='basica', lane_width=0.50, vertices=v, radii=r,
        description='Circuito oval con 4 curvas amplias a la izquierda (radio 0.58 m). '
                    'Ideal para empezar: control de velocidad y centrado.')


def _intermedia():
    # Curvas de 90 y 45 grados, un chicane (izquierda-derecha) arriba y un cambio de
    # sentido al final del recto trasero.
    v = [(1.3, 0.37),
         (2.62, 0.37),          # curva izquierda 90
         (2.62, 1.05),          # curva izquierda ~48 (diagonal)
         (2.05, 1.63),          # curva izquierda ~48
         (1.35, 1.63),          # chicane: baja a la izquierda
         (0.98, 1.22),          # chicane: vuelve a la izquierda
         (0.38, 1.22),          # curva izquierda 90
         (0.38, 0.37)]          # curva izquierda 90
    r = [0.0, 0.40, 0.34, 0.34, 0.30, 0.30, 0.36, 0.40]
    return dict(
        name='pista_intermedia', level='intermedia', lane_width=0.45, vertices=v, radii=r,
        description='Circuito con curvas de 90 y 45 grados, diagonal y un chicane '
                    '(izquierda-derecha). Radio minimo 0.30 m.')


def _avanzada():
    # Horquilla de 180 grados, S cerrada, curva derecha 90 y curvas izquierdas de 90 con
    # radio pequeno. Carril angosto.
    v = [(0.95, 0.37),
         (2.62, 0.37),          # horquilla (parte 1)
         (2.62, 0.95),          # horquilla (parte 2)
         (1.95, 0.95),          # S: sube a la derecha
         (1.65, 1.20),          # S: vuelve a la izquierda
         (1.25, 1.20),          # curva DERECHA 90 (cambia de sentido)
         (1.25, 1.66),          # curva izquierda 90
         (0.40, 1.66),          # curva izquierda 90
         (0.40, 1.22),          # S en el recto de la izquierda
         (0.64, 0.95),
         (0.40, 0.68),
         (0.40, 0.37)]          # curva izquierda 90
    r = [0.0, 0.27, 0.27, 0.24, 0.24, 0.22, 0.22, 0.27, 0.22, 0.22, 0.22, 0.22]
    return dict(
        name='pista_avanzada', level='avanzada', lane_width=0.40, vertices=v, radii=r,
        description='Circuito tecnico: horquilla de 180 grados, eses, cambio de sentido '
                    'y curvas cerradas. Radio minimo 0.22 m con carril de 0.40 m.')


def _cuadrado():
    # Cuadrado de 2 x 2 m con 4 esquinas de 90 grados a la izquierda (mesa de 2 x 2 m).
    v = [(1.0, 0.30), (1.70, 0.30), (1.70, 1.70), (0.30, 1.70), (0.30, 0.30)]
    r = [0.0, 0.25, 0.25, 0.25, 0.25]
    return dict(
        name='pista_cuadrado', level='cuadrado', lane_width=0.45, vertices=v, radii=r,
        box=(2.0, 2.0),
        description='Cuadrado de 2 x 2 m con 4 esquinas de 90 grados (radio 0.25 m). '
                    'Sirve para calibrar giros y comprobar que el robot cierra el cuadrado.')


DEFINITIONS = [_basica, _intermedia, _avanzada, _cuadrado]


def build_tracks():
    tracks = []
    for fn in DEFINITIONS:
        d = fn()
        radii = [abs(x) for x in d['radii']]
        pts = rounded_polygon(d['vertices'], radii)
        t = Track(d['name'], pts[::1], d['lane_width'], level=d['level'],
                  box=d.get('box', BOX),
                  resolution=RESOLUTION, description=d['description'])
        t.validate()
        tracks.append(t)
    return tracks


def write_world(track, out_dir):
    """Escribe <name>.pgm/.yaml (mapa) y <name>.track.yaml (pista) en out_dir."""
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, track.name)
    write_pgm(base + '.pgm', track.render_map())
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {track.name}.pgm\n')
        f.write('mode: trinary\n')
        f.write(f'resolution: {track.resolution}\n')
        f.write('origin: [0.0, 0.0, 0]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    track.save(base + '.track.yaml')
    return base


ARENA_NAME = 'arena_rc1'
ARENA_SIZE = (4.0, 4.0)    # m: el cuadrado de 2 x 2 m del RC-1 queda con 1 m de margen por lado
ARENA_WALL = 0.03          # m de grosor, solo en el borde


def write_arena(out_dir, size=ARENA_SIZE, wall=ARENA_WALL, name=ARENA_NAME):
    """Arena abierta (sin carril, sin carrera): paredes solo en el borde. Para el RC-1."""
    os.makedirs(out_dir, exist_ok=True)
    cols, rows = (int(round(s / RESOLUTION)) for s in size)
    k = max(1, int(round(wall / RESOLUTION)))
    img = np.full((rows, cols), 255, dtype=np.uint8)          # 255 = libre
    img[:k, :] = img[-k:, :] = 0
    img[:, :k] = img[:, -k:] = 0                               # 0 = pared
    base = os.path.join(out_dir, name)
    write_pgm(base + '.pgm', img)
    with open(base + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n')
        f.write('origin: [0.0, 0.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return base


def preview(track, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    floor = track.render_floor()
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.imshow(floor, origin='lower', extent=(0, track.box[0], 0, track.box[1]))
    cp = track.center
    ax.plot(cp[:, 0], cp[:, 1], 'c-', lw=0.5)
    sx, sy, syaw = track.start_pose
    ax.arrow(sx, sy, 0.2 * math.cos(syaw), 0.2 * math.sin(syaw), color='lime', width=0.02)
    s = track.stats()
    ax.set_title(f"{track.name} - carril {track.lane_width:.2f} m - largo {s['length_m']} m - "
                 f"R min {s['min_radius_m']} m - {s['curves']} curvas")
    ax.set_aspect('equal')
    fig.savefig(path, dpi=110, bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Genera las pistas de 3x2 m')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'worlds'), help='carpeta de salida')
    ap.add_argument('--preview', action='store_true', help='guardar PNG de cada pista')
    args = ap.parse_args(argv)
    for t in build_tracks():
        base = write_world(t, args.out)
        print(f'{t.name}: {t.stats()}  -> {base}.*')
        if args.preview:
            preview(t, base + '_preview.png')
    print(f'{ARENA_NAME}: {ARENA_SIZE[0]} x {ARENA_SIZE[1]} m abierta  -> '
          f'{write_arena(args.out)}.*')


if __name__ == '__main__':
    main()
