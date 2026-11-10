"""Cronometro de carrera sobre una pista (sin dependencias de ROS).

El progreso es la distancia neta recorrida a lo largo de la linea central (se desenrolla,
asi que ir de reversa lo resta y no se pueden contar vueltas en falso). Una vuelta se
completa al acumular el largo de la pista.
"""
import math

from .track import wrap_pi


class RaceTracker:
    def __init__(self, track, total_laps=0):
        self.track = track
        self.total_laps = int(total_laps)       # 0 = sin limite
        self.reset(None)

    def reset(self, pose, t=0.0):
        self.status = 'ready'                   # ready -> running -> finished
        self.lap = 0                            # vueltas completas
        self.progress = 0.0                     # m netos desde la salida
        self.lap_times = []
        self.best_lap = None
        self.lap_start_t = None
        self.race_start_t = None
        self.now = t
        self.collisions = 0
        self.wall_time = 0.0
        self.top_speed = 0.0
        self.wrong_way = False
        self.lateral = 0.0
        self.idx = 0
        self._was_colliding = False
        self._prev_idx = None
        if pose is not None:
            self.idx, self.lateral = self.track.project(pose[0], pose[1])
            self._prev_idx = self.idx

    # ------------------------------------------------------------------
    def update(self, x, y, yaw, t, speed, colliding):
        """Procesa una muestra. Devuelve la lista de eventos ('lap', 'finish', 'collision')."""
        events = []
        tr = self.track
        dt = max(0.0, t - self.now) if self.now is not None else 0.0
        self.now = t
        if self.status == 'finished':
            return events

        idx, lat = tr.project(x, y, hint=self._prev_idx if self._prev_idx is not None else None)
        if self._prev_idx is not None:
            step = (idx - self._prev_idx + tr.n // 2) % tr.n - tr.n // 2
            self.progress += step * tr.ds
        self._prev_idx = idx
        self.idx, self.lateral = idx, lat

        err = float(wrap_pi(yaw - tr.heading[idx]))
        self.wrong_way = abs(err) > math.radians(110) and speed > 0.05

        if self.status == 'ready' and speed > 0.03:
            self.status = 'running'
            self.race_start_t = self.lap_start_t = t
            self.progress = 0.0 if abs(self.progress) < 0.3 else self.progress
        if self.status != 'running':
            return events

        self.top_speed = max(self.top_speed, speed)
        if colliding:
            self.wall_time += dt
            if not self._was_colliding:
                self.collisions += 1
                events.append('collision')
        self._was_colliding = colliding

        while self.progress >= (self.lap + 1) * tr.length:
            self.lap += 1
            lt = t - self.lap_start_t
            self.lap_start_t = t
            self.lap_times.append(lt)
            if self.best_lap is None or lt < self.best_lap:
                self.best_lap = lt
            events.append('lap')
            if self.total_laps and self.lap >= self.total_laps:
                self.status = 'finished'
                events.append('finish')
                break
        return events

    # ------------------------------------------------------------------
    @property
    def lap_progress(self):
        """Fraccion (0..1) de la vuelta en curso."""
        L = self.track.length
        return max(0.0, min(1.0, (self.progress - self.lap * L) / L))

    @property
    def checkpoint(self):
        return int(self.lap_progress * self.track.n_checkpoints)

    @property
    def current_lap_time(self):
        if self.lap_start_t is None:
            return 0.0
        return self.now - self.lap_start_t

    @property
    def total_time(self):
        if self.race_start_t is None:
            return 0.0
        return self.now - self.race_start_t

    def as_dict(self):
        return {
            'track': self.track.name,
            'status': self.status,
            'lap': self.lap,
            'total_laps': self.total_laps,
            'lap_progress': round(self.lap_progress, 3),
            'checkpoint': self.checkpoint,
            'lap_time': round(self.current_lap_time, 2),
            'total_time': round(self.total_time, 2),
            'last_lap': round(self.lap_times[-1], 2) if self.lap_times else None,
            'best_lap': round(self.best_lap, 2) if self.best_lap is not None else None,
            'lap_times': [round(v, 2) for v in self.lap_times],
            'lateral_offset': round(self.lateral, 3),
            'wrong_way': bool(self.wrong_way),
            'collisions': self.collisions,
            'wall_time': round(self.wall_time, 2),
            'top_speed': round(self.top_speed, 2),
        }
