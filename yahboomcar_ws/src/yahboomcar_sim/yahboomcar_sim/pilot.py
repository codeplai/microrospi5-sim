"""Piloto automatico de referencia (pure pursuit sobre la linea central). Sin ROS."""
import math

import numpy as np

from .track import wrap_pi


class PurePursuit:
    def __init__(self, track, v_max=0.6, a_lat=0.8, lookahead=0.30, v_min=0.15, kp_lat=1.5):
        self.track = track
        self.v_max, self.a_lat, self.lookahead = v_max, a_lat, lookahead
        self.v_min, self.kp_lat = v_min, kp_lat
        self.idx = None

    def reset(self):
        self.idx = None

    def command(self, x, y, yaw):
        """Devuelve (vx, vy, wz) en el marco del robot."""
        tr = self.track
        self.idx, lateral = tr.project(x, y, hint=self.idx)
        # Velocidad segun la curva mas cerrada en los proximos 0.6 m (limite de aceleracion lateral).
        n_ahead = int(0.6 / tr.ds)
        ks = np.abs(tr.curvature[(self.idx + np.arange(n_ahead)) % tr.n])
        kmax = float(ks.max()) if ks.size else 0.0
        v = self.v_max if kmax < 1e-3 else min(self.v_max, math.sqrt(self.a_lat / kmax))
        v = max(self.v_min, v)
        # Punto objetivo adelante; la distancia de mira se acorta a baja velocidad.
        la = max(0.15, min(self.lookahead, 0.15 + 0.35 * v))
        tx, ty = tr.point_ahead(self.idx, la)
        alpha = float(wrap_pi(math.atan2(ty - y, tx - x) - yaw))
        wz = 2.0 * v * math.sin(alpha) / max(la, 1e-3)
        wz = max(-3.0, min(3.0, wz))
        # Robot holonomico: ademas corrige el desvio lateral con movimiento de lado.
        vy = max(-0.3, min(0.3, -self.kp_lat * lateral * 0.5))   # lateral>0: a la izquierda
        return v * math.cos(alpha), vy, wz


class PID:
    """PID discreto con derivada filtrada, limite de salida y anti-windup (integral acotada)."""

    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=1.0, i_limit=None, d_tau=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = out_limit if i_limit is None else i_limit
        self.d_tau = d_tau
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.d_filt = 0.0
        self.last = (0.0, 0.0, 0.0)           # (P, I, D) de la ultima salida

    def update(self, err, dt):
        dt = max(dt, 1e-3)
        p = self.kp * err
        d = 0.0
        if self.prev_err is not None:
            raw = (err - self.prev_err) / dt
            a = dt / (self.d_tau + dt)
            self.d_filt += a * (raw - self.d_filt)
            d = self.kd * self.d_filt
        self.prev_err = err
        unsat = p + self.ki * self.integral + d
        sat = max(-self.out_limit, min(self.out_limit, unsat))
        # Anti-windup: solo integra si no esta saturado o si el error ayuda a salir de la saturacion.
        if unsat == sat or (unsat > 0) != (err > 0):
            self.integral += err * dt
            lim = self.i_limit / self.ki if self.ki > 0 else 0.0
            self.integral = max(-lim, min(lim, self.integral))
        i = self.ki * self.integral
        self.last = (p, i, d)
        return max(-self.out_limit, min(self.out_limit, p + i + d))


class PIDFollower:
    """Sigue la linea central con dos PID (sin ROS).

    * PID lateral: desvio respecto de la linea central -> velocidad de lado (vy, el robot
      mecanum se puede mover de lado) en el marco de la pista.
    * PID de rumbo: angulo entre el robot y el rumbo de la pista unos cm adelante -> wz,
      mas un termino anticipado (feed-forward) = curvatura * velocidad.
    * La velocidad baja en curvas (limite de aceleracion lateral) y cuando el error crece.
    """

    def __init__(self, track, v_max=0.6, a_lat=0.8, v_min=0.15,
                 kp_lat=3.5, ki_lat=0.3, kd_lat=0.4,
                 kp_head=3.0, ki_head=0.0, kd_head=0.15, lookahead=0.12):
        self.track = track
        self.v_max, self.a_lat, self.v_min = v_max, a_lat, v_min
        self.lookahead = lookahead
        self.pid_lat = PID(kp_lat, ki_lat, kd_lat, out_limit=0.45)
        self.pid_head = PID(kp_head, ki_head, kd_head, out_limit=3.0)
        self.idx = None
        self.lateral = 0.0
        self.heading_err = 0.0
        self.speed_cmd = 0.0

    def reset(self):
        self.idx = None
        self.pid_lat.reset()
        self.pid_head.reset()

    def command(self, x, y, yaw, dt=0.02):
        """Devuelve (vx, vy, wz) en el marco del robot."""
        tr = self.track
        prev = self.idx
        self.idx, lateral = tr.project(x, y, hint=self.idx)
        if prev is not None and min((self.idx - prev) % tr.n, (prev - self.idx) % tr.n) > 100:
            self.pid_lat.reset()                 # salto grande (teletransporte): reinicia
            self.pid_head.reset()
        th = float(tr.heading[self.idx])
        # Rumbo de referencia unos cm adelante: anticipa las curvas.
        la = int(self.lookahead / tr.ds)
        th_ref = float(tr.heading[(self.idx + la) % tr.n])
        e_head = float(wrap_pi(th_ref - yaw))
        # Velocidad: limite por curvatura en los proximos 0.6 m y por el error actual.
        n_ahead = int(0.6 / tr.ds)
        kmax = float(np.abs(tr.curvature[(self.idx + np.arange(n_ahead)) % tr.n]).max())
        v = self.v_max if kmax < 1e-3 else min(self.v_max, math.sqrt(self.a_lat / kmax))
        v *= max(0.5, 1.0 - 3.0 * abs(lateral) - 0.3 * abs(e_head))
        v = max(self.v_min, v)

        vn = self.pid_lat.update(-lateral, dt)        # lateral>0: izquierda -> corregir a la derecha
        kappa = float(tr.curvature[self.idx])
        wz = self.pid_head.update(e_head, dt) + kappa * v
        wz = max(-3.0, min(3.0, wz))
        rel = float(wrap_pi(th - yaw))
        vx = v * math.cos(rel) - vn * math.sin(rel)
        vy = v * math.sin(rel) + vn * math.cos(rel)
        self.lateral, self.heading_err, self.speed_cmd = lateral, e_head, v
        return vx, vy, wz
