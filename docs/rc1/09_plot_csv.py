#!/usr/bin/env python3
"""CapyTown scripts_pdi/09 — Grafica y metricas de una corrida real (CSV de cuadrado_controller).

COPIA CON PARCHE (RC-1). Original: scripts_pdi/09_plot_csv.py. Unico cambio (marcado "PARCHE"):
tras terminar, cuadrado_controller.py sigue escribiendo filas en estado FIN con e_line = 0. Eso hacia
que, en la recta, "error final" saliera siempre 0.00 y que "t asentamiento" y "|e| medio" quedaran
mal. Ahora se descartan esas filas antes de calcular las metricas de la recta (en el giro y el
cuadrado no cambian nada: ahi las filas FIN repiten la pose final).

El nodo guarda un CSV con  -p log_csv:=exp1b.csv  (columnas:
t, x, y, yaw, v, w, e_line, e_yaw, estado). Este script no necesita ROS.

Uso:
  python3 09_plot_csv.py exp1b.csv --experimento recta
  python3 09_plot_csv.py exp2.csv  --experimento giro90
  python3 09_plot_csv.py exp3b.csv --experimento cuadrado --lado 2.0

Imprime las mismas metricas que 08_cuadrado_sim.py y guarda <csv>.png.
Recuerda: el error de cierre sale de /odom; confirmalo con cinta metrica.
"""
import argparse
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from core_cuadrado import Params, metricas, normalize


def leer(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: el CSV esta vacio")
    col = lambda k: np.array([float(r[k]) for r in rows])
    d = {k: col(k) for k in ("t", "x", "y", "yaw", "v", "w", "e_line", "e_yaw")}
    d["estado"] = np.array([r["estado"] for r in rows])
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--experimento", choices=["recta", "giro90", "cuadrado", "cuadrado_abierto"], default="cuadrado")
    ap.add_argument("--lado", type=float, default=2.0)
    a = ap.parse_args()

    d = leer(a.csv)
    # PARCHE: descartar las filas FIN (e_line = 0) en la recta; "terminado" se decide antes.
    fin_rows = np.nonzero(d["estado"] == "FIN")[0]
    terminado = bool(fin_rows.size)
    if terminado and a.experimento == "recta" and fin_rows[0] > 5:
        k = int(fin_rows[0])
        d = {key: v[:k] for key, v in d.items()}
    # pose relativa al inicio, en el marco del primer rumbo (x hacia adelante)
    x0, y0, th0 = d["x"][0], d["y"][0], d["yaw"][0]
    dx, dy = d["x"] - x0, d["y"] - y0
    d["x"] = dx * math.cos(th0) + dy * math.sin(th0)
    d["y"] = -dx * math.sin(th0) + dy * math.cos(th0)
    yaw_rel = np.array([normalize(v - th0) for v in d["yaw"]])
    d["yaw"] = np.unwrap(yaw_rel)
    d["closure"] = float(math.hypot(d["x"][-1], d["y"][-1]))
    d["yaw_total_deg"] = float(math.degrees(d["yaw"][-1] - d["yaw"][0]))
    d["terminado"] = terminado

    p = Params(experimento=a.experimento, side=a.lado)
    m = metricas(d, p)
    print(f"archivo     : {a.csv}  ({len(d['t'])} muestras, {d['t'][-1]:.1f} s)")
    print("terminado   :", "si" if d["terminado"] else "NO (el CSV termina antes de FIN)")
    for k, v in m.items():
        print(f"{k:26s}: " + ("no se asienta" if v is None else f"{float(v):.2f}"))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6))
    ax1.plot(d["x"], d["y"], c="#0277BD", lw=2, label="robot (odom)")
    if a.experimento in ("cuadrado", "cuadrado_abierto"):
        L = a.lado
        ax1.plot([0, L, L, 0, 0], [0, 0, L, L, 0], "--", c="gray", lw=1.2, label="cuadrado ideal")
    ax1.plot(0, 0, "o", c="#0F9D58", label="inicio")
    ax1.set_aspect("equal"); ax1.grid(alpha=0.3); ax1.legend(fontsize=8)
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_title("Trayectoria")
    if a.experimento == "giro90":
        ax2.plot(d["t"], np.degrees(d["yaw"]), c="#0277BD", lw=2)
        ax2.axhline(90, ls="--", c="gray"); ax2.axhspan(88, 92, color="green", alpha=0.08)
        ax2.set_ylabel("yaw (deg)"); ax2.set_title("Giro de 90 deg")
    else:
        ax2.plot(d["t"], d["e_line"] * 100, c="#C49000", lw=1.5)
        ax2.axhline(0, ls="--", c="gray"); ax2.axhspan(-2, 2, color="green", alpha=0.08)
        ax2.set_ylabel("error a la linea virtual (cm)"); ax2.set_title("Error lateral")
    ax2.set_xlabel("tiempo (s)"); ax2.grid(alpha=0.3)
    fig.suptitle(f"{a.csv} - {a.experimento} - corrida real", fontsize=11)
    fig.tight_layout()
    out = a.csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=130)
    print("grafica     :", out)


if __name__ == "__main__":
    main()
