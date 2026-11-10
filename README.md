# microrospi5-sim · Simulador del robot Yahboom MicroROS-Pi5 + Reto RC-1 (control PID)

Simulador **2D** en ROS 2 Humble que reemplaza al robot físico (`/cmd_vel` → `/odom`, `/scan`, IMU, cámara), se ve en RViz y trae pistas de carrera, piso irregular, odometría con error y un piloto automático con PID. Sirve para practicar en casa lo que luego se hace en el laboratorio con el robot real.

Incluye además el **Reto RC-1 «Control PID: del simulador al robot real»** del curso de Robótica (ESAN, CapyTown).

## Contenido

| Carpeta / archivo | Qué es |
|---|---|
| `yahboomcar_ws/src/yahboomcar_sim/` | El simulador (nodo `sim_robot`, pistas, piloto PID, launch, RViz, pruebas) |
| `yahboomcar_ws/src/…` (resto) | Paquetes del workspace Yahboom (descripción del robot, bringup, nav, etc.) |
| `SIMULACION.md` | **Guía completa** del simulador: instalación, opciones, ruido, pistas, topics |
| `instalar_en_wsl.sh` | Instalador para Ubuntu 22.04 en WSL2 |
| `RC1_Control_PID_2026-2.docx` | Documento del reto RC-1 (reto + guía) |
| `docs/rc1/` | Herramientas del RC-1: `cierre_real.py`, `09_plot_csv.py` parcheado, figuras |
| `docs/pistas/` | Vistas previas de las pistas |
| `docker/`, `ros2_humble.sh`, `setup_workspace.sh`, `WORKSPACE_SETUP.md` | Entorno Docker del robot real (Raspberry Pi 5, arm64) |

## Instalación del simulador (Windows 11 + WSL2)

1. En PowerShell como administrador: `wsl --install -d Ubuntu-22.04` y reiniciar.
2. Copiar este repositorio **dentro** de Ubuntu (por ejemplo `~/microrospi5`); trabajar desde `/mnt/c/...` es mucho más lento.
   ```bash
   git clone https://github.com/codeplai/microrospi5-sim.git ~/microrospi5
   ```
3. Instalar y compilar (descarga ≈ 3 GB: ROS 2 Humble, RViz, Nav2, SLAM Toolbox, teleop):
   ```bash
   cd ~/microrospi5 && bash instalar_en_wsl.sh
   ```
4. Lanzar (WSLg muestra RViz directamente):
   ```bash
   ros2 launch yahboomcar_sim sim.launch.py world:=pista_intermedia laps:=3
   ros2 run teleop_twist_keyboard teleop_twist_keyboard      # otra terminal
   ```

Requisitos: Windows 11 (o Windows 10 22H2 con WSL actualizado). Más opciones (`noise:=`, `autopilot:=true`, `world:=`…) en [`SIMULACION.md`](SIMULACION.md).

Pruebas del núcleo sin ROS (Python): `cd yahboomcar_ws/src/yahboomcar_sim && python -m unittest discover -s test -v` (25 pruebas).

## Reto RC-1 · Control PID: del simulador al robot real

El grupo sintoniza un PID con el nodo `cuadrado_controller.py` de la clase y lo prueba en **tres plantas** con el mismo código: Python puro, este simulador y el robot real. Idea central: *un lazo cerrado es tan bueno como el sensor que lo cierra*.

- **Experimento 1** – corregir una línea virtual de 2 m (bang-bang, P, P+D, PID).
- **Experimento 2** – giro de 90° (efecto de Kp, Kd, Ki).
- **Experimento 3** – cuadrado de 2 × 2 m: lazo abierto, PID sobre `/odom` y PID + Ki.
- **Partes A–D** en el simulador, en la arena abierta `arena_rc1` (4 × 4 m); **Partes E–F** en el robot real, donde la verdad la mide la cinta métrica. La parte real concentra la mayor parte de la nota.

El simulador publica `/sim/pose` (la pose **real**): con `docs/rc1/cierre_real.py` se compara lo que *cree* el robot (`/odom`) con lo que *pasó*. Con piso irregular, el nodo puede decir que cierra a 0.3 cm mientras la pose real queda a decenas de cm.

```bash
ros2 launch yahboomcar_sim sim.launch.py world:=arena_rc1 start:=1.0,1.0,0.0 robot_radius:=0.14 noise:=low
python3 cuadrado_controller.py --ros-args -p experimento:=cuadrado -p log_csv:=exp3b_sim.csv   # nodo de la clase
python3 docs/rc1/cierre_real.py --csv exp3b_real.csv --png exp3b_real.png --lado 2.0          # antes del nodo
```

El nodo de la clase (`scripts_pdi/cuadrado_controller.py` y `core_cuadrado.py`) **no está en este repositorio**: es material del curso.

## Estado de la verificación

- **Probado:** 25 pruebas sin ROS; compilación y ejecución de `sim_robot` con ROS 2 Humble en WSL; el nodo de la clase corre sin cambios contra `arena_rc1` y reproduce las cifras de su simulador de Python (experimento 1, sin ruido).
- **No probado:** `sim.launch.py` completo con RViz sobre `arena_rc1`, Nav2 con el simulador, y todo lo del robot real.
- Con muchas pruebas en paralelo la CPU se satura y el PID da resultados falsos: correr una a la vez (o ≤ 5).
- El simulador **no** modela el derrape de las 4 ruedas ni el retardo de `/odom`.
