# Robot Yahboom MicroROS-Pi5 virtual (sin robot físico)

Paquete `yahboomcar_sim`: un simulador **2D** que reemplaza al hardware, se ve en RViz y trae **tres pistas de carreras de 3 × 2 m** (básica, intermedia y avanzada) más una **pista cuadrada de 2 × 2 m**, con cronómetro de vueltas, piso irregular y realineado de la odometría.

| Entra | Sale |
|---|---|
| `/cmd_vel` (vx, vy, wz: el robot admite desplazamiento lateral) | `/odom` + TF `odom → base_footprint`, `/scan` (lidar MS200: 360°, 0.03–12 m, 450 puntos, 10 Hz), `/imu/data_raw`, `/vel_raw`, `/voltage`, `/camera/image_raw` + `/camera/camera_info`, `/sim/pose` (pose real), `/sim/collision`, `/race/*` |

**Qué simula:** cinemática mecanum con límites de velocidad y aceleración, colisión y deslizamiento sobre las paredes, odometría con error (1 % por defecto), lidar con ruido, IMU, batería, cámara frontal con piso y paredes en perspectiva, y la carrera (vueltas, tiempos, choques).

**Qué NO simula:** física real (inercia, derrape, par de los motores) ni el brazo. `yahboomcar_astra`, `yahboomcar_visual` y `yahboomcar_mediapipe` necesitan la cámara real (Astra/USB); la cámara simulada es una vista sencilla, sirve para seguir línea y evitar paredes pero no para reconocimiento de objetos.

## 1. Instalación en Windows (una sola vez)

Requisito: Windows 11 (o Windows 10 22H2 con WSL actualizado), que muestra RViz directamente con WSLg.

1. En PowerShell como administrador: `wsl --install -d Ubuntu-22.04`, y reiniciar.
2. Copiar esta carpeta del proyecto **dentro** del sistema de Ubuntu (por ejemplo `~/microrospi5`). Trabajar desde `/mnt/c/...` es mucho más lento.
3. En Ubuntu: `cd ~/microrospi5 && bash instalar_en_wsl.sh`. Descarga unos 3 GB. Instala, entre otros, `joint_state_publisher`, que el launch necesita para el modelo del robot.

## 2. Las pistas

Tres circuitos cerrados dentro de una mesa de **3 × 2 m**, con paredes de 3 cm de grosor a ambos lados del carril. Salida/meta con damero, línea central discontinua y líneas de borde (se ven en la cámara). La dificultad sale de las curvas:

| Pista | Carril | Largo | Radio mín. | Curvas | Qué tiene | Vuelta de referencia* |
|---|---|---|---|---|---|---|
| `pista_basica` | 0.50 m | 5.96 m | 0.57 m | 4 | Óvalo con 4 curvas amplias de 90° a la izquierda | ≈ 10 s |
| `pista_intermedia` | 0.45 m | 5.88 m | 0.30 m | 7 | Curvas de 90° y 45°, diagonal y un chicane (izquierda-derecha) | ≈ 11 s |
| `pista_avanzada` | 0.40 m | 6.31 m | 0.22 m | 8 | Horquilla de 180°, eses, curva a la derecha y curvas cerradas | ≈ 14 s |

| `pista_cuadrado` (mesa de **2 × 2 m**) | 0.45 m | 5.17 m | 0.25 m | 4 | Cuadrado con 4 esquinas de 90° a la izquierda; sirve para calibrar giros | ≈ 11 s |

\* Piloto automático a 0.6 m/s, sin choques. Vistas previas en `docs/pistas/`.

El robot en las pistas usa radio 0.13 m (el carril más angosto deja 7 cm de holgura por lado).

Para cambiar o crear pistas edita las definiciones en `yahboomcar_sim/track_gen.py` (vértices + radio de cada curva) y regenera:
```bash
ros2 run yahboomcar_sim gen_tracks --preview
```
El generador valida que la pista quepa en 3 × 2 m, que el radio sea mayor que medio carril y que ningún tramo se cruce con otro.

## 3. Uso

Terminal 1: robot + RViz.
```bash
ros2 launch yahboomcar_sim sim.launch.py world:=pista_intermedia laps:=3
```
Terminal 2: manejarlo con el teclado (con Shift se mueve de lado).
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
El tablero en el centro de la pista muestra la vuelta, el tiempo, la última y mejor vuelta y los choques. El cronómetro arranca al primer movimiento. En RViz, **2D Pose Estimate** teletransporta el robot (y reinicia la carrera).

Opciones del launch:

| Argumento | Valor por defecto | Para qué |
|---|---|---|
| `world:=pista_avanzada` | `pista_basica` | Elegir el mundo: `pista_basica`, `pista_intermedia`, `pista_avanzada`, `pista_cuadrado`, `arena_rc1` (arena abierta de 4 × 4 m, sin carril, para el RC-1), `yahboom_map`, `yahboomcar` |
| `map_yaml:=/ruta/mi_mapa.yaml` | | Usar un mapa propio |
| `start:=1.0,0.5,1.57` | `auto` | Pose inicial `x,y,yaw` en el mapa |
| `laps:=3` | `0` | Vueltas de la carrera (0 = sin límite) |
| `autopilot:=true` | `false` | Piloto automático: recorre la pista solo, corrigiendo con PID |
| `controller:=pursuit` | `pid` | `pid`: dos PID (desvío lateral y rumbo); `pursuit`: pure pursuit de referencia |
| `pose_source:=tf` | `truth` | Pose que usa el piloto: `truth` = `/sim/pose` (real); `tf` = `map→base_footprint` (la que ve el robot, con `odom_align`) |
| `speed:=0.8` | `0.6` | Velocidad máxima del piloto automático (m/s) |
| `camera:=false` | `true` | Desactivar la cámara simulada |
| `odom_noise:=0.0` | `0.01` | Error relativo de la odometría (0 = perfecta) |
| `noise:=high` | `medium` | **Escenario de ruido para cualquier mundo** (pistas o mapas propios). `none`: todo perfecto; `low`; `medium`; `high`; `custom`: usa `floor_noise`, `floor_grain` y `odom_noise` de abajo. Cada nivel fija piso, cámara, odometría, lidar e IMU a la vez (ver tabla de niveles) |
| `floor_noise:=0.05` | `0.04` (solo con `noise:=custom`) | **Piso irregular**: empuje aleatorio (m/s) que desvía al robot y que las ruedas no miden; hay que corregirlo con lidar/cámara. 0 = piso perfecto |
| `floor_grain:=0.08` | `0.04` (solo con `noise:=custom`) | **Piso sucio** en la cámara (grano, manchas y rayones). 0 = limpio, 0.15 = muy sucio |
| `odom_align:=0.3` | `0.3` (launch) | Realineado tipo localizador: `map→odom` acerca la odometría a la pose real a razón de este valor (1/s) mientras el robot avanza. `/odom` sigue acumulando error; 0 = sin realinear (solo `standalone:=true`) |
| `robot_radius:=0.15` | automático | Radio del robot (m) |
| `accel_lin:=2.5` | `2.5` | Límite de aceleración lineal (m/s²) |
| `accel_ang:=8.0` | `8.0` | Límite de aceleración angular (rad/s²). Con poca carga de CPU casi no cambia el giro de 90° (con Kp = 10 y Kd = 0 sobrepasa ≈ 5 % con 8 y 0 % con 20) |
| `standalone:=false` | `true` | Con Nav2 o SLAM: ellos publican `/map` y `map → odom` |
| `rviz:=false` | `true` | Sin ventana de RViz |

Más parámetros del nodo (`--ros-args -p ...` o en el launch): `max_vx`, `max_vy`, `max_wz`, `accel_lin`, `accel_ang`, `scan_noise`, `imu_noise`, `battery_voltage`, `camera_rate`, `camera_width`, `camera_height`.

Niveles de `noise` (valen para todas las pistas y mapas; `odom_align` es independiente):

| Nivel | Empuje del piso (m/s) | Suciedad cámara | Error odometría | Ruido lidar (m) | Ruido IMU |
|---|---|---|---|---|---|
| `none` | 0 | 0 | 0 % | 0 | 0 |
| `low` | 0.02 | 0.02 | 1 % | 0.005 | 0.005 |
| `medium` (por defecto) | 0.04 | 0.04 | 2 % | 0.01 | 0.01 |
| `high` | 0.10 | 0.10 | 5 % | 0.02 | 0.03 |

Con `ros2 run` el parámetro es `-p noise_level:=high`. Usa `none`, no `off`: ROS convierte `off` en booleano.

### Piloto automático con PID

```bash
ros2 launch yahboomcar_sim sim.launch.py world:=pista_avanzada laps:=3 autopilot:=true noise:=high
```
- **PID lateral** (`kp_lat` 3.5, `ki_lat` 0.3, `kd_lat` 0.4): lleva el desvío respecto de la línea central a cero con movimiento de lado (el robot mecanum puede hacerlo).
- **PID de rumbo** (`kp_head` 3.0, `ki_head` 0, `kd_head` 0.15): alinea el robot con el rumbo de la pista unos 12 cm adelante, más un término anticipado (curvatura × velocidad).
- La velocidad baja en curvas (`lateral_accel`) y cuando crece el error. El PID tiene derivada filtrada y anti-windup.
- Cambia las ganancias en vivo: `ros2 param set /yahboomcar_autopilot kp_lat 5.0`.
- Publica `/pid/cross_track` (m) y `/pid/heading_error` (rad): `ros2 run rqt_plot rqt_plot /pid/cross_track /pid/heading_error`.

Resultado con ruido `high` (empuje del piso 0.10 m/s, odometría 5 %), 2 vueltas, sin ROS: las 4 pistas terminan, desvío medio 1.5–2 cm, sin choques con las ganancias por defecto (en la avanzada, el margen es de 7 cm por lado). Con piso perfecto el desvío medio es 0.1 cm.

`pose_source:=tf` usa la pose que ve el robot, no la real. En la pista básica terminó 2 vueltas con 0 choques; en la avanzada (margen de 7 cm) tuvo 20–37 choques, porque el realineado llega con retraso y el error de pose es comparable al margen. Para pistas angostas usa `truth` o mejora tu propio estimador.

### Arena abierta y control PID de la clase (RC-1)

`world:=arena_rc1` es una arena **abierta de 4 × 4 m**, sin carril ni cronómetro, para el cuadrado de 2 × 2 m del RC-1. Sin pista no hay `/race/*`: la verdad terreno es `/sim/pose`.

```bash
ros2 launch yahboomcar_sim sim.launch.py world:=arena_rc1 start:=1.0,1.0,0.0 robot_radius:=0.14 noise:=low
```

El nodo de la clase (`scripts_pdi/cuadrado_controller.py`: `/odom` → `/cmd_vel`) corre **sin modificar** contra el simulador, desde la carpeta que contiene `core_cuadrado.py`:

```bash
python3 cuadrado_controller.py --ros-args -p experimento:=cuadrado -p log_csv:=exp3b_sim.csv
```

Para comparar lo que cree el robot (`/odom`) con lo que pasó (`/sim/pose`) está `docs/rc1/cierre_real.py`: se lanza antes que el nodo, guarda ambas poses y al terminar con Ctrl+C imprime el cierre según cada una. El documento del reto y su guía es `RC1_Control_PID_2026-2.docx`.

Para quien mida o escriba su propio controlador:
- **Una prueba a la vez (o ≤ 5 pares en paralelo).** Con más, la CPU se satura, el temporizador de 30 Hz del nodo se atrasa y su derivada (Kd) amplifica ese jitter: el PID da resultados que no son del simulador. A poca carga, el experimento 1 (recta de 2 m, `ki:=0.5`, `line_offset:=0.10`, `bias_w:=0.06`) da `|e|` medio 2.56 cm, asentamiento 5.75 s y error final 0.49 cm, frente a 2.69 cm, 5.83 s y 0.48 cm en `08_cuadrado_sim.py`.
- **El piso gira al robot y `/odom` no lo ve.** Con `noise:=medium` el cuadrado en lazo cerrado sobre `/odom` puede salirse de la línea y rozar una pared aunque el nodo diga que cierra a pocos cm.
- `/imu/data_raw`: `angular_velocity.z` lleva el giro real; `orientation` copia el yaw de `/odom` y no sirve para fusionar.
- El simulador **no** modela el derrape de las 4 ruedas al girar (χ) ni el retardo de `/odom`; sí limita la aceleración (`accel_lin`, `accel_ang`).

### Topics y servicios

| Nombre | Tipo | Descripción |
|---|---|---|
| `/race/status` | `std_msgs/String` (JSON) | `status` (ready/running/finished), `lap`, `lap_progress`, `lap_time`, `last_lap`, `best_lap`, `lap_times`, `lateral_offset`, `wrong_way`, `collisions`, `top_speed` |
| `/race/lap_time` | `std_msgs/Float32` | Tiempo de cada vuelta al completarla |
| `/sim/pose` | `geometry_msgs/PoseStamped` | Pose real en `map` (verdad terreno, para evaluar tu algoritmo) |
| `/sim/collision` | `std_msgs/Bool` | `true` mientras el robot empuja una pared |
| `/sim/reset` | `std_srvs/Empty` | Robot a la salida y carrera a cero: `ros2 service call /sim/reset std_srvs/srv/Empty` |
| `/imu/data_raw`, `/vel_raw`, `/voltage` | `Imu`, `Twist`, `Float32` | Mismos nombres que el firmware del robot real |

Las vueltas se cuentan por el avance neto a lo largo de la línea central: ir de reversa resta y no se puede hacer trampa; `wrong_way` avisa si vas a contramano.

## 4. Con el código de la materia

- **Teleop, patrullaje, evitar obstáculos (`yahboomcar_laser`), `yahboomcar_ctrl`:** deberían funcionar con el simulador sin cambios, porque usan `/cmd_vel` y `/scan`.
- **Seguimiento de línea / pasillo con cámara:** usa `/camera/image_raw` (rgb8, 160×120, 15 Hz). El piso es gris con línea central amarilla discontinua y bordes blancos.
- **Mapeo:** lanzar con `standalone:=false` y luego el SLAM del curso (`slam_toolbox` viene instalado; `slam_gmapping` y cartographer de Yahboom no vienen en este entorno).
- **Navegación (Nav2):** los launch de `yahboomcar_nav` usan una ruta fija `/root/yahboomcar_ws/...` para el mapa. Pasar el mapa con `maps:=<ruta>` y lanzar el simulador con `standalone:=false`. Ese flujo todavía no está validado.

## 5. Pruebas

Núcleo sin ROS (pistas, física, carrera, cámara, piloto):
```bash
cd yahboomcar_ws/src/yahboomcar_sim && python -m unittest discover -s test -v
```

## Estado de las pruebas

- **Probado sin ROS (25 pruebas, incluida la arena abierta del RC-1):** las tres pistas caben en 3 × 2 m y no se cruzan; la dificultad crece con las curvas (radio mínimo, giro total, carril); la física (aceleración, choque, desplazamiento lateral, odometría); el conteo de vueltas (reversa no cuenta); la cámara; y que el piloto automático completa 2 vueltas sin choques en cada pista.
- **Probado con ROS 2 Humble (WSL, sin RViz):** compila con `colcon`; `sim_robot` + `autopilot` publican `/odom`, `/scan` (10 Hz), `/imu/data_raw`, `/vel_raw`, `/voltage`, `/camera/*` (15 Hz), `/sim/pose`, `/race/status`, y el TF `map → base_footprint`; la carrera termina con 2 vueltas y 0 choques en la intermedia (11.3 s y 11.1 s) y en la avanzada (14.0 s y 14.0 s); `/sim/reset` funciona.
- **Piso irregular y realineado (ROS 2 en WSL, pista cuadrada, odometría 5 %, `floor_noise` 0.05):** con `odom_align:=0` el error medio de `map→base_footprint` frente a la pose real fue 0.18 m (máx. 0.50 m); con `odom_align:=0.5` fue 0.07 m (máx. 0.15 m). El piloto automático sigue dando vueltas de ≈ 11 s con el piso ruidoso.
- **Piloto PID (ROS 2 en WSL, avanzada, ruido `high`, `pose_source:=truth`):** 2 vueltas en 29.1 s, 0 choques. Con `pose_source:=tf` en la básica: 2 vueltas, 0 choques; en la avanzada: 20–37 choques (ver arriba).
- **Pendiente de probar:** `sim.launch.py` completo con el modelo del robot y la ventana de RViz (en la máquina de pruebas faltaba `joint_state_publisher`; instálalo con `instalar_en_wsl.sh`), la vista de los marcadores y la cámara en RViz, y la integración con Nav2.
