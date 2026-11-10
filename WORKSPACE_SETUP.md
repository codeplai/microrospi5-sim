# Setup del workspace yahboomcar_ws en el host

Documento de los cambios para sacar el workspace del contenedor Docker y montarlo desde el host, de modo que sea editable y persistente.

## Estado inicial

- `ros2_humble.sh` lanza el contenedor `yahboomtechnology/ros-humble:4.1.2` y ejecuta `/root/1.sh`.
- El script `/root/1.sh` (dentro del contenedor) sourcea `/root/yahboomcar_ws/install/setup.bash`.
- El workspace vivía **únicamente dentro del contenedor**, sin bind-mount al host. Cualquier cambio se perdía si el contenedor se eliminaba (`docker rm`).

### Workspaces detectados dentro de `/root/` del contenedor

| Workspace | Tamaño | Sourceado en `1.sh` |
|---|---|---|
| `yahboomcar_ws` | 2.0 GB | sí |
| `yahboomcar_ros2_ws` | 11 MB | no |
| `gmapping_ws` | 25 MB | no |
| `imu_ws` | 149 MB | no |

Por ahora solo se copia el activo (`yahboomcar_ws`). Los demás se pueden copiar de la misma forma si hacen falta.

## Para estudiantes: ejecutar `setup_workspace.sh`

Todo el procedimiento está empaquetado en [setup_workspace.sh](setup_workspace.sh). Es **idempotente**: si ya está hecho lo detecta y no hace nada destructivo.

```bash
./setup_workspace.sh
./ros2_humble.sh
```

Lo que hace el script:

1. **Copia el workspace al host** (solo si `/home/pi/yahboomcar_ws` no existe):
   - Crea un contenedor temporal `yahboom_ws_extract` desde la imagen `yahboomtechnology/ros-humble:4.1.2` (sin arrancarlo).
   - `docker cp` extrae `/root/yahboomcar_ws` al host.
   - Borra el contenedor temporal.
   - Ventaja vs. `docker cp <contenedor_corriendo>`: no requiere un contenedor activo y siempre saca la versión limpia de la imagen.

2. **Añade el bind-mount a `ros2_humble.sh`** (solo si no está):
   - Hace backup en `ros2_humble.sh.bak`.
   - Inserta la línea `-v /home/pi/yahboomcar_ws:/root/yahboomcar_ws \` justo antes de la línea `yahboomtechnology/ros-humble:4.1.2 ...`.

## Pasos manuales equivalentes (referencia)

```bash
# 1. extraer workspace
docker create --name tmp yahboomtechnology/ros-humble:4.1.2
docker cp tmp:/root/yahboomcar_ws /home/pi/yahboomcar_ws
docker rm tmp

# 2. añadir esta línea a ros2_humble.sh antes de la línea "yahboomtechnology/...":
#    -v /home/pi/yahboomcar_ws:/root/yahboomcar_ws \
```

## Cómo usar después

- Editar el workspace desde el host: `/home/pi/yahboomcar_ws/src/...`
- Lanzar el contenedor: `./ros2_humble.sh` (igual que antes; ahora el workspace persiste).
- Recompilar dentro del contenedor:
  ```bash
  cd /root/yahboomcar_ws
  colcon build --symlink-install
  ```

## Notas

- El typo `/tem/.X11-unix` en la línea 8 del script original sigue presente; no se toca aquí porque no es parte de esta tarea.
- Si en el futuro se necesitan también `yahboomcar_ros2_ws`, `gmapping_ws` o `imu_ws`, repetir el `docker cp` y añadir más `-v` al script.
- El contenedor sigue conteniendo la copia original en `/root/yahboomcar_ws` dentro de la imagen — el bind-mount simplemente la *oculta* en runtime; la imagen no se modifica.
