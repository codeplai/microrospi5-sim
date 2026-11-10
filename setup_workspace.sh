#!/bin/bash
# setup_workspace.sh
# Saca el workspace yahboomcar_ws de la imagen Docker al host y deja
# ros2_humble.sh montandolo como volumen, para que los cambios persistan.
# Es idempotente: se puede ejecutar varias veces sin romper nada.

set -e

WS_HOST="/home/pi/yahboomcar_ws"
ROS_SCRIPT="/home/pi/ros2_humble.sh"
IMAGE="yahboomtechnology/ros-humble:4.1.2"
TMP_CONTAINER="yahboom_ws_extract"

echo "==> Setup del workspace yahboomcar_ws"

# --- 1. Copiar el workspace al host si no existe ---
if [ -d "$WS_HOST" ]; then
    echo "[OK] $WS_HOST ya existe, omito la copia."
else
    if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        echo "[ERROR] La imagen $IMAGE no esta en el sistema."
        echo "        Descargala primero con: docker pull $IMAGE"
        exit 1
    fi
    echo "[..] Creando contenedor temporal para extraer el workspace..."
    docker rm -f "$TMP_CONTAINER" >/dev/null 2>&1 || true
    docker create --name "$TMP_CONTAINER" "$IMAGE" >/dev/null
    echo "[..] Copiando /root/yahboomcar_ws -> $WS_HOST (puede tardar, son ~2 GB)..."
    docker cp "$TMP_CONTAINER":/root/yahboomcar_ws "$WS_HOST"
    docker rm "$TMP_CONTAINER" >/dev/null
    echo "[OK] Workspace copiado."
fi

# --- 2. Anadir el bind-mount a ros2_humble.sh si no esta ---
if [ ! -f "$ROS_SCRIPT" ]; then
    echo "[ERROR] No existe $ROS_SCRIPT"
    exit 1
fi

if grep -q "yahboomcar_ws:/root/yahboomcar_ws" "$ROS_SCRIPT"; then
    echo "[OK] $ROS_SCRIPT ya monta el workspace."
else
    echo "[..] Anadiendo bind-mount a $ROS_SCRIPT (backup en ${ROS_SCRIPT}.bak)..."
    cp "$ROS_SCRIPT" "${ROS_SCRIPT}.bak"
    sed -i 's|^yahboomtechnology/ros-humble|-v /home/pi/yahboomcar_ws:/root/yahboomcar_ws \\\nyahboomtechnology/ros-humble|' "$ROS_SCRIPT"
    echo "[OK] Script actualizado."
fi

echo ""
echo "Listo. Ahora ejecuta:  ./ros2_humble.sh"
echo "Tu workspace vive en:  $WS_HOST  (editable desde el host)"
