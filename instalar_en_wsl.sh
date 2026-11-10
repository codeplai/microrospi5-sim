#!/bin/bash
# instalar_en_wsl.sh
# Instala ROS 2 Humble + RViz + Nav2 en Ubuntu 22.04 (WSL2) y compila el simulador del robot Yahboom.
# Uso (dentro de Ubuntu 22.04 en WSL2, desde la carpeta del proyecto):  bash instalar_en_wsl.sh
set -e

if ! grep -q "22.04" /etc/os-release; then
    echo "[ERROR] Se necesita Ubuntu 22.04 (Humble no soporta otras versiones)."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/yahboomcar_ws"
if [ ! -d "$WS/src/yahboomcar_sim" ]; then
    echo "[ERROR] No encuentro $WS/src/yahboomcar_sim"
    exit 1
fi

echo "==> 1/3 Repositorio de ROS 2"
sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "==> 2/3 Paquetes (descarga grande, ~3 GB)"
sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    python3-colcon-common-extensions python3-numpy python3-yaml

echo "==> 3/3 Compilando yahboomcar_description + yahboomcar_sim"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select yahboomcar_description yahboomcar_sim --symlink-install

LINE1="source /opt/ros/humble/setup.bash"
LINE2="source $WS/install/setup.bash"
grep -qxF "$LINE1" ~/.bashrc || echo "$LINE1" >> ~/.bashrc
grep -qxF "$LINE2" ~/.bashrc || echo "$LINE2" >> ~/.bashrc

echo ""
echo "Listo. Abre una terminal nueva y ejecuta:"
echo "  ros2 launch yahboomcar_sim sim.launch.py"
