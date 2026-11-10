# Entorno de simulacion x86_64 del robot Yahboom (ROS 2 Humble + RViz + Nav2).
# La imagen del robot real (yahboomtechnology/ros-humble) es arm64 y NO corre en PCs normales.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-teleop-twist-keyboard \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-numpy python3-yaml \
        xvfb mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# El workspace se monta como volumen en /root/yahboomcar_ws (ver SIMULACION.md)
WORKDIR /root/yahboomcar_ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/yahboomcar_ws/install/setup.bash ] && source /root/yahboomcar_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
