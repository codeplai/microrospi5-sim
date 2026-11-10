"""Lanza el robot Yahboom virtual: modelo URDF + simulador + RViz (+ piloto automatico).

Ejemplos:
  ros2 launch yahboomcar_sim sim.launch.py                         # pista basica
  ros2 launch yahboomcar_sim sim.launch.py world:=pista_intermedia laps:=3
  ros2 launch yahboomcar_sim sim.launch.py world:=pista_avanzada autopilot:=true speed:=0.7
  ros2 launch yahboomcar_sim sim.launch.py world:=yahboomcar start:=0.5,0.5,1.57
  # Con Nav2 / SLAM (ellos publican /map y map->odom):
  ros2 launch yahboomcar_sim sim.launch.py standalone:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('yahboomcar_sim')
    desc_share = get_package_share_directory('yahboomcar_description')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map_yaml')
    start = LaunchConfiguration('start')
    standalone = LaunchConfiguration('standalone')
    use_rviz = LaunchConfiguration('rviz')

    # Si no se da map_yaml, se usa worlds/<world>.yaml
    default_map = PathJoinSubstitution(
        [sim_share, 'worlds', [world, TextSubstitution(text='.yaml')]])

    def p(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='pista_basica',
            description='Mundo: pista_basica | pista_intermedia | pista_avanzada | '
                        'pista_cuadrado | arena_rc1 (2.5x2.0 m abierta, RC-1) | '
                        'yahboom_map | yahboomcar'),
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='Ruta a un mapa .yaml propio (sobrescribe world)'),
        DeclareLaunchArgument('start', default_value='auto',
                              description="Pose inicial en el mapa: 'auto' o 'x,y,yaw'"),
        DeclareLaunchArgument('standalone', default_value='true',
                              description='true: el simulador publica /map y map->odom. '
                                          'false: lo hacen Nav2 / SLAM'),
        DeclareLaunchArgument('rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('laps', default_value='0',
                              description='Vueltas de la carrera (0 = sin limite)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Simular la camara frontal (/camera/image_raw)'),
        DeclareLaunchArgument('odom_noise', default_value='0.01',
                              description='Error relativo de la odometria (0 = perfecta)'),
        DeclareLaunchArgument('noise', default_value='medium',
                              description='Nivel de ruido (aplica a cualquier mundo): none | low | '
                                          'medium | high | custom (usa floor_noise, floor_grain, '
                                          'odom_noise tal cual)'),
        DeclareLaunchArgument('floor_noise', default_value='0.04',
                              description='Empuje aleatorio del piso en m/s (0 = piso perfecto)'),
        DeclareLaunchArgument('floor_grain', default_value='0.04',
                              description='Suciedad del piso en la camara (0 = limpio, 0.15 = mucha)'),
        DeclareLaunchArgument('odom_align', default_value='0.3',
                              description='1/s: map->odom realinea la odometria con la pose real '
                                          'mientras avanza (0 = sin realinear; solo standalone)'),
        DeclareLaunchArgument('robot_radius', default_value='-1.0',
                              description='Radio del robot en m (<0: automatico segun el mundo)'),
        DeclareLaunchArgument('accel_lin', default_value='2.5',
                              description='Limite de aceleracion lineal (m/s^2)'),
        DeclareLaunchArgument('accel_ang', default_value='8.0',
                              description='Limite de aceleracion angular (rad/s^2)'),
        DeclareLaunchArgument('autopilot', default_value='false',
                              description='Piloto automatico de referencia (solo pistas)'),
        DeclareLaunchArgument('controller', default_value='pid',
                              description='Piloto automatico: pid (corrige con 2 PID) | pursuit'),
        DeclareLaunchArgument('pose_source', default_value='truth',
                              description='Pose que usa el piloto: truth (/sim/pose) | '
                                          'tf (map->base_footprint, con odom_align)'),
        DeclareLaunchArgument('speed', default_value='0.6',
                              description='Velocidad maxima del piloto automatico (m/s)'),

        # Modelo del robot: robot_state_publisher + joint_state_publisher + base_footprint->base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'description_launch.py'))),

        # Lidar: mismo TF que usan los launch de yahboomcar_nav
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=['-0.0046412', '0', '0.094079', '0', '0', '0',
                       'base_link', 'laser_frame'],
        ),

        # El "robot": /cmd_vel -> /odom, TF odom->base_footprint, /scan, IMU, camara, carrera
        Node(
            package='yahboomcar_sim',
            executable='sim_robot',
            name='yahboomcar_sim',
            output='screen',
            parameters=[{
                'map_yaml': map_yaml,
                'start': start,
                'publish_map': standalone,
                'publish_map_to_odom': standalone,
                'laps': p('laps', int),
                'camera': p('camera', bool),
                'odom_noise': p('odom_noise', float),
                'robot_radius': p('robot_radius', float),
                'accel_lin': p('accel_lin', float),
                'accel_ang': p('accel_ang', float),
                'noise_level': p('noise', str),
                'floor_noise': p('floor_noise', float),
                'floor_grain': p('floor_grain', float),
                'odom_align': p('odom_align', float),
            }],
        ),

        Node(
            package='yahboomcar_sim',
            executable='autopilot',
            name='yahboomcar_autopilot',
            output='screen',
            parameters=[{'map_yaml': map_yaml, 'speed': p('speed', float),
                         'controller': p('controller', str),
                         'pose_source': p('pose_source', str)}],
            condition=IfCondition(LaunchConfiguration('autopilot')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(sim_share, 'rviz', 'sim.rviz')],
            condition=IfCondition(use_rviz),
        ),
    ])
