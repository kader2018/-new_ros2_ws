import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'asterassembly_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'asterassembly.urdf')

    # [cite_start]1. Traitement de l'URDF (xacro) [cite: 1]
    robot_description_content = Command(['xacro ', urdf_file])

    # [cite_start]2. Robot State Publisher (indispensable pour les liens) [cite: 1]
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True 
        }]
    )

    # 3. Lancement de Gazebo Sim (Nouveau moteur pour Jazzy)
    # On lance un monde vide par défaut
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Faire apparaître le robot dans Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'aster',
                   '-z', '0.1'],
        output='screen'
    )

    # 5. Bridge (Le Pont) : Indispensable pour que Gazebo et ROS se parlent
    # Cela permet d'envoyer les JointStates entre les deux
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
