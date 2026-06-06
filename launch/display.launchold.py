from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command 

def generate_launch_description():
    pkg_name = 'asterassembly_description'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'asterassembly.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')

    # CHARGEMENT ROBUSTE URDF/XACRO
    robot_description_content = Command(['xacro ', urdf_file])

    return LaunchDescription([
        
        # 1. TF STATIQUE : ANCRAGE MAP -> WORLD
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'], 
        ),

        # 2. LIEN STATIQUE DE REDRESSEMENT : world -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_base_link_setup',
            arguments=['0', '0', '0', '0', '1.5707', '0', 'world', 'base_link'],
         ),
        
        # 3. ROBOT_STATE_PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': False},
                {'robot_description': robot_description_content}
            ]
        ),
        
        # 4. INIT VISUELLE : JOINT_STATE_PUBLISHER_GUI (À FERMER manuellement pour libérer le mouvement)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # 5. CONTRÔLEUR LIPM (sera actif dès la fermeture du GUI)
        Node(
            package='asterassembly_description',
            executable='lipm_controller', 
            name='lipm_controller',
            parameters=[{'port': '/dev/ttyACM0'}],
        ),

        # 6. LANCEMENT DE RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
