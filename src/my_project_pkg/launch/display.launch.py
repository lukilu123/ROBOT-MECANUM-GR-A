import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # verifica que se está usando tiempo de simulación
    use_sim_time = LaunchConfiguration('use_sim_time')

    # procesa el archivo URDF
    pkg_path = os.path.join(get_package_share_directory('my_project_pkg'))
    xacro_file = os.path.join(pkg_path,'model','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    #eventualmente robot_description debería ser la carpeta /src/my_project_pkg/model
    
    # (NODO)crea un lanzador para robot_description
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    launch_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # (NODO)crea un lanzador para rviz2
    launch_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'config_file.rviz')]]
    )
    
    # (NODO)crea un lanzador para joint_state_publisher_gui
    launch_joint_state_publisher_gui= Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # (NODO)crea un lanzador 
    my_robot_controller= Node(
        package='my_project_pkg',
        executable='my_robot_controller',
        name='my_robot_controller',
    )

    # ejecuta los lanzadores!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use sim time if true'),
        launch_robot_state_publisher_node,
        launch_rviz2,
        launch_joint_state_publisher_gui,
        my_robot_controller,
        
    ])