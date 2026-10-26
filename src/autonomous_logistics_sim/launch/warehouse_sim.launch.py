import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Setup Paths
    pkg_sim_dir = get_package_share_directory('autonomous_logistics_sim')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_sim_dir, 'worlds', 'warehouse.world'),
        description='Absolute path to the Gazebo world file'
    )
    world_path = LaunchConfiguration('world')
    models_dir = os.path.join(pkg_sim_dir, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = models_dir + (os.pathsep + existing_model_path if existing_model_path else '')
    
    # 2. Set Model Variable (Crucial for Robot State Publisher)
    # We force it to waffle_pi to match your spawn command
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    model_folder = 'turtlebot3_waffle_pi'
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', model_folder, 'model.sdf')

    # 3. Define Launch Actions
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # This node sends the URDF/SDF to RViz
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Added a 5-second delay to spawning to ensure Gazebo is ready
    spawn_turtlebot3 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', model_folder,
                    '-file', sdf_path,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1' # Slightly higher start to avoid floor clipping
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        world_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot3
    ])
