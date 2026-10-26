import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():
    # --- 1. SETUP WORKSPACE PATHS ---
    pkg_sim_dir = get_package_share_directory('autonomous_logistics_sim')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_sim_dir, 'worlds', 'warehouse.world'),
        description='Absolute path to the Gazebo world file'
    )
    world_path = LaunchConfiguration('world')
    models_dir = os.path.join(pkg_sim_dir, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = models_dir + (os.pathsep + existing_model_path if existing_model_path else '')
    params_file_path = os.path.join(pkg_sim_dir, 'params', 'warehouse_params.yaml')
    rviz_config_dir = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    
    # --- 2. EXPOSE DYNAMIC LAUNCH ARGUMENTS ---
    # Overridable from the command line
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_sim_dir, 'maps', 'warehouse_map.yaml'),
        description='Full path to local map yaml configuration file'
    )
    
    run_nav_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Whether to deploy the Navigation2 lifecycle stack (set to false for mapping)'
    )

    # --- 3. EXPORT HARDWARE ENVIRONMENT VARIABLES ---
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    model_folder = 'turtlebot3_waffle_pi'
    sdf_path = os.path.join(pkg_tb3_gazebo, 'models', model_folder, 'model.sdf')

    # --- 4. GAZEBO SIMULATION BACKEND ENGINE ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # --- 5. ROBOT STATE PUBLISHER (TRANSFORM TREE) ---
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- 6. ROBOT ENTITY SPAWNER (TIME SYNCED) ---
    spawn_turtlebot3 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', model_folder,
                    '-file', sdf_path,
                    '-x', '0.0', '-y', '0.0', '-z', '0.05'
                ],
                parameters=[{'use_sim_time': True}],  # Syncs clock and prevents disappearing behavior
                output='screen'
            )
        ]
    )

    # --- 7. NAV2 NAVIGATION LIFECYCLE MONITOR (DELAYED GENERATION) ---
    nav2_stack = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
                condition=IfCondition(LaunchConfiguration('navigation')), # Runs only if navigation:=true
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': LaunchConfiguration('map'),
                    'params_file': params_file_path,
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items()
            )
        ]
    )

    # --- 8. VISUALIZATION WINDOW (SINGLE DELAYED INSTANCE) ---
    rviz_node = TimerAction(
        period=12.0, 
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        world_arg,
        map_yaml_arg,
        run_nav_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot3,
        nav2_stack,
        rviz_node
    ])
