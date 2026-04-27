import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity,
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():

    # ── World argument ────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)')

    # ── Initial pose arguments ────────────────────────────────────────────────
    x_arg     = DeclareLaunchArgument('x', default_value='0.0',  description='Initial X position')
    y_arg     = DeclareLaunchArgument('y', default_value='0.0',  description='Initial Y position')
    z_arg     = DeclareLaunchArgument('z', default_value='0.1',  description='Initial Z position')
    roll_arg  = DeclareLaunchArgument('R', default_value='0.0',  description='Initial Roll')
    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',  description='Initial Pitch')
    yaw_arg   = DeclareLaunchArgument('Y', default_value='0.0',  description='Initial Yaw')

    # ── Mode argument ─────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='teleop',
        description='Control mode: teleop | auto | CLR')

    # ── Auto-mode arguments ───────────────────────────────────────────────────
    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='1.5',
        description='Velocity for auto mode (m/s)')

    steering_arg = DeclareLaunchArgument(
        'steering', default_value='0.8',
        description='Steering angle for auto mode (-1 to +1)')

    # ── CLR lateral controller arguments ─────────────────────────────────────
    # FIX: target_lateral_pos was used but never declared — added here
    target_lateral_pos_arg = DeclareLaunchArgument(
        'target_lateral_pos', default_value='0.0',
        description='Target lateral position for CLR lateral controller (m)')

    # ── Retrieve launch configurations ───────────────────────────────────────
    world_file = LaunchConfiguration('world')
    x          = LaunchConfiguration('x')
    y          = LaunchConfiguration('y')
    z          = LaunchConfiguration('z')
    roll       = LaunchConfiguration('R')
    pitch      = LaunchConfiguration('P')
    yaw        = LaunchConfiguration('Y')

    # ── Package paths ─────────────────────────────────────────────────────────
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)

    robot_description_path = os.path.join(package_path, 'model', 'vehicle.xacro')
    gz_bridge_params_path  = os.path.join(package_path, 'config', 'ros_gz_bridge.yaml')
    vehicle_params_path    = os.path.join(package_path, 'config', 'parameters.yaml')

    # ── Load URDF ─────────────────────────────────────────────────────────────
    robot_description = load_robot_description(robot_description_path, vehicle_params_path)

    # ── Gazebo launch ─────────────────────────────────────────────────────────
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    # ── Spawn robot ───────────────────────────────────────────────────────────
    robot_name = "ackermann_steering_vehicle"

    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robot_name,
                   '-string', robot_description,
                   '-x', x, '-y', y, '-z', z,
                   '-R', roll, '-P', pitch, '-Y', yaw,
                   '-allow_renaming', 'false'],
        output='screen')

    # ── Core nodes (always running) ───────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen')

    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[vehicle_params_path],
        output='screen')

    vehicle_state_node = Node(
        package='autonomous_systems_project_team_12',
        executable='vehicle_state',
        output='screen',
        parameters=[{'use_sim_time': True}])

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen')

    # ── Teleop-only node ──────────────────────────────────────────────────────
    keyboard_reader_node = Node(
        package='autonomous_systems_project_team_12',
        executable='keyboard_reader',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'teleop'"])))

    # ── Auto-only node ────────────────────────────────────────────────────────
    olr_node = Node(
        package='autonomous_systems_project_team_12',
        executable='olr_node',
        output='screen',
        parameters=[{
            'velocity':       LaunchConfiguration('velocity'),
            'steering_angle': LaunchConfiguration('steering'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'auto'"])))

    # ── CLR-only nodes ────────────────────────────────────────────────────────
    clr_alg1_speed_node = Node(
        package='autonomous_systems_project_team_12',
        executable='Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12',
        output='screen',
        parameters=[{
            'velocity':     LaunchConfiguration('velocity'),
            'use_sim_time': True,
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'CLR'"])))

    clr_alg2_lateral_node = Node(
        package='autonomous_systems_project_team_12',
        executable='Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12',
        output='screen',
        parameters=[{
            # FIX: target_lateral_pos now properly passed from declared arg
            'target_lateral_pos': LaunchConfiguration('target_lateral_pos'),
            # FIX: correct URDF values for wheel_radius and wheelbase
            'wheel_radius':       0.3,    # from URDF: wheel_radius default
            'wheelbase':          1.4,    # from URDF: 2 * (body_length/2 - wheel_radius) = 2*0.7
            'use_sim_time':       True,
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'CLR'"])))

    # ── Controllers ───────────────────────────────────────────────────────────
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # ── Launch description ────────────────────────────────────────────────────
    launch_description = LaunchDescription([
        # Controller sequencing
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_model_gazebo_node,
                on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state,
                on_exit=[forward_velocity, forward_position])),

        # Arguments
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        mode_arg,
        velocity_arg,           # FIX: was declared but never added to LaunchDescription
        steering_arg,           # FIX: was declared but never added to LaunchDescription
        target_lateral_pos_arg, # FIX: newly declared and added

        # Simulation
        gazebo_launch,
        spawn_model_gazebo_node,

        # Core nodes
        robot_state_publisher_node,
        vehicle_controller_node,
        vehicle_state_node,
        gz_bridge_node,
        rqt_graph_node,

        # Mode-conditional nodes
        keyboard_reader_node,
        olr_node,
        clr_alg1_speed_node,
        clr_alg2_lateral_node,
    ])

    return launch_description