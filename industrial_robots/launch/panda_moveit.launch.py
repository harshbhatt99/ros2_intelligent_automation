import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# Extra import to resolve the error in semantic file (to explicitly define the content of yaml file as string)
import launch_ros.parameter_descriptions

# We are importing 4 config files:
# kinematics.yaml: It contains 
# ompl_planning.yaml: It contains Motion Planning configurations
# panda_controllers.yaml: It contains joint names
# panda_ros_controller.yaml: It contains PID values for joints
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # For simulation, we use the fake node and ip. We can replace this for real robot.
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'

    # LaunchConfiguration 
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    # Command-line arguments - we can set it to true in the terminal along with ros2 launch command

    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context

    # Include the URDF of the robot
    # Original: franka_description | robots | panda_arm.urdf.xacro
    # Current : industrial_robot | urdf | panda_arm_urdf.xacro
    franka_xacro_file = os.path.join(get_package_share_directory('industrial_robots'), 'urdf',
                                     'panda_arm.urdf.xacro')
    
    # In this command, we added robot_ip, hardware and sensor nodes extra compared to panda_simulation.launch.py
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    # To use moveit, we need to import SRDF files.
    # SRDF - Semantic Robot Description Format. 
    # This file complements URDF by defining joint groups, default robot configurations, additional collision checking information,...
    # ... and additional transforms that may be needed to completely specify the robot's pose.

    # Original: franka_moveit_config
    # Current: industrial_robots
    franka_semantic_xacro_file = os.path.join(get_package_share_directory('industrial_robots'),
                                              'srdf',
                                              'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {
        # Original File
        # 'robot_description_semantic': robot_description_semantic_config

        # Current File - Updated with launch_ros to explicitly define the content of yaml file as string (to resolve the error)
        'robot_description_semantic': launch_ros.parameter_descriptions.ParameterValue(
        robot_description_semantic_config, value_type=str)
    }

    # Original: franka_moveit_config
    # Current: industrial_robots
    kinematics_yaml = load_yaml(
        'industrial_robots', 'config/kinematics.yaml'
    )

    # Planning Functionality
    # OMPL - Open Motion Planning Library
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    # Original: franka_moveit_config
    # Current: industrial_robots
    ompl_planning_yaml = load_yaml(
        'industrial_robots', 'config/ompl_planning_config.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    # Original: franka_moveit_config
    # Current: industrial_robots

    # ------------------CONTROLLERS START----------------------#

    moveit_simple_controllers_yaml = load_yaml(
        'industrial_robots', 'config/panda_controllers.yaml'
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ------------------CONTROLLERS END----------------------#


    # Start the actual move_group node/action server
    # moveit_ros_move_group is the package available by moveit
    # Installation: sudo apt install ros-humble-moveit
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz - get the rviz file for moveit
    # Original: franka_moveit_config
    # Current: industrial_robots
    rviz_base = os.path.join(get_package_share_directory('industrial_robots'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Original: franka_moveit_config
    # Current: industrial_robots
    ros2_controllers_path = os.path.join(
        get_package_share_directory('industrial_robots'),
        'config',
        'panda_ros_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
    )

    robot_arg = DeclareLaunchArgument(
        # Original file: No default value
        # Current file: default_value='false' because all other DeclareLaunchArgument has 3 arguments. So, we need to put default_value in this also.
        robot_ip_parameter_name,
        default_value='false',
        description='Hostname or IP address of the robot.')
    
    # Original File >> default_value = 'false'
    # Current File >> default_value = 'true'
    # We need to set this to true when using the simulation and false when using the real robot
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='true',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    
    # Original file contains gripper.launch.py file from franka_gripper package
    
    #gripper_launch_file = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([PathJoinSubstitution(
    #        [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
    #    launch_arguments={'robot_ip': robot_ip,
    #                      use_fake_hardware_parameter_name: use_fake_hardware}.items(),
    #)
    return LaunchDescription(
        [robot_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
         franka_robot_state_broadcaster,
         # gripper_launch_file
         ]
        + load_controllers
    )


# IMPORTANT:
# See the modifications in the following files
# package.xml
# panda_arm.srdf.xacro
# panda_arm.xacro

# INSTALL FOLLOWING PACKAGES
# sudo apt update
# sudo apt install ros-humble-joint-state-publisher
# sudo apt install ros-humble-controller-manager
# sudo apt install ros-humble-joint-state-publisher-gui
# sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers