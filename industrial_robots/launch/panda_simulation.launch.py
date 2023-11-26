import os

# get the share directory of a package
from ament_index_python.packages import get_package_share_directory

# class for defining a launch file
from launch import LaunchDescription

# This action is used to declare launch file arguments
from launch.actions import DeclareLaunchArgument

# substitutions used to define command-line arguments
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

# This represents a ROS node that can be launched
from launch_ros.actions import Node

# built-in function required to run any launch file
def generate_launch_description():

    # Argument load_gripper:=true to load the gripper with robot
    # To get the hand in the robot_description
    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    # Original file >> Package name >> franka_description & folder name >> robots
    # Current file >> Package name >> industrial_robots & folder name >> urdf
    # panda_arm_urdf.xacro files collects 3 different xacro files
    # 1: panda_arm.xacro >> URDF of the robot
    # 2: hand.xacro >> URDF of the hand
    # 3. panda_arm.ros2_control.xacro >> This contains joints initial position
    # In all URDF files, fake_sensor_commands, fake_hardwares are used as the parameters. For real robot, this will be replaced by ip address
    franka_xacro_file = os.path.join(get_package_share_directory('industrial_robots'), 'urdf',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper])

    # Include the rviz file. This includes all the tools and features we need to show in rviz
    rviz_file = os.path.join(get_package_share_directory('industrial_robots'), 'rviz',
                             'panda_rviz.rviz')

    # Launch nodes
    # Modify this to include future scripts or other nodes
    return LaunchDescription([
        # Loads the gripper
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),
        # This node publishes the robot's state to the ROS Parameter Server using the robot description.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # joint_state_publisher_gui will allow us to visualize the joints and how they change with the help of the sliders
        # This will launch as a new window when we launch this file
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # Runs RViz with the specified configurations in rviz_file
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])