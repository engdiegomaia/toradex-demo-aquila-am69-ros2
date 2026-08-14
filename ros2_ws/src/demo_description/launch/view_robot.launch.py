"""
Visualize the robot model in RViz2 — ML2 acceptance ("RViz works on x86").

Runs on: x86 workstation ONLY. RViz2 is an OGRE 2 / desktop-OpenGL application
and must never be placed on the Aquila AM69 (CLAUDE.md rule 1).

No Gazebo here on purpose: ML2 proves the URDF and the TF tree in isolation.
Simulation enters at ML3.

    ros2 launch demo_description view_robot.launch.py

The joint_state_publisher_gui sliders drive the wheel joints; the wheels should
visibly rotate in RViz. There is no `map` frame at this stage — that arrives with
Nav2's amcl in ML3 — so the RViz fixed frame is base_footprint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('demo_description'), 'urdf', 'demo_robot.urdf.xacro',
        ]),
        description='Absolute path to the robot xacro to visualize.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui to drive the wheel joints.',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2. Set false when only TF is being inspected.',
    )

    # Passthrough for xacro arguments, so dimensions and the mesh/primitive
    # switch can be tried without editing the model:
    #
    #   xacro_args:="use_meshes:=false"
    #   xacro_args:="wheel_separation:=0.30 chassis_radius:=0.20"
    #
    # Space-separated key:=value pairs, exactly as xacro takes them on the
    # command line. Empty by default, which expands the model as committed.
    xacro_args_arg = DeclareLaunchArgument(
        'xacro_args',
        default_value='',
        description='Extra xacro arguments, e.g. "use_meshes:=false".',
    )

    # ParameterValue(..., value_type=str) is required: without it the expanded
    # URDF is interpreted as a YAML document and robot_state_publisher rejects it.
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('model'),
            ' ', LaunchConfiguration('xacro_args'),
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Owns nothing in the TF tree — it only feeds joint angles to
    # robot_state_publisher, which publishes the transforms.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('demo_description'), 'rviz', 'demo_description.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        rviz_arg,
        xacro_args_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
