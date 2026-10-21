from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="vojext_robot_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="vojext_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="vojext",
            description="Prefix for link and joint names, for multi-robot setups",
        ),
        DeclareLaunchArgument(
            "launch_robot_state_publisher",
            default_value="false",
            description="Launch robot_state_publisher. Normally it is launched by the description package.",
        ),
        DeclareLaunchArgument(
            "config_package",
            default_value="roboteq_motor_controller_driver",
            description="Package with configuration file",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="roboteq_driver.yaml",
            description="Configuration file",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="roboteq_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_robot_state_publisher = LaunchConfiguration("launch_robot_state_publisher")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    controllers_file = LaunchConfiguration("controllers_file")

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "robots", description_file]),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Configuration
    roboteq_configuration = PathJoinSubstitution([FindPackageShare(config_package), "config", config_file])
    controllers_configuration = PathJoinSubstitution([FindPackageShare(config_package), "config", controllers_file])

    # The controller manager reads the robot description from the robot_description topic.
    # Normally robot_state_publisher is launched by the description package.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
        condition=IfCondition(launch_robot_state_publisher),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[roboteq_configuration, controllers_configuration],
        output="screen",
    )

    # The controller manager no longer forwards its parameters to the controllers,
    # so the parameter files are passed to each controller through its spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--param-file",
            controllers_configuration,
        ],
    )

    # The broadcaster also needs the Roboteq driver parameters (queries list)
    roboteq_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "roboteq_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--param-file",
            controllers_configuration,
            "--param-file",
            roboteq_configuration,
        ],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--param-file",
            controllers_configuration,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            roboteq_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
