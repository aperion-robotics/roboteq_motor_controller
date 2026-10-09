from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    config_package = LaunchConfiguration("config_package")
    config_file = LaunchConfiguration("config_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "robots", description_file]),
            " ",
            "prefix:=",
            prefix
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Configuration
    roboteq_configuration = PathJoinSubstitution([FindPackageShare(config_package), "config/", config_file.perform(context)])

    initial_joint_controllers = PathJoinSubstitution([FindPackageShare(config_package), "config/", controllers_file.perform(context)])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            roboteq_configuration,
            initial_joint_controllers
            # ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )

    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags,
        )

    controller_spawner_names = [
        "joint_state_broadcaster",
        "roboteq_broadcaster"
    ]
    controller_spawner_inactive_names = []

    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
    )
    
    nodes_to_start = [
        control_node,
        initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="vojext_robot_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="vojext_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="vojext",
            description="Prefix for link and joint names, for multi-robot setups",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_package",
            default_value="roboteq_motor_controller_driver",
            description="Package with configuration file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="roboteq_driver.yaml",
            description="Configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="roboteq_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="diff_drive_controller",
            description="Initially loaded robot controller.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])