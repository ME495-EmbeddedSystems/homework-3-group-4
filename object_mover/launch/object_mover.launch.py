from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fer", package_name="franka_fer_moveit_config").to_moveit_configs()

    return LaunchDescription([
        Node(package="rviz2",
             executable="rviz2",
             output="log",
             arguments=["-d", PathJoinSubstitution([FindPackageShare("franka_fer_moveit_config"), "config", "moveit.rviz"])],
             parameters = [ moveit_config.planning_pipelines, moveit_config.robot_description_kinematics],
             # Remapping actions not implemented, must remap individual topics/services
             # https://github.com/ros2/ros2/issues/1312 Accessed on 10/30/2024
             remappings = [("/move_action/_action/feedback", "/viz/move_action/_action/feedback"),
                           ("/move_action/_action/status", "/viz/move_action/_action/status"),
                           ("/move_action/_action/cancel_goal", "/viz/move_action/_action/cancel_goal"),
                           ("/move_action/_action/get_result", "/viz/move_action/_action/get_result"),
                           ("/move_action/_action/send_goal", "/viz/move_action/_action/send_goal"),
                           ]
             ),
        Node(package="object_mover",
             executable="pick_node",
             )
        ])