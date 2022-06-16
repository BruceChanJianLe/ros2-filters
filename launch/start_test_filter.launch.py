from launch import LaunchDescription
from launch_ros.actions import Node
# Launch Params Yaml
import os
from ament_index_python.packages import get_package_share_directory
# Shell Commands
from launch.actions import ExecuteProcess

def generate_launch_description():
    test_filter_node = Node(
        package="ros2-filters",
        executable="pc_to_pc_filter_chain",
        name="pc_to_pc_filter_chain_node",
        parameters=[os.path.join(get_package_share_directory("ros2-filters"), "config", "params_test_filter.yaml")],
    )

    start_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("ros2-filters"), "rviz2", "testFilter.rviz")],
        # parameters=[{"use_sim_time":True}]
    )

    play_rosbag = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-l", os.path.join(get_package_share_directory("ros2-filters"), "test", "point_cloud_filter_test_bag")],
        shell=True,
        output="screen"
    )

    # Define launch description
    ld = LaunchDescription()
    ld.add_action(test_filter_node)
    ld.add_action(start_rviz2)
    ld.add_action(play_rosbag)
    

    # Return launch description
    return ld