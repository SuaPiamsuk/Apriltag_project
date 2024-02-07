#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    
    launch_description = LaunchDescription()

    ## TESR
    apriltag_launch_cmd2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('apriltag_ros'), 'launch', 'tag_realsense.launch.py')
        ),
        launch_arguments={'camera_name': '/t265_camera','image_topic': 'image_raw'}.items()
    )

    undistort = Node(
        package='test_pkg',
        executable='camera_test.py',
        name='publisher_undist',
        # parameters=[{'use_sim_time': True}],
        output='screen'
    )

    start_t265_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='publisher_image_t265',
        # parameters=[{'enable_pose': True, 'device_type': 't265', 'fisheye_width': '848', 'fisheye_height': '800'}],
        output='screen'
    )

    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d' + os.path.join(get_package_share_directory('package_name'), 'config', 'config_file.rviz')]
    )




    
    # launch_description.add_action(use_rviz_launch_arg)
    # launch_description.add_action(namespace_launch_arg)
    # # launch_description.add_action(gazebo)
    # launch_description.add_action(full_spawn)

    # launch_description.add_action(gzserver_cmd)
    # launch_description.add_action(gzclient_cmd)

    # launch_description.add_action(camera_topic_helper)
    # launch_description.add_action(docking)
    # launch_description.add_action(apriltag_launch_cmd)

    launch_description.add_action(apriltag_launch_cmd2)
    launch_description.add_action(start_t265_camera)
    launch_description.add_action(undistort)
    launch_description.add_action(rviz2)

    return launch_description








