from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bag = LaunchConfiguration('bag')

    stereo_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stereo_image_proc'),
                'launch',
                'stereo_image_proc.launch.py'
            ])
        ),
        launch_arguments={
            'left_namespace': 'left',
            'right_namespace': 'right',
            'approximate_sync': 'False',
        }.items()
    )


    rosbag = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag,

            '--clock',
            '--loop',

            '--remap',
            '/cam0/image_raw:=/left/image_raw',
            '/cam0/camera_info:=/left/camera_info',
            '/cam1/image_raw:=/right/image_raw',
            '/cam1/camera_info:=/right/camera_info',
        ],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag',
            description='Ruta al rosbag'
        ),
        stereo_image_proc,
        rosbag,
        Node(
            package='tp3',
            executable='features',
            name='features',
            output='screen'
        ),
        Node(
            package='tp3',
            executable='matchings',
            name='matchings',
            output='screen'
        ),
        Node(
            package='tp3',
            executable='ransac',
            name='ransac',
            output='screen'
        ),
        Node(
            package='tp3',
            executable='triangulation',
            name='triangulation',
            output='screen'
        ),
        Node(
            package='tp3',
            executable='disparity',
            name='disparity',
            output='screen'
        ),
        Node(
            package='tp3',
            executable='stereo-pose',
            name='stereo_pose',
            output='screen'
        ),        
        Node(
            package='tp3',
            executable='dense-reconstruction',
            name='dense_reconstruction',
            output='screen'
        ),
    ])