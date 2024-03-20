
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        rviz = Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', 'nav_ws/install/rviz_satellite/share/rviz_satellite/demo/demo.rviz']
                )

        gnss = Node(
                    package='gnss_nav_pkg',
                    executable='gnss_nav',
                    output='screen'
                )

        tf2 = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    output='screen',
                    arguments=['--frame-id', 'map', '--child-frame-id', 'gps_sensor']
                )

        return LaunchDescription([
            rviz,
            gnss,
            tf2
        ])