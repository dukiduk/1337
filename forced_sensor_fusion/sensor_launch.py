from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_dir = get_package_share_directory('YOUR_PACKAGE_NAME')
    config_dir = os.path.join(package_dir, 'config')

    return LaunchDescription([

        #IMU + External Magnetometer fusion
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'imu_fusion.yaml')
            ]
        ),

        # Converts GPS to normal Cartesian odometry
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'navsat_transform.yaml')
            ]
        ),

        # Final EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'plane_ekf.yaml')
            ]
        ),
    ])