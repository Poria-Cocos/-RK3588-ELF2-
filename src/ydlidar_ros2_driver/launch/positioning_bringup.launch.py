#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
execute_laser_permission_command = ExecuteProcess(
    cmd=['sudo', 'chmod', '777', '/dev/ttyUSB0'],
    output='screen'
)
execute_imu_permission_command = ExecuteProcess(
    cmd=['sudo', 'chmod', '777', '/dev/ttyCH341USB0'],
    output='screen'
)
def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    pkg_share = FindPackageShare(package='ydlidar_ros2_ws').find('ydlidar_ros2_driver')
    
    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，设置为false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='bot_2d.lua')

    
    
    
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    laser_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    imu_node=Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_node_publisher',
            output='log',  # 控制节点的输出
            ros_arguments=['--log-level', 'error'],  # 设置日志级别为error
            
        )
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                   ###取消注释这里可以加载地图
                #    '-load_state_filename',"/home/myubuntu/workspace/ydlidar_ros2_ws/temp_map.pbstream"
                   ],
        remappings=[
            ('scan', 'scan'),
            ('imu','imu_data_ros')
        ],
        ros_arguments=['--log-level', 'error'],  # 设置日志级别为error
        )
    
        
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    tf2_node_laser = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    tf2_node_imu=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu_baselink',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_frame'],
        )
    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )

    return LaunchDescription([
        execute_laser_permission_command,
        execute_imu_permission_command,
        cartographer_node,
        occupancy_grid_node,
        params_declare,
        laser_node,
        tf2_node_laser,
        tf2_node_imu,
        imu_node,
        rviz2_node
    ])
