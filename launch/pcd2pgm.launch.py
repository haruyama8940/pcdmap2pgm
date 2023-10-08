import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
def generate_launch_description():
    return LaunchDescription([
    launch.actions.LogInfo(
                msg="Launch pcd2pgm convert pcd to pgm."
            ),
    Node(
        package='pcdmap2pgm',
        executable='convert_pcd2pgm',
        parameters=[{
                    'pcd_file':'/home/haru/bags/ros2bag_tsudanuma_gaisyu/0811/range120/trans_map.pcd',
                    # 'pcd_file':'/home/haru/bags/tsukuba/map_tsudanuma.pcd',
                    'thre_z_min':0.5,
                    'thre_z_max':4.0,
                    # 'flag_pass_through':0,
                    'flag_pass_through':1,
                    'grid_x':0.1,
                    'grid_y':0.1,
                    'grid_z':0.1,
                    'thre_radius':0.5,
                    'map_resolution':0.1,
                    'map_topic_name':'map'
                    }]
    )
    ])