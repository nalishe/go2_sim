# go2_navigation/launch/navigation_gpt.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    go2_nav_share = get_package_share_directory('go2_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_bt_share = get_package_share_directory('nav2_bt_navigator')

    # 預設檔案（用字串路徑）
    default_map = os.path.join(go2_nav_share, 'map', 'map.yaml')
    default_params = os.path.join(nav2_bringup_share, 'params', 'nav2_params.yaml')
    default_bt = os.path.join(nav2_bt_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # LaunchConfiguration 變數
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')

    # 在 DeclareLaunchArgument 的 default_value 一律給「字串」'true'/'false'
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    declare_map         = DeclareLaunchArgument('map', default_value=default_map, description='Full path to map YAML')
    declare_params      = DeclareLaunchArgument('params_file', default_value=default_params, description='Full path to Nav2 params YAML')
    declare_bt          = DeclareLaunchArgument('default_bt_xml_filename', default_value=default_bt, description='BT XML file for Nav2')
    declare_autostart   = DeclareLaunchArgument('autostart', default_value='True', description='Autostart nav2 stack')
    declare_slam        = DeclareLaunchArgument('slam', default_value='False', description='Run SLAM instead of AMCL/map_server')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart,
            'slam': slam
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_bt)
    ld.add_action(declare_autostart)
    ld.add_action(declare_slam)
    ld.add_action(bringup)
    return ld
