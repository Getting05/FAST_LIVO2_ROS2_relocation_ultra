import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('fast_livo'), 'config') 

  # icp relocalization
  map_odom_trans = Node(
      package='relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  icp_node = Node(
      package='relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          {'initial_x':0.0},
          {'initial_y':0.0},
          {'initial_z':0.0},
          {'initial_a':0.0},

          {'map_voxel_leaf_size':0.5},
          {'cloud_voxel_leaf_size':0.3},
          {'map_frame_id':'map'},
          {'solver_max_iter':100},
          {'max_correspondence_distance':0.1},
          {'RANSAC_outlier_rejection_threshold':0.5},
          # {'map_path':'/home/sentry_ws/src/sentry_bringup/maps/CC#0.pcd'},
          {'map_path':'/home/getting/humble/FAST_LIVO2_ROS2_relocation_edit/all_raw_points.pcd'},
          {'fitness_score_thre':0.2}, # 是最近点距离的平均值，越小越严格
          {'converged_count_thre':40}, # pcl pub at 20 hz, 2s
          {'pcl_type':'livox'},
      ],
  )
  
  # fast-livo localization
  fast_livo_param = os.path.join(
      config_path, 'avia_relocation.yaml')
  fast_livo_node = Node(
      package='fast_livo',
      executable='fastlivo_mapping',
      parameters=[
          fast_livo_param
      ],
      output='screen',
      remappings=[('/Odometry','/state_estimation')]
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('relocalization'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_livo = TimerAction(
    period=5.0,
    actions=[
      icp_node,
      fast_livo_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(map_odom_trans)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_livo)

  return ld
