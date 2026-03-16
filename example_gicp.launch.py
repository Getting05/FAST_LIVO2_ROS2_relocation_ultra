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

  # 1. 坐标系转换发布 (保持不变)
  map_odom_trans = Node(
      package='relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  # 2. 核心修改：使用 small_gicp_node 替代普通的 icp_node
  # small_gicp 通常速度更快，对多线程优化更好
  small_gicp_node = Node(
      package='relocalization',
      executable='small_gicp_node', # 确保 CMakeLists.txt 中生成的可执行文件名也是这个
      name='small_gicp_node',
      output='screen',
      parameters=[
          {'initial_x': 0.0},
          {'initial_y': 0.0},
          {'initial_z': 0.0},
          {'initial_a': 0.0},

          # GICP 对体素大小比较敏感，适当调整
          {'map_voxel_leaf_size': 0.2},   # 地图下采样
          {'cloud_voxel_leaf_size': 0.1}, # 当前雷达帧下采样
          
          {'map_frame_id': 'map'},
          {'solver_max_iter': 10000},       # 最大迭代次数
          {'max_correspondence_distance': 20.0}, # 对应点搜索半径，GICP可以适当放大一点
          {'RANSAC_outlier_rejection_threshold': 0.5},
          
          # 请务必确认这个 PCD 路径是正确的
          {'map_path': '/home/getting/humble/FAST_LIVO2_ROS2_relocation_edit/all_raw_points.pcd'},
          
          {'fitness_score_thre': 0.2},
          {'converged_count_thre': 20}, 
          {'pcl_type': 'livox'}, # 确保处理 Livox CustomMsg
          
          # Small GICP 特有参数 (如果源码中支持读取这些参数)
          {'num_threads': 16}, # 并行计算线程数
      ],
  )
  
  # 3. FAST-LIVO 定位模式 (保持不变)
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
        
  # 4. RViz (保持不变)
  rviz_config_file = os.path.join(
    get_package_share_directory('relocalization'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  # 5. 延时启动逻辑 (保持不变)
  delayed_start_livo = TimerAction(
    period=5.0,
    actions=[
      small_gicp_node, # 这里替换为新的节点变量
      fast_livo_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(map_odom_trans)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_livo)

  return ld