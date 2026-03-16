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

    # 1. 坐标系转换发布
    map_odom_trans = Node(
        package='relocalization',
        executable='transform_publisher',
        name='transform_publisher',
        output='screen'
    )

    # 2. TEASER + GICP 结合节点
    # 先使用 TEASER++ 进行全局定位，再切换到 GICP 进行精细配准
    teaser_gicp_node = Node(
        package='relocalization',
        executable='teaser_gicp_node',
        name='teaser_gicp_node',
        output='screen',
        parameters=[
            # ====== 基础配置 ======
            {'map_path': '/home/getting/Desktop/FASTLIVO2Relocation/FAST_LIVO2_ROS2_relocation_ultra/all_raw_points.pcd'},
            {'map_frame_id': 'map'},
            {'pcl_type': 'livox'},

            # ====== TEASER 降采样参数 (较大体素，加快计算) ======
            {'map_voxel_leaf_size': 0.4},
            {'cloud_voxel_leaf_size': 0.4},

            # ====== GICP 降采样参数 (较小体素，提高精度) ======
            {'gicp_map_voxel_leaf_size': 0.2},
            {'gicp_cloud_voxel_leaf_size': 0.1},

            # ====== TEASER++ 参数 ======
            {'fpfh_normal_radius': 0.8},       # 法向量搜索半径 (增大)
            {'fpfh_feature_radius': 1.2},      # 特征描述子搜索半径 (增大)
            {'noise_bound': 0.3},              # 预期噪声水平 (米) (增大)
            {'teaser_solver_max_iter': 100},   # TEASER 最大迭代次数
            {'rotation_gnc_factor': 1.4},
            {'teaser_inlier_threshold': 5},    # TEASER 内点数阈值 (降低，因为实际内点很少)
            {'teaser_success_count': 3},       # TEASER 连续成功多少次后切换到 GICP

            # ====== GICP 参数 ======
            {'gicp_solver_max_iter': 50},
            {'num_threads': 16},               # 并行计算线程数
            {'max_correspondence_distance': 5.0},  # 对应点搜索半径
            {'fitness_score_thre': 0.2},       # GICP 适应度分数阈值
            {'converged_count_thre': 10},      # GICP 连续收敛次数后确认定位成功
            {'registration_type': 'VGICP'},    # 配准类型: VGICP 或 GICP
        ],
    )
    
    # 3. FAST-LIVO 定位模式
    fast_livo_param = os.path.join(
        config_path, 'avia_relocation.yaml')
    camera_param = os.path.join(
        config_path, 'camera_pinhole.yaml')
    fast_livo_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        parameters=[
            fast_livo_param,
            camera_param
        ],
        output='screen',
        remappings=[('/Odometry', '/state_estimation')]
    )
        
    # 4. RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('relocalization'), 'rviz', 'loam_livox.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 5. 延时启动逻辑
    # TEASER+GICP 节点和 FAST-LIVO 延时启动，等待其他节点就绪
    delayed_start_livo = TimerAction(
        period=5.0,
        actions=[
            teaser_gicp_node,
            fast_livo_node
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_odom_trans)
    ld.add_action(start_rviz)
    ld.add_action(delayed_start_livo)

    return ld
