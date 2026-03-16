import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

    # 获取 fast_livo 的配置路径 (保持不变)
    config_path = os.path.join(
        get_package_share_directory('fast_livo'), 'config') 

    # 1. 坐标系转换发布 (保持不变)
    # 它的作用是将 teaser_node 计算出的 pose (map->odom) 广播出去
    map_odom_trans = Node(
        package='relocalization',
        executable='transform_publisher',
        name='transform_publisher',
        output='screen'
    )

    # 2. 核心修改：使用 teaser_node 替代 small_gicp_node
    # TEASER++ 是全局配准，不需要 initial_x/y/z 等初值参数
    teaser_node = Node(
        package='relocalization',
        executable='teaser_node', # 请确保 CMakeLists.txt 中生成的可执行文件名也是这个
        name='teaser_node',
        output='screen',
        parameters=[
            # === 地图与降采样配置 ===
            {'map_frame_id': 'map'},
            # 请务必修改为你实际的点云地图路径
            {'map_path': '/home/getting/humble/FAST_LIVO2_ROS2_relocation_edit/all_raw_points.pcd'}, 
            {'pcl_type': 'livox'}, # 处理 Livox CustomMsg

            # === 降采样参数 ===
            # 降低体素大小以保留更多特征点，提高匹配质量
            {'map_voxel_leaf_size': 0.25},   # 地图降采样 (更小=更多点=更精确)
            {'cloud_voxel_leaf_size': 0.25}, # 实时点云降采样

            # === TEASER++ 核心参数 (FPFH) ===
            # 几何约束必须满足: voxel_leaf_size < normal_radius < feature_radius
            # 增大搜索半径以获得更稳定的特征
            {'fpfh_normal_radius': 0.5},    # 计算法向量的搜索半径 (m)
            {'fpfh_feature_radius': 1.0},   # 计算特征描述子的搜索半径 (m)

            # === TEASER++ 求解器参数 ===
            {'noise_bound': 0.3},           # 增大噪声容限，提高鲁棒性
            {'solver_max_iter': 200},       # 增加迭代次数 (注意：参数名修正)
            {'rotation_gnc_factor': 1.4},   # GNC 收敛因子
            {'inlier_threshold': 20},       # 降低最小内点数阈值，更容易通过验证
        ]
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
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 5. 延时启动逻辑
    delayed_start_livo = TimerAction(
        period=5.0,
        actions=[
            teaser_node,
            fast_livo_node
        ]
    )

    ld = LaunchDescription()

    ld.add_action(map_odom_trans)
    ld.add_action(rviz_node)
    ld.add_action(delayed_start_livo)

    return ld