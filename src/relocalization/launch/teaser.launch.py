from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 保持原有的坐标变换发布节点 (如果需要把 icp_result 转为 odom->map 变换)
        Node(
            package='relocalization',
            executable='transform_publisher',
            name='transform_publisher',
            output='screen'
        ),
        
        # 新的 TEASER++ 节点
        Node(
            package='relocalization',
            # 注意：executable 必须与 CMakeLists.txt 中 add_executable 定义的名称一致
            # 建议命名为 "teaser_node"
            executable='teaser_node', 
            name='teaser_node',
            output='screen',
            parameters=[
                # === 地图与降采样配置 ===
                {'map_frame_id': 'map'},
                # 请务必修改为你实际的点云地图路径
                {'map_path': '/home/getting/humble/FAST-LIVO2-ROS2-v3/all_raw_points.pcd'}, 
                # 地图降采样 (TEASER++ 计算 FPFH 非常耗时，建议设大一点，例如 0.3 或 0.5)
                {'map_voxel_leaf_size': 0.3},   
                # 实时雷达点云降采样
                {'cloud_voxel_leaf_size': 0.3}, 
                # 雷达类型: 'livox' 或 'standard'
                {'pcl_type': 'livox'},

                # === FPFH 特征参数 (关键) ===
                # 注意：normal_radius 必须 > voxel_leaf_size
                #       feature_radius 必须 > normal_radius
                {'fpfh_normal_radius': 0.4},  # 计算法向量的搜索半径
                {'fpfh_feature_radius': 0.6}, # 计算 FPFH 特征的搜索半径

                # === TEASER++ 核心参数 ===
                {'noise_bound': 0.1},            # 预期的噪声水平 (单位: 米)
                {'teaser_solver_max_iter': 100}, # 最大迭代次数
                {'rotation_gnc_factor': 1.4},    # GNC 因子
                {'rotation_cost_threshold': 0.005} # 旋转代价阈值
            ]
        )
    ])