import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fast_livo_config_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    relocalization_rviz = os.path.join(
        get_package_share_directory("relocalization"),
        "rviz",
        "loam_livox.rviz",
    )

    default_map_path = os.path.abspath(os.path.join(os.getcwd(), "all_raw_points.pcd"))
    avia_config = os.path.join(fast_livo_config_dir, "avia_relocation.yaml")
    camera_config = os.path.join(fast_livo_config_dir, "camera_pinhole.yaml")

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=default_map_path,
        description="Prior PCD map path.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch RViz2.",
    )
    source_cloud_topic_arg = DeclareLaunchArgument(
        "source_cloud_topic",
        default_value="/cloud_registered",
        description="FAST-LIVO2 accumulated local cloud topic to align against the prior map.",
    )
    accumulate_duration_arg = DeclareLaunchArgument(
        "accumulate_duration_sec",
        default_value="10.0",
        description="Seconds of FAST-LIVO2 map output to accumulate before registration starts.",
    )

    transform_publisher = Node(
        package="relocalization",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
        parameters=[
            {"map_frame_id": "map"},
            {"odom_frame_id": "camera_init"},
            {"static_transform": False},
            {"publish_rate_hz": 20.0},
        ],
    )

    map_teaser_gicp_node = Node(
        package="relocalization",
        executable="map_teaser_gicp_node",
        name="map_teaser_gicp_node",
        output="screen",
        parameters=[
            {"map_path": LaunchConfiguration("map_path")},
            {"source_cloud_topic": LaunchConfiguration("source_cloud_topic")},
            {"source_frame_id": "camera_init"},
            {"map_frame_id": "map"},
            {
                "accumulate_duration_sec": ParameterValue(
                    LaunchConfiguration("accumulate_duration_sec"),
                    value_type=float,
                )
            },
            {"registration_period_sec": 1.0},
            {"max_accumulated_points": 300000},
            {"map_voxel_leaf_size": 0.4},
            {"cloud_voxel_leaf_size": 0.4},
            {"gicp_map_voxel_leaf_size": 0.2},
            {"gicp_cloud_voxel_leaf_size": 0.1},
            {"fpfh_normal_radius": 0.8},
            {"fpfh_feature_radius": 1.2},
            {"noise_bound": 0.3},
            {"teaser_solver_max_iter": 100},
            {"rotation_gnc_factor": 1.4},
            {"teaser_inlier_threshold": 5},
            {"teaser_success_count": 3},
            {"gicp_solver_max_iter": 50},
            {"num_threads": 16},
            {"max_correspondence_distance": 5.0},
            {"publish_fitness_score_thre": 0.5},
            {"final_fitness_score_thre": 0.2},
            {"stable_translation_thre": 0.05},
            {"stable_rotation_thre_deg": 1.0},
            {"stable_count_thre": 3},
            {"registration_type": "VGICP"},
        ],
    )

    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            avia_config,
            camera_config,
            {"locate_in_prior_map": False},
        ],
        output="screen",
        remappings=[("/Odometry", "/state_estimation")],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", relocalization_rviz],
        output="screen",
    )

    return LaunchDescription(
        [
            map_path_arg,
            use_rviz_arg,
            source_cloud_topic_arg,
            accumulate_duration_arg,
            transform_publisher,
            fast_livo_node,
            map_teaser_gicp_node,
            rviz_node,
        ]
    )
