# Relocalization

本包用于在 ROS2 环境下，将实时接收到的激光雷达点云与先验地图进行重定位（relocalization）。
实现了多种全局/局部配准算法并可组合使用：基于 TEASER++ 的全局配准（粗定位） + small_gicp（或 GICP）的精细配准，从而在大尺度场景下实现更快速、更精确的重定位。

## 主要功能

- 使用 TEASER++（全局鲁棒配准）做初始位姿估计，能在无初值情况下全局检索候选位姿。
- 使用 small_gicp（或 GICP）做局部精配准，基于 TEASER 结果进行跟踪与精化。
- 支持 Livox 原生 CustomMsg、以及标准 `sensor_msgs/PointCloud2`。
- 支持地图与在线点云的降采样以控制速度/精度权衡。
- 可配置的阈值和恢复策略：当 GICP 失败时可回退到 TEASER；也支持手动通过 `initialpose` 触发 GICP。

## 算法来源与致谢

- 基础框架以及 ICP、SAC_IA+ICP 实现来自：
	https://github.com/PolarisXQ/Fast-LIO2-Localization
- 全局配准（TEASER++）参考：
	https://github.com/MIT-SPARK/TEASER-plusplus
- small_gicp (改进版 GICP) 参考：
	https://github.com/koide3/small_gicp

得益于以上工作，本包将全局鲁棒初始化与高效的局部配准结合起来，在重定位精度与速度上相比单一方法有显著提升。

## 依赖（主要）

- ROS 2 Humble（或兼容版本）
- PCL
- Eigen3
- TEASER++（建议开启 FPFH 支持，见下）
- small_gicp
- livox_ros_driver2（可选，若使用 Livox）

注意：TEASER++ 的 FPFH 支持为可选项（CMake 选项 `BUILD_TEASER_FPFH`），若你在使用 TEASER 的 FPFH 功能，请确保在编译 TEASER++ 时启用该选项：

```bash
colcon build --packages-select teaserpp --cmake-args -DBUILD_TEASER_FPFH=ON
```

然后重新编译本包：

```bash
colcon build --packages-select relocalization
source install/setup.bash
```

## 快速启动（示例）

已经包含示例 launch 文件：

- `example_gicp.launch.py`：仅使用 small_gicp 的定位流程（适合已有较好初始位姿场景）。
- `example_teaser_gicp.launch.py`：先 TEASER++ 全局定位，达到阈值后切换到 GICP 精细配准（推荐用于完全丢失位姿或大范围重定位）。

运行示例：

```bash
source install/setup.bash
ros2 launch example_teaser_gicp.launch.py
```

如果你只想测试 GICP：

```bash
ros2 launch example_gicp.launch.py
```

## 参数说明（常用）

以下参数在 launch 文件或参数 YAML 中可以调整（示例中均有默认值）：

- `map_path` : 地图 PCD 文件路径（必填）
- `map_frame_id` : 地图 TF Frame，默认 `map`
- `pcl_type` : `livox` 或 `standard`

TEASER 参数：
- `fpfh_normal_radius`：法线估计半径（m）
- `fpfh_feature_radius`：FPFH 特征半径（m）
- `noise_bound`：TEASER 预期噪声水平（m）
- `teaser_inlier_threshold`：内点数阈值（达到后计为一次成功）
- `teaser_success_count`：连续成功次数后切换到 GICP

GICP / small_gicp 参数：
- `gicp_map_voxel_leaf_size`, `gicp_cloud_voxel_leaf_size`：地图与点云降采样体素大小（m）
- `num_threads`：并行线程数
- `max_correspondence_distance`：对应点搜索半径
- `fitness_score_thre`：GICP 适应度阈值（越小越严格）
- `converged_count_thre`：连续收敛次数阈值（判定定位成功）
- `registration_type`：`VGICP` 或 `GICP`

这些参数可以通过 launch 的 `parameters` 字段传入（参考 `example_teaser_gicp.launch.py`）。

## 节点话题（默认）

- 订阅：
	- `/pointcloud2` 或 `/livox/lidar`（取决于 `pcl_type`）——输入实时点云
	- `initialpose` —— 手动给定初始位姿（会触发 GICP 阶段）

- 发布：
	- `icp_result`（geometry_msgs/PoseWithCovarianceStamped）——发布重定位结果位姿
	- `prior_map`（sensor_msgs/PointCloud2）——发布下采样后的地图（用于 RViz）
	- `transformed_cloud`（sensor_msgs/PointCloud2）——发布根据当前估计变换后的点云以便可视化

## 可视化（RViz）

启动 `example_teaser_gicp.launch.py` 时会自动启动 RViz（如果系统配备），并加载 `rviz/loam_livox.rviz`。
确保 RViz 中：

- 订阅正确的 `Fixed Frame` 为 `map`
- 添加 `PointCloud2` 话题 `prior_map` 与 `transformed_cloud`，查看地图与当前变换后的点云

如果在 RViz 中看不到 `transformed_cloud`：

1. 检查节点日志，确认程序有调用 `publish_debug_cloud()` 并没有返回早期错误。
2. 确认 `transformed_cloud` 的 `header.frame_id` 与 RViz 中的 Fixed Frame（通常为 `map`）一致。
3. 如果 TEASER 无足够内点而没有生成变换，节点现在已改为在失败时发布降采样后的原始点云作为调试点云，方便查看。

## 调参建议

- 如果 TEASER 匹配到的内点很少（例如只有个位数），可：
	- 降低 `teaser_inlier_threshold`（或先观察对应数再调整）；
	- 放大 `fpfh` 搜索半径（`fpfh_normal_radius`, `fpfh_feature_radius`）；
	- 增大 `noise_bound` 提高鲁棒性。

- GICP 收敛慢或质量差时：
	- 减小 `gicp_cloud_voxel_leaf_size` 提高分辨率；
	- 增大 `max_correspondence_distance` 以容忍较大初始误差；
	- 调整 `fitness_score_thre` 以便判定准/不准。

## 故障排查

- 若编译时提示找不到 `teaserpp::teaser_features`，请确保 TEASER++ 在编译时启用了 FPFH：

```bash
colcon build --packages-select teaserpp --cmake-args -DBUILD_TEASER_FPFH=ON
```

- 若运行时看不到点云或 TF 错误：检查话题名、frame_id 与 RViz Fixed Frame 是否一致，并查看节点日志获取更多线索。

## 许可证与引用

该项目整合并改进了若干开源实现，请在学术或工程引用中同时引用相关工作：

- Fast-LIO2 Localization: https://github.com/PolarisXQ/Fast-LIO2-Localization
- TEASER++: https://github.com/MIT-SPARK/TEASER-plusplus
- small_gicp: https://github.com/koide3/small_gicp

欢迎基于本代码进一步改进。


