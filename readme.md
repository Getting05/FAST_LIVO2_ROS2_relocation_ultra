# FAST-LIVO2 ROS2 HUMBLE RELOCALIZATION ULTRA

## 1. 项目介绍

本项目基于 [**FAST-LIVO2**](https://github.com/hku-mars/FAST-LIVO2)、[**FAST-LIVO2 ROS2**](https://github.com/SuperLDG/FASTLIVO2_ROS2)以及[**Fast-LIO2-Localization**](https://github.com/PolarisXQ/Fast-LIO2-Localization)，在原有的 LiDAR-Inertial 里程计与建图功能上，**增强了雷达点云重定位（relocalization）能力**。

整体框架来源于：

- FAST-LIVO2 ROS2: 激光–惯导里程计与建图
- Fast-LIO2-Localization: 原始 ICP / SAC-IA+ICP 重定位框架

在此基础上，本项目新增并集成了：
- **修改FAST-LIVO2 ROS2以适应重定位的框架**

- **GICP / small_gicp 重定位**  
  参考 https://github.com/koide3/small_gicp  
  使用高效并行的 GICP / VGICP 算法替代传统 ICP，提高配准精度与速度。

- **TEASER++ 全局重定位**  
  参考 https://github.com/MIT-SPARK/TEASER-plusplus  
  利用鲁棒特征匹配和最大 clique 求解，在**无初始位姿**的情况下进行全局粗定位。

- **TEASER++ + small_gicp 组合重定位**  
  使用 TEASER++ 提供可靠粗位姿作为初值，再由 small_gicp 进行精细迭代配准，显著提升重定位的**成功率、精度和收敛速度**。

> 简单理解：  
> - FAST-LIVO2 做**连续建图与里程计**；  
> - `relocalization` 包负责将当前点云与先验地图进行**重定位**，支持 ICP、SAC-IA+ICP、GICP 以及 TEASER++ + GICP 组合方案。


## 2. 依赖环境（Prerequisites）

### 2.1 系统与 ROS

- Ubuntu 22.04
- ROS 2 Humble

ROS2 安装可参考官方文档：  
http://wiki.ros.org/ROS/Installation

### 2.2 基础第三方库

- **PCL** ≥ 1.6  
  点云处理库  
  安装参考：https://pointclouds.org/

- **Eigen** ≥ 3.3.4  
  线性代数库  
  安装参考：https://eigen.tuxfamily.org/

- **OpenCV** ≥ 3.2  
  图像相关（FAST-LIVO2 中用到）  
  安装参考：http://opencv.org/

### 2.3 Sophus

可直接使用 ROS2 的二进制版本：

```bash
sudo apt install ros-$ROS_DISTRO-sophus
```

或手动从源码安装（如遇到 `so2.cpp` 的赋值错误，需要小改一行）：

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff  # 与 Fast-LIO2/FAST-LIVO2 兼容的旧版本
mkdir build && cd build
cmake ..
make -j
sudo make install
```

若编译失败报 `so2.cpp:32:26: error: lvalue required as left operand of assignment`，修改 `so2.cpp`：

```diff
namespace Sophus
{

SO2::SO2()
{
-  unit_complex_.real() = 1.;
-  unit_complex_.imag() = 0.;
+  unit_complex_.real(1.);
+  unit_complex_.imag(0.);
}
```

### 2.4 Vikit

项目已经自带 Vikit 相关代码：

- rpg_vikit
- `src/vikit_common/`
- `src/vikit_py/`
- `src/vikit_ros/`

无需额外安装，按本仓库一起编译即可。

### 2.5 Livox ROS Driver 2

若使用 Livox 雷达（如 MID360），需要：

- `livox_ros_driver2`

安装参考官方仓库：  
https://github.com/Livox-SDK/livox_ros_driver2

> 说明：  
> 本项目使用 `livox_ros_driver2` 的 ROS2 版本，其 `CustomMsg` 与老版 `livox_ros_driver` 基本兼容，但后者没有原生 ROS2 支持，因此统一使用 `livox_ros_driver2`。

### 2.6 small_gicp

用于高效 GICP/VGICP：

- 仓库：https://github.com/koide3/small_gicp

本项目以 ROS2 包形式引入（small_gicp），会跟随本仓库一起构建。  
若单独参考安装方式，可见原仓库。

### 2.7 TEASER++

用于全局鲁棒配准：

- 仓库：https://github.com/MIT-SPARK/TEASER-plusplus

本项目以 ROS2 包形式引入（TEASER-plusplus）。

> **重要**：需要在编译 TEASER++ 时开启 FPFH 支持，否则无法使用本项目中的 FPFH 特征接口：  
> `-DBUILD_TEASER_FPFH=ON`



## 3. 编译与安装

### 3.1 将本仓库放入 ROS2 工作空间

假设你的 ROS2 工作空间为 `~/humble`，结构大致如下：

```bash
~/humble/
  ├── src/
  │   ├── FAST_LIVO2_ROS2_relocation_edit/  # 本仓库
  │   ├── 其他包...
  └── ...
```

确保本仓库内的结构为（简略）：

```bash
FAST_LIVO2_ROS2_relocation_edit/
  ├── src/
  │   ├── FAST-LIVO2/          # fast_livo 主包
  │   ├── relocalization/  # 重定位包
  │   ├── livox_ros_driver2/
  │   ├── small_gicp/
  │   ├── TEASER-plusplus/
  │   └── rpg_vikit/ ...
  └── readme.md
```

### 3.2 编译 livox_ros_driver2

**注意：安装时请退出conda环境。**

**先将`relocalization`移出，避免执行`livox_ros_driver`编译报错。**

进入 `livox_ros_driver2` 目录执行脚本（仓库内已自带）：

```bash
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit/src/livox_ros_driver2
./build.sh humble
```

这一步会在你的工作空间中安装 `livox_ros_driver2`。

### 3.3 编译 small_gicp 与 TEASER++（开启 FPFH）

回到工作空间根目录，编译下游依赖：

```bash
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit

# 编译 small_gicp + TEASER++，并为 TEASER++ 打开 FPFH 支持
colcon build --packages-select small_gicp teaserpp --cmake-args -DBUILD_TEASER_FPFH=ON
```

完成后，source 一下：

```bash
source install/setup.bash
```

### 3.4 编译 relocalization 和整体工程
**移回 `relocalization` 包**

```bash
# 仅编译重定位包（可选）
colcon build --packages-select relocalization 

# 编译整个工作空间
colcon build

# 每次新终端需要 source
source install/setup.bash
```



## 4. 示例运行

### 4.1 启动 Livox 雷达驱动

```bash
# 终端1
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 4.2 启动 FAST-LIVO2 里程计与建图

```bash
# 终端2
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True
```

该 launch 会启动 FAST-LIVO2 的主节点 `fastlivo_mapping`，并开启 RViz 可视化。

### 4.3 启动原始 ICP 重定位（参考 Fast-LIO2-Localization）

```bash
# 终端3
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch example.launch.py
# 启动基于 ICP 的重定位（来自 PolarisXQ 原实现）
```
偏差角度大于10度时，可能无法收敛。

### 4.4 启动 small_gicp 重定位

```bash
# 终端3
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch example_gicp.launch.py
# 使用 small_gicp 进行局部重定位，适合已经有较好初始位姿的场景
```

相比icp速度大大提升。偏差角度大于30度时，可能无法收敛。


### 4.5 启动 TEASER++ 重定位

```bash
# 终端3
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch example_teaserpp.launch.py
```

在任意偏差、环境变化较大的情况下均能匹配，但很难收敛，精度有限。

### 4.6 启动 TEASER++ + GICP 组合重定位（推荐）

```bash
# 终端3
cd ~/humble/FAST_LIVO2_ROS2_relocation_edit
source install/setup.bash

ros2 launch example_teaser_gicp.launch.py
```

在任意偏差、环境变化较大的情况下均能匹配，并能快速收敛到高精度结果，推荐使用。


example_teaser_gicp.launch.py 的逻辑：

1. 启动 `transform_publisher`：发布必要的坐标变换。
2. 启动 `teaser_gicp_node`：
   - 使用 TEASER++ 进行全局特征匹配，估计粗略位姿；
   - 达到一定内点数阈值与连续成功次数后，切换到 small_gicp 进行精细配准；
   - 收敛后在话题 `icp_result` 发布最终位姿，并在 `transformed_cloud` 发布对齐后的点云。
3. 启动 `fastlivo_mapping`（如果在该 launch 中包含）和 RViz。

> 你可以在 RViz 中订阅：  
> - `prior_map`（下采样地图）  
> - `transformed_cloud`（当前变换后的点云）  
> 并设置 Fixed Frame 为 `map` 进行可视化比对。



## 5. 报错与常见问题

### 5.1 找不到 `liblivox_lidar_sdk_shared.so`

1. 先在系统中搜索该库：

```bash
find /usr/lib /usr/local/lib ~/humble -name "liblivox_lidar_sdk_shared.so"
```

若能找到，如：

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
/home/getting/humble/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so
```

则加入 LD_LIBRARY_PATH：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

并写入 `~/.bashrc` 以持久生效：

```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
```

若完全找不到该库，需重新编译 / 安装 Livox SDK。

### 5.2 `image_transport/compressed_sub` 插件缺失

运行某些依赖压缩图像的节点（如 `republish`）时报错：

```text
image_transport/compressed_sub plugin missing ...
```

安装：

```bash
sudo apt install -y ros-humble-image-transport-plugins
```

### 5.3 TEASER++ 相关链接错误

若编译 `relocalization` 时，提示：

- 找不到 `teaserpp::teaser_features`
- 或 FPFH 相关符号未定义

请确认：

```bash
colcon build --packages-select teaserpp --cmake-args -DBUILD_TEASER_FPFH=ON
source install/setup.bash
colcon build --packages-select relocalization
```



## 6. 参考项目

本项目大量参考并基于以下开源工作：

- FAST-LIVO2（LiDAR-Inertial 里程计与建图）：

  https://github.com/hku-mars/FAST-LIVO2

- FAST-LIVO2 ROS2（基础框架与 FAST-LIO2 适配）：  
  https://github.com/SuperLDG/FASTLIVO2_ROS2

- Fast-LIO2 Localization（ICP / SAC-IA+ICP 重定位框架）：  
  https://github.com/PolarisXQ/Fast-LIO2-Localization

- small_gicp（高效 GICP / VGICP 实现）：  
  https://github.com/koide3/small_gicp

- TEASER++（鲁棒全局点云配准）：  
  https://github.com/MIT-SPARK/TEASER-plusplus

