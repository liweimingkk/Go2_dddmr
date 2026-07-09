# dfl-rlab/dddmr_navigation Issues Q&A 摘要

提取时间：2026-07-01  
来源：https://github.com/dfl-rlab/dddmr_navigation/issues  
口径：GitHub REST API 当前返回 71 个 issue、559 条评论。主要作者/维护者答复按仓库贡献者 `tsengapola` 和协作者 `pstsengb` 的评论归纳；无这两者回复的 issue 标为“无主要作者回复”。

## 高频结论

- “转圈不走 / `/cmd_vel` 为 0 / local plan 失败”常见原因：cuboid 内或周围有点云、`segmented_cloud_pure` 混入地面点、感知层把可通行地形标成障碍、ground/dGraph 稀疏或未清除。
- Mid360 已被多次确认可接入，但要仔细设置 TF、ground FOV、扫描参数；高速运动时仅靠 laser odometry 容易失败，作者多次建议外部 odom 或 FAST-LIO 作为 `/odom`。
- DDDMR 的 LeGO-LOAM 不是可直接替换成原版 LIO-SAM/LeGO-LOAM 的普通前端，它额外提供 pose graph、dense ground、改进 ICP 等导航需要的数据。
- 作者强烈建议使用官方 Docker，原因主要是 PCL/GTSAM/Jetson L4T/ROS2 环境参数容易不一致。
- 真机接入通常按“先建图并保存 map/ground，再启动 localization/navigation”推进；已有 Fast-LIO/外部定位时，可以绕开 mcl，将 `map->base_link` TF 和 `mapcloud/mapground` 接入规划器。

## 全部 Issue 摘要

| Issue | 状态 | 问题摘要 | 作者/维护者答复摘要 |
|---|---|---|---|
| [#73](https://github.com/dfl-rlab/dddmr_navigation/issues/73) | open | Mid360 给目标点后机器人只原地转圈，不朝目标走。 | 全局路径看起来已计算；重点查终端是否显示所有轨迹被拒绝。检查 cuboid 内是否有点云，以及 `/segmented_cloud_pure` 是否包含地面点，地面点会被当障碍。 |
| [#72](https://github.com/dfl-rlab/dddmr_navigation/issues/72) | open | Jazzy/Gazebo 下用 `/odom_3d` 后 localization XY 漂移。 | 先确认 IMU 坐标右手系、X 前进、Z 朝上；3D odom 应输入 `mcl_3dl`，不要接到 `mcl_feature`。作者请求 map、只含 `/odom`/IMU/lidar 的 bag 和初始位姿示意用于复现。 |
| [#71](https://github.com/dfl-rlab/dddmr_navigation/issues/71) | closed | 斜坡环境中全局路径从坡侧绕错，不走坡道。 | 用最新 commit，并替换相关源码段；根据坡高和 ground voxel size 调阈值。作者说明正在改进轮式/四足对楼梯坡道的差异处理，且新版已用 OMP 加速 static graph。 |
| [#70](https://github.com/dfl-rlab/dddmr_navigation/issues/70) | closed | Go2 + XT16 想无线运行，点云延迟和丢点严重。 | Jetson 用以太网接雷达，Wi-Fi 开热点给笔记本；DDDMR 栈运行在机器人侧。建议在 Jetson 上远程桌面开 RViz，减少大点云无线传输。Jetson Orin Nano 建议 JetPack 6.2.1。 |
| [#69](https://github.com/dfl-rlab/dddmr_navigation/issues/69) | open | mapping 时 `map->odom_combined` 变换缺失或更新慢。 | 作者要求补充：未运行 DDDMR 时系统是否发布 `map->odom_corrected`，使用什么计算平台，是否用 Release 模式编译。 |
| [#68](https://github.com/dfl-rlab/dddmr_navigation/issues/68) | open | 能否移植 ROS1，并适配 Fast-LIO 等 LIO SLAM。 | 理论上算法可移植，但不建议迁回 ROS1；Noetic/Ubuntu 20.04 会限制新版 PCL、GTSAM、YOLOv11、TensorRT 等路线，系统集成问题会很多。 |
| [#67](https://github.com/dfl-rlab/dddmr_navigation/issues/67) | open | 倒装 Mid360 后 mapping 显示异常。 | ground FOV 在 sensor frame 下设置；roll 180 可等价拆成 pitch 180 + yaw 180。作者根据 bag 找到并修复问题，建议用 commit `9307ec7...`，高速旋转时最好提供外部 odom。 |
| [#66](https://github.com/dfl-rlab/dddmr_navigation/issues/66) | open | `rs435_rgbd_848x380.zip` 下载不存在。 | 作者确认 bag 丢失，随后上传新 D455 bag，要求 pull 最新 commit；同时提醒录制带宽导致 depth/rgb 频率偏低。 |
| [#64](https://github.com/dfl-rlab/dddmr_navigation/issues/64) | closed | Mid360 25 度倾斜安装时地面投影/检测倾斜，周围有红色栅格。 | 修改 TF 和 ground FOV；倾斜后雷达后方看不到地面，需要忽略后方地面区域；天花板不应被映射成 ground。 |
| [#63](https://github.com/dfl-rlab/dddmr_navigation/issues/63) | open | `/cmd_vel` 是否适配阿克曼底盘；linear.x 为 0、angular.z 有值。 | 当前状态机会先旋转对齐再前进。可放宽朝向阈值，或改轨迹生成器让“原地旋转”也带小线速度；作者说 Ackermann 模型还需要时间发布。 |
| [#62](https://github.com/dfl-rlab/dddmr_navigation/issues/62) | closed | Exploration Mode 如何启动，语义分割模式何时可用。 | 作者先说明正在做 navigation while mapping，后续 exploration 基于该能力；随后回复已发布 naive exploring 和 navigation demo。 |
| [#61](https://github.com/dfl-rlab/dddmr_navigation/issues/61) | open | Go2 + XT16 大范围户外建图，机身下方 ground 空洞，人员被建进地图。 | 开启 `patch_first_ring_to_baselink`，静止调 ground FOV，确保首帧 ground 不空再慢速移动。16 线雷达不适合深度学习去人，先用点云编辑器删除；作者随后发布点云删除工具。 |
| [#60](https://github.com/dfl-rlab/dddmr_navigation/issues/60) | open | 用户迁移到 Gazebo Sim Harmonic / Jazzy 并汇报结果。 | 无主要作者回复。 |
| [#59](https://github.com/dfl-rlab/dddmr_navigation/issues/59) | open | 楼梯被 `segmented_cloud_pure` 当障碍，local planner 不让爬。 | 作者判断 local planner 轨迹全部被拒绝。可尝试聚类滤小点、减小 local perception window、用 ground cloud 忽略贴近地面的点；长期最好用可通行性/语义分割。 |
| [#58](https://github.com/dfl-rlab/dddmr_navigation/issues/58) | open | Go2 EDU 上 Hesai XT16 是否支持。 | 多数雷达都支持，可参考 C16 配置并按 XT16 规格改。若遇线程/构建问题，试 issue58 branch；PCL/GTSAM 交叉编译参数不匹配时建议官方 Docker，并提供 bag 复现。 |
| [#57](https://github.com/dfl-rlab/dddmr_navigation/issues/57) | open | RViz 里如何获得 3D Goal Pose 工具。 | 强烈建议 Docker；非 Docker 需要成功编译 `dddmr_rviz_tools`，该插件编译后可在 RViz tool 面板添加 3D Goal Pose。 |
| [#56](https://github.com/dfl-rlab/dddmr_navigation/issues/56) | open | Mid360 坐标变换后地图仍倾斜，四舵轮导航乱转。 | 作者要求安装照片和只含点云/odom 的 bag。四舵轮应先确认底层能正确执行 `linear.x/angular.z`，odom 坐标右手系和速度准确；后续作者发布 omni drive feature。 |
| [#55](https://github.com/dfl-rlab/dddmr_navigation/issues/55) | open | 感知能否跑在 RK3588 上。 | 理论上 RK3588 + 雷达可跑，作者在更弱的 Jetson Orin Nano 上测过；但需要自建 Docker，并按官方配置编译 PCL/GTSAM。 |
| [#54](https://github.com/dfl-rlab/dddmr_navigation/issues/54) | closed | `download_files.bash` 链接超时/404。 | 无主要作者回复。 |
| [#53](https://github.com/dfl-rlab/dddmr_navigation/issues/53) | open | Go2 + Mid360 遇障碍只停下/转身，不能绕障。 | 先看 dGraph、global marking、base_link 轴和 cuboid。问题包括 cuboid 附近有点、dGraph 未清除；可缩 cuboid、增大 ground 密度、提高 clearing FOV，或试 issue53 branch 的小簇过滤参数。 |
| [#52](https://github.com/dfl-rlab/dddmr_navigation/issues/52) | open | 没有轮式里程计能否导航。 | 建议使用 FAST-LIO 等 lidar odometry 包为 `MCL_3DL` 提供 odom。 |
| [#51](https://github.com/dfl-rlab/dddmr_navigation/issues/51) | open | 新分支后 Mid360 建图报 `Empty key frame2`。 | 更新到最新 commit；作者要求雷达倾角后给了 TF/config 示例。重点复查 base_link->雷达 TF 和 ground FOV。 |
| [#50](https://github.com/dfl-rlab/dddmr_navigation/issues/50) | open | 只有主图，不想用很多子图，如何运行。 | 将 PCD 改名为 `0_feature.pcd` 和 `0_ground.pcd` 放到指定目录；把 `sub_map_search_radius` 调到覆盖整图，把 warmup distance 设很大。 |
| [#49](https://github.com/dfl-rlab/dddmr_navigation/issues/49) | open | 是否可在 ARM 架构运行。 | 支持 NVIDIA Jetson Orin Nano/NX/AGX，参考 Dockerfile。 |
| [#48](https://github.com/dfl-rlab/dddmr_navigation/issues/48) | open | Livox Mid360 运行 `lego_loam_bag` exit code -11。 | 作者要求用 Docker、RelWithDebInfo/gdb、提供可复现 bag；给了 issue48 镜像/配置。后续还讨论 loop closure、TF 倾角、在线/离线 mapping、导航可用性和 lidar odom 的环境局限。 |
| [#47](https://github.com/dfl-rlab/dddmr_navigation/issues/47) | open | 如何在真机 Go2_W 上一步步运行。 | 流程：构建 Docker 和容器，确认 lidar topic/TF，配置 Mid360 TF 和参数，运行 LeGO-LOAM 建图，保存 `/tmp` 地图到 `dddmr_bags`，再启动 localization/navigation。反复强调 odom、时间戳、TF、base_footprint 和感知插件逐步排查。 |
| [#46](https://github.com/dfl-rlab/dddmr_navigation/issues/46) | closed | 是否计划用 Fast-LIO2 替代/增强 SLAM mapping/localization。 | 短期不会集成 Fast-LIO2，作者当时优先做 YOLOv11/perception。Mid360 的短期方案是调参和收集 bag；也指出用户配置中 C32/16 线、horizontal lines 等参数不匹配。 |
| [#45](https://github.com/dfl-rlab/dddmr_navigation/issues/45) | open | Mid360 倒装是否有样例配置。 | 设置雷达 pitch 约 180 度或 180+倾角，正确设置 ground FOV；作者根据用户 bag 给出配置，并要求使用最新 commit。 |
| [#44](https://github.com/dfl-rlab/dddmr_navigation/issues/44) | open | 启动后出现两个 `mcl_3dl`，且未发布 `map->odom_combined`。 | 检查 odom topic 和 TF 冲突；若外部已发 odom->base，可关闭 mcl_3dl 的 TF 发布。导航控制 frame 应在地面，如 `base_footprint`；倒装雷达时要避免感知输入包含 ground，cuboid 附近不能有点。 |
| [#42](https://github.com/dfl-rlab/dddmr_navigation/issues/42) | closed | LeGO-LOAM 报期望点数 X、实际点数 Y。 | `num_vertical_scans * num_horizontal_scans` 与雷达点数不匹配，需改水平扫描数/容差并重新编译。若崩溃，用最新 commit；宿主运行需 PCL 1.15/GTSAM 4.2a9 及官方编译参数。 |
| [#41](https://github.com/dfl-rlab/dddmr_navigation/issues/41) | closed | 外接里程计 TF 与工程默认方向相反，dGraph 未初始化。 | 无主要作者回复。 |
| [#40](https://github.com/dfl-rlab/dddmr_navigation/issues/40) | closed | 跑 `lego_loam_bag` 看不到回环连线。 | 默认可视化有 pose graph，黄色边/蓝色点；检查 `pose_graph` topic 是否有数据，除非显卡/RViz 问题，否则默认开启。 |
| [#39](https://github.com/dfl-rlab/dddmr_navigation/issues/39) | closed | 用 LIO-SAM map/ground/odom/TF 但无全局路径。 | 无主要作者回复。 |
| [#38](https://github.com/dfl-rlab/dddmr_navigation/issues/38) | closed | 四轮移动机器人能否用该框架。 | 可以，作者称框架最初就是给差速轮式机器人设计；前提是多线雷达、发布 odometry、计算平台能跑 Docker，然后按 beginner guide 从 mapping 开始。 |
| [#37](https://github.com/dfl-rlab/dddmr_navigation/issues/37) | open | 已有 Fast-LIO2 地图和定位，如何把 PCD 接入 DDDMR。 | 若使用自己的定位，只需提供 `map->base_link` TF，移除 `mcl_3dl/mcl_features`；发布 feature/ground PCD 到 `map_topic/ground_topic`。单 PCD 可命名为 `0_feature.pcd/0_ground.pcd`，或用 `pcl_publisher`；大地图需 Release 编译和静态层缓存/参数调整。 |
| [#36](https://github.com/dfl-rlab/dddmr_navigation/issues/36) | open | 实时建图后如何保存并用自己的地图导航。 | 实时 mapping 时调用保存服务，地图在 `/tmp` 下按日期生成，再复制到 `dddmr_bags`；bag replay 结束也会保存。导航/定位需要合适 odom，坡道/楼梯最好有 3D odom。 |
| [#35](https://github.com/dfl-rlab/dddmr_navigation/issues/35) | open | ground point cloud 不在同一平面。 | ground index 过高可能把低桌、楼梯甚至天花板当 ground；可降低 ground index。作者认为该案例可能仍能规划。 |
| [#34](https://github.com/dfl-rlab/dddmr_navigation/issues/34) | closed | 找不到本地 Docker image `dddmr_gz:x64`。 | 需要先构建 Docker image，并在脚本中选择 `x64_gz`。 |
| [#33](https://github.com/dfl-rlab/dddmr_navigation/issues/33) | closed | 建图和定位完成后，全局规划报错无路径。 | 日志显示 start/goal 在 ground 上，但路径找不到；检查 ground 连续性、连接半径、cuboid 内/周围障碍。可先移除 lidar plugin 隔离问题；后续控制超调可放宽 goal tolerance 或降低加速度。 |
| [#32](https://github.com/dfl-rlab/dddmr_navigation/issues/32) | open | 如何接入真实 Mid360 或 Point-LIO 处理后的点云。 | Mid360 支持 raw point cloud，但 TF/参数要细调。避免重复启动旧节点。由于 Mid360 非重复扫描，高速运动下 laser odometry 可能失败；可用 wheel odom 或 FAST-LIO 发布 `/odom`。 |
| [#31](https://github.com/dfl-rlab/dddmr_navigation/issues/31) | open | 能否直接用 LIO-SAM 替换源码里的 LeGO-LOAM。 | 不能直接替换；作者版 LeGO-LOAM 做了大量导航相关修改，包括给 `mcl_3dl` 的高效 pose graph、dense ground、改进 loop closure ICP 等。 |
| [#30](https://github.com/dfl-rlab/dddmr_navigation/issues/30) | open | Go2 EDU 部署后 `/cmd_vel` 一直为 0。 | 先确认是否发送 goal、是否生成 global path。设置 local planner 的 odom topic 和 TF，避免 stale TF/重复 odom->base TF。轨迹被拒绝通常是 cuboid 附近/内部障碍或地面点未去除；可用 `rejection_debug` branch 打印拒绝原因。 |
| [#29](https://github.com/dfl-rlab/dddmr_navigation/issues/29) | closed | 询问 ROS2 Humble 开源版本。 | 无主要作者回复。 |
| [#28](https://github.com/dfl-rlab/dddmr_navigation/issues/28) | open | 第一次导航成功，第二次目标后全局规划失败并恢复旋转。 | 作者怀疑 global planner crash 或 rclcpp race/CPU 过载；建议降低 dynamic-window query frequency，并使用 r36.4 Docker image。 |
| [#27](https://github.com/dfl-rlab/dddmr_navigation/issues/27) | open | Go2 EDU JetPack 5/R35 应使用哪个 Dockerfile。 | 可先试 L4T r36 image，导航栈在 JetPack 5.1.4 Orin Nano 上测试可用；语义分割/TensorRT 相关功能可能受限，未来更偏向 R36/JetPack 6。 |
| [#26](https://github.com/dfl-rlab/dddmr_navigation/issues/26) | open | Mid360 使用问题。 | Mid360 已在 main 支持，可直接用点云；作者给出完整配置思路，重点是正确的 `sensor2baselink` TF、pitch 和 ground FOV。 |
| [#25](https://github.com/dfl-rlab/dddmr_navigation/issues/25) | open | 自己建图/定位，只想用 DDDMR 规划，在 corridor 中 start not found。 | graph 已生成，多数正常；检查 `map->base_link` 是否在 ground 上或接近 ground cloud，cuboid 周围不能有点。若 base_link 在机身中心，应加地面的 `base_footprint`。 |
| [#24](https://github.com/dfl-rlab/dddmr_navigation/issues/24) | open | Airy 垂直安装后 ground detection 失败。 | 参考最新 Airy example 的 launch/config；投影方向很关键，`ground_scan_index` 要合理。作者请求 bag 和 TF 继续判断。 |
| [#23](https://github.com/dfl-rlab/dddmr_navigation/issues/23) | open | 编译 `rviz_tools` 报 PCL 版本问题。 | 推荐 PCL 1.15；宿主安装需参考 Dockerfile 中 PCL/GTSAM 编译步骤。作者仍强烈建议 Docker，便于未来功能兼容。 |
| [#22](https://github.com/dfl-rlab/dddmr_navigation/issues/22) | closed | Mid360 perception moving obstacles 不清除，voxel 累积。 | 先检查 map/ground rotation；ray casting 可能因雷达中心附近点云失败。作者在 issue22 branch 修复 casting line 严重错误，并说明 marking/clearing 依赖 global frame 和 localization/TF。 |
| [#21](https://github.com/dfl-rlab/dddmr_navigation/issues/21) | open | 倾斜雷达投影和 ground detection 讨论。 | 作者说明已修复倾斜投影方程；wheel odom 在 bag 中效果更好，laser odom 缺少 IMU/特征时容易差。还解释 LeGO-LOAM 内部 baselink 更接近 lidar frame，DDDMR 用外部 odom 时要做 frame 转换。 |
| [#20](https://github.com/dfl-rlab/dddmr_navigation/issues/20) | open | 为什么 launch 用 XML 而不是 Python，为什么不用 ROS2 components。 | 项目从 ROS1 迁移而来，作者熟悉 XML；需要时也会用 Python launch。Composable nodes 值得探索，但当前瓶颈更可能是算法计算量，而不是 ROS2 组织形式。 |
| [#19](https://github.com/dfl-rlab/dddmr_navigation/issues/19) | closed | 是否有 ROS1 版本。 | 作者明确不支持 ROS1。 |
| [#18](https://github.com/dfl-rlab/dddmr_navigation/issues/18) | open | Unitree B2 + 32 线雷达建图慢、ground 提取差。 | 检查 Release 编译、雷达频率、CPU。可减少 horizontal size、增大 ground patch size、增大 keyframe distance。导航崩溃还要检查 cuboid、odom frame、mcl frame 和 cuboid 内点云。 |
| [#17](https://github.com/dfl-rlab/dddmr_navigation/issues/17) | closed | B2W Jetson Orin Nano 上 Docker/CUDA 架构问题。 | 无主要作者回复。 |
| [#16](https://github.com/dfl-rlab/dddmr_navigation/issues/16) | open | 真实 Go2 的 `/utlidar/cloud` 接 LeGO-LOAM 报错。 | 当时作者回复 Unitree G4 lidar 属非重复扫描，`lego_loam_bor` 暂不支持。 |
| [#15](https://github.com/dfl-rlab/dddmr_navigation/issues/15) | open | 如何在真机运行，issue 正文只有简短称赞。 | 无主要作者回复。 |
| [#14](https://github.com/dfl-rlab/dddmr_navigation/issues/14) | open | Gazebo bag 建图漂移/崩溃。 | 作者能用 `lego_loam_bag_go2.launch` 成功建图；建议尝试 wheel/laser odom 切换。针对用户 bag，作者给 issue14 branch，并指出要改负向 vertical FOV、绕过窄走廊检查、减小 keyframe distance。 |
| [#13](https://github.com/dfl-rlab/dddmr_navigation/issues/13) | open | Unitree B2 上楼梯时，楼梯是否被当障碍。 | 全局规划可处理不连续 ground map，只要台阶点间距不超过约 50 cm；local planner 会把 perception 中的楼梯当障碍，除非实现楼梯/可通行区域过滤。后续发布语义分割示例。 |
| [#12](https://github.com/dfl-rlab/dddmr_navigation/issues/12) | open | Go2-W 没有 odom，想用 LeGO-LOAM laser odom 定位。 | 可试 issue12 branch；laser odom 不稳，作者没能在很多环境中可靠使用，建议 Fast-LIO。无需改源码时，TF 应类似 `map->lego_loam_map_laser_odom->base_link->laser_link`。 |
| [#11](https://github.com/dfl-rlab/dddmr_navigation/issues/11) | open | Gazebo/真机 Go2-W/B2-W 运行步骤混乱。 | 有地图后启动 localization launch，改 map dir，给 initial pose，再发 goal。雷达规格要设水平扫描数，建议 wheel odom。大量后续答复集中在 Docker build、Ubuntu apt/DNS/网络问题和拉取官方镜像。 |
| [#10](https://github.com/dfl-rlab/dddmr_navigation/issues/10) | closed | B2-W + Helios 5515 是否可用。 | 无主要作者回复。 |
| [#9](https://github.com/dfl-rlab/dddmr_navigation/issues/9) | open | 没有机器人，如何在 Gazebo 使用。 | 作者回复已发布 Unitree Go2 Gazebo tutorial。 |
| [#8](https://github.com/dfl-rlab/dddmr_navigation/issues/8) | open | global path planner 失败，目标附近到不了、DWA 卡住、Ctrl+C 难停。 | 初期判断 inscribed_radius 内有障碍，需按机器人尺寸调小。随后定位到 A* 扩展半径与 line-of-sight 检查问题，提供 issue branch；还建议增大 look_ahead_distance、检查 dGraph/marking、修正 z drift。 |
| [#7](https://github.com/dfl-rlab/dddmr_navigation/issues/7) | open | RS-Helios-5515 运行 `lego_loam.launch` crash。 | 作者要求 bag，测试后给 issue7 branch；多次指出 PCL/GTSAM 编译参数问题，建议 Docker。后续导航问题归因到 cuboid/地面点/雷达 FOV/感知 topic 丢失或过期。 |
| [#6](https://github.com/dfl-rlab/dddmr_navigation/issues/6) | closed | 更换 LiDAR，尤其 Mid360，是否计划支持。 | 作者称会集成 Mid360，需要 bag/spec；后续上传 Mid360 快速实现并合入 main。建议使用 FAST-LIO odom 改善 Mid360 laser odom 的激烈运动问题，raw point cloud 可用，核心是多帧 stitching 和倾角设置。 |
| [#5](https://github.com/dfl-rlab/dddmr_navigation/issues/5) | closed | demo 缺少 `dddmr_bags`，Mid360 实时 mapping 找不到输入 topic。 | 用下载脚本获取 demo bag；Mid360 实时 mapping 可参考对应 launch 并 remap 到实际 topic。当时作者提示 Mid360 非重复扫描，可能还需源码修改。 |
| [#4](https://github.com/dfl-rlab/dddmr_navigation/issues/4) | open | Robosense Helios 32 建图成功但 localization fail。 | 作者要求确认是否给 `mcl_3dl` 提供 wheel odom、使用哪个 launch、地图是否正确加载、是否给 initial pose，并提供录屏继续调试。 |
| [#3](https://github.com/dfl-rlab/dddmr_navigation/issues/3) | open | 真机运行时无法规划全局路径、RViz 3D goal 点击异常、想换 SLAM。 | 作者强调 ground 对全局规划很关键；替换 SLAM 时至少发布 ground PC 到 `mapground`。真机需要 odom，最好 3D odom；base_link/base_footprint 应在地面；Go2-W 可把 sportmodestate 转 `/odom`，已有地图定位时只需 `mcl_feature` 而非完整 LeGO-LOAM。 |
| [#2](https://github.com/dfl-rlab/dddmr_navigation/issues/2) | closed | demo bag 中机器人自行移动、全局轨迹慢、不跟踪目标、坐标翻转。 | bag 记录了遥控运动，所以会移动；未给正确 initial pose 时粒子滤波会在空间中随机猜。这个 demo 只是演示 bringup 和发 goal，不会真正开到你指定目标；需要 Gazebo 或真机闭环测试，并用 Release 编译。 |
| [#1](https://github.com/dfl-rlab/dddmr_navigation/issues/1) | closed | README 里提到的 `p2p_move_base_bag.launch` 缺失。 | 作者确认文档问题，并说明命令/launch 名称已改。 |
