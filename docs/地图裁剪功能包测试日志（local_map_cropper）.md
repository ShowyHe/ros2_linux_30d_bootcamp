# 地图裁剪功能包测试（local_map_cropper）

## 1. 本次任务目标

基于已有全局地图，在仿真环境中根据机器人当前位置裁剪出一张更小的局部地图，并将裁剪结果交由 Nav2 的 `map_server` 加载，最终在 RViz2 中显示。

---

## 2. 当前完成情况

本次已完成以下闭环：

1. 读取已有全局地图 `/map`
2. 通过 TF 获取机器人在 `map` 坐标系下的当前位置（`map -> base_link`）
3. 以机器人当前位置为中心裁剪局部子图
4. 生成 `cropped_map.pgm` 和 `cropped_map.yaml`
5. 通过 `map_server/load_map` 加载裁剪后的地图
6. 通过 `/map` 话题和 RViz2 验证裁剪后的地图已生效

---

## 3. 实现思路

### 3.1 输入

- 全局地图：`/map`（`nav_msgs/msg/OccupancyGrid`）
- 机器人位姿：TF `map -> base_link`

### 3.2 处理流程

1. 订阅 `/map`，读取全局地图的：
   - `resolution`
   - `width`
   - `height`
   - `origin`
   - `data`

2. 通过 TF 查询机器人当前在 `map` 坐标系下的位置 `(x, y)`

3. 将世界坐标转换为地图栅格坐标：
       col = int((x - origin_x) / resolution)
       row = int((y - origin_y) / resolution)

4. 以机器人当前位置为中心，按固定窗口大小裁剪局部地图  
   本次最终有效参数为：
       crop_size_m = 8.0

5. 重新计算裁剪后子图的：
   - `width`
   - `height`
   - `origin`

6. 将裁剪结果保存为：
   - `cropped_map.pgm`
   - `cropped_map.yaml`

7. 调用 Nav2 的 `map_server/load_map` 服务加载裁剪图

8. 在 RViz2 中显示并验证

---

## 4. 两次裁剪结果对比

### 4.1 原始全局地图

全局地图参数为：
- resolution = 0.05 m/cell
- width = 384
- height = 384
- 实际尺寸约为 `19.2m × 19.2m`

### 4.2 第一次测试：裁剪窗口 20m

第一次裁剪后 `/map` 参数约为：
- width = 359
- height = 384
- origin = (-10.0, -10.0)

说明：
- 20m 的裁剪窗口接近甚至超过原始全图尺寸（19.2m）
- 裁剪窗口触碰到地图边界，导致边界裁切后实际仍接近整图
- 因此“局部图”效果不明显

### 4.3 第二次测试：裁剪窗口 8m

第二次裁剪后 `/map` 参数为：
- width = 158
- height = 158
- origin = (-5.949999939650297, -4.099999912083149)

换算后实际尺寸约为：
- `158 × 0.05 ≈ 7.9m`
- 即约 `7.9m × 7.9m`

说明：
- 此时裁剪窗口明显小于原始全图尺寸
- 裁剪后的地图已体现为局部子图
- 同时 `origin` 已发生变化，说明裁剪结果不是原图简单复用，而是以机器人当前位置附近区域生成了新的子图

---

## 5. 功能包位置

当前有效功能包路径为：

    ~/ros2_linux_30d_bootcamp/ros2_ws/src/local_map_cropper

主节点文件路径为：

    ~/ros2_linux_30d_bootcamp/ros2_ws/src/local_map_cropper/local_map_cropper/crop_map_node.py

说明：
- 该目录为当前实际参与编译和运行的 ROS2 Python 功能包
- 后续提交、交接、复现均以此目录为准

---

## 6. 裁剪结果文件位置

裁剪生成的地图文件当前位于：

    ~/ros2_linux_30d_bootcamp/data/local_map_cropper/

包含：

    ~/ros2_linux_30d_bootcamp/data/local_map_cropper/cropped_map.pgm
    ~/ros2_linux_30d_bootcamp/data/local_map_cropper/cropped_map.yaml

说明：
- `cropped_map.pgm` 为裁剪后的灰度地图图像
- `cropped_map.yaml` 为 Nav2 `map_server` 可直接加载的地图配置文件

---

## 7. 推荐日志归档路径

建议将本次任务相关日志统一归档到：

    ~/ros2_linux_30d_bootcamp/logs/local_map_cropper/

建议至少保留以下日志文件：

    run_crop_map.log
    load_map.log
    verify_map.log

其中：
- `run_crop_map.log`：裁剪节点运行日志
- `load_map.log`：调用 `map_server/load_map` 的返回日志
- `verify_map.log`：重新读取 `/map` 验证后的结果日志

---

## 8. 一键复现口径（默认 Nav2 仿真环境已启动）

### 8.1 编译功能包

    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select local_map_cropper
    source install/setup.bash

### 8.2 运行裁剪节点并保存日志

    mkdir -p ~/ros2_linux_30d_bootcamp/logs/local_map_cropper

    ros2 run local_map_cropper crop_map_node \
      2>&1 | tee ~/ros2_linux_30d_bootcamp/logs/local_map_cropper/run_crop_map.log

### 8.3 调用 map_server 加载裁剪后的地图并保存日志

    ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
      "{map_url: '/home/hexiaoyi/ros2_linux_30d_bootcamp/data/local_map_cropper/cropped_map.yaml'}" \
      2>&1 | tee ~/ros2_linux_30d_bootcamp/logs/local_map_cropper/load_map.log

### 8.4 验证当前 `/map` 是否已切换为局部图并保存日志

    ros2 topic echo /map --once \
      2>&1 | tee ~/ros2_linux_30d_bootcamp/logs/local_map_cropper/verify_map.log

### 8.5 RViz2 验证

- Fixed Frame 设为 `map`
- 查看 `Map` display
- 确认显示内容已变为裁剪后的局部地图

---

## 9. 最小复现前提

本功能包的复现依赖以下环境前提已满足：

1. Humble 环境已正确 `source`
2. Nav2 仿真环境已启动
3. `/map` 话题可正常获取全局地图
4. TF 中 `map -> base_link` 可正常查询
5. `map_server` 服务 `/map_server/load_map` 已可用

---

## 10. 当前结论

本次任务已完成一个最小且有效的工程闭环：

- 基于已有全局地图裁剪局部子图
- 生成 `pgm + yaml`
- 交由 Nav2 `map_server` 加载
- 通过 `/map` 与 RViz2 验证显示

当前最终有效裁剪结果约为：

- 地图尺寸：`158 × 158`
- 物理尺寸：约 `7.9m × 7.9m`
- origin：`(-5.95, -4.10)`

说明局部地图裁剪与加载逻辑已生效。

---

## 11. 后续可继续优化项

后续如需增强，可在当前功能包基础上继续补充：

1. 将“运行裁剪 + 调用 load_map + 验证”封装为单个脚本
2. 支持参数化裁剪尺寸
3. 支持机器人移动后重新裁剪并动态更新地图
4. 增加 launch 文件，形成一键启动闭环
5. 补充 README 与测试说明，方便交接和复用
