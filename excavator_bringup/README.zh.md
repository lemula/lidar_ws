# excavator_bringup 分阶段启动

所有阶段默认启动各自的 RViz。追加 `rviz:=false` 可关闭 RViz。

```bash
ros2 launch excavator_bringup stage_01_lidar.launch.py
ros2 launch excavator_bringup stage_02_urdf.launch.py
ros2 launch excavator_bringup stage_03_filters.launch.py
ros2 launch excavator_bringup stage_04_ground.launch.py
ros2 launch excavator_bringup stage_05_height_map.launch.py
ros2 launch excavator_bringup stage_06_dig_point.launch.py
```

阶段 3 和阶段 5 默认使用同一裁剪区域：

```text
x = [-0.5, 1.0] m
y = [ 0.0, 1.5] m
z = [-2.0, 2.0] m
```

启动完整流水线时只打开最终入铲点 RViz，避免同时创建六个 RViz 窗口：

```bash
ros2 launch excavator_bringup pipeline.launch.py
```

完整流水线同样支持 `rviz:=false`，并可用 `start_lidar:=false` 等参数关闭指定阶段。

机体剔除默认启用。关闭机体剔除时仍保留裁剪，并把裁剪结果直接发布到
`/localmap/body_filtered_points`，后续地面分割、高度图和入铲点不需要改话题：

```bash
ros2 launch excavator_bringup stage_03_filters.launch.py enable_body_filter:=false
ros2 launch excavator_bringup pipeline.launch.py enable_body_filter:=false
```
