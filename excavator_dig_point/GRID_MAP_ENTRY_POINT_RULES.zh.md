# 基于 GridMap 的入铲点最小生成规则

## 1. 目标与系统边界

本阶段用于验证从高程图生成候选入铲点的最小流程。感知层只根据地形信息选择点，不负责规划挖掘机如何到达该点。

```text
输入：/terrain/grid_map
输出：候选入铲点、分数、置信度和有效状态
```

本阶段不输入：

- 挖掘机姿态；
- 铲尖位置或姿态；
- URDF 和挖掘机 TF；
- 关节状态和运动学信息。

本阶段不输出：

- 入铲姿态；
- 铲斗航向和攻击角；
- 挖掘轨迹；
- 关节或执行器指令。

因此，感知层输出的是“基于地形特征的候选入铲点”，不保证该点能够被挖掘机到达。候选点的可达性、碰撞和运动姿态由规划层判断。

## 2. GridMap 最小输入

最小算法需要以下图层：

| 图层 | 用途 |
|---|---|
| `ground_height` | 基准地面高程 |
| `surface_height` | 当前地表高程 |
| `confidence` | 栅格数据置信度 |

建议同时使用以下图层：

| 图层 | 用途 |
|---|---|
| `soil_height` | 已计算的土层厚度 |
| `slope` | 过滤过平或过陡区域 |
| `roughness` | 排除不稳定或噪声区域 |
| `point_count` | 排除点数不足的栅格 |

如果高程图中没有 `soil_height`，按下式计算：

```text
soil_height(x, y) = surface_height(x, y) - ground_height(x, y)
```

所有输出均继承输入 `GridMap.header` 的时间戳和坐标系。

## 3. 输入有效性检查

收到高程图后，依次检查：

1. `frame_id` 非空且符合系统坐标系约定；
2. 地图时间戳有效，地图没有超过允许时效；
3. 必需图层存在且尺寸一致；
4. 栅格分辨率和地图尺寸有效；
5. 有效栅格数量达到最低要求；
6. `surface_height` 和 `ground_height` 中没有大面积 NaN 或 Inf。

任一检查失败时，不生成候选点，发布：

```text
valid = false
status = 对应失败原因
```

## 4. 土堆掩膜生成

基础土堆掩膜定义为：

```text
pile_mask =
    finite(surface_height)
    AND finite(ground_height)
    AND soil_height >= minimum_soil_height
    AND confidence >= minimum_confidence
    AND point_count >= minimum_points_per_cell
```

其中高度阈值不应只凭经验固定。应在空场采集高程图并计算地面高度噪声标准差 `sigma_height`：

```text
minimum_soil_height = max(3 × sigma_height, 2 × map_resolution)
```

第一阶段可以使用下列值进行可视化测试，但必须根据实测噪声调整：

```yaml
minimum_soil_height: 0.15
minimum_confidence: 0.70
minimum_points_per_cell: 3
```

## 5. 掩膜清理和土堆提取

对二值土堆掩膜执行：

1. 开运算，删除孤立噪声；
2. 闭运算，填补土堆内部的小孔洞；
3. 连通域分析；
4. 删除面积过小的连通域；
5. 为保留的连通域分配稳定的 `pile_id`。

最小土堆面积需要大于局部评分窗口，初始可设置为：

```text
minimum_pile_area = 0.5 m²
```

该数值只用于初始验证，后续应按实际铲斗尺寸调整。

## 6. 候选点生成

由于本阶段没有挖掘机位置和姿态，算法不能区分土堆的近侧、远侧或最佳进入方向。因此候选点只根据土堆自身几何生成。

对于每个土堆连通域：

1. 计算该土堆最大土层高度 `maximum_soil_height`；
2. 计算每个栅格到土堆边界的距离；
3. 计算局部平均土层厚度；
4. 计算局部有效栅格比例；
5. 过滤不满足条件的栅格；
6. 将剩余栅格作为候选入铲点。

相对高度定义为：

```text
relative_height = soil_height / maximum_soil_height
```

最小验证建议保留：

```text
0.30 <= relative_height <= 0.60
10° <= slope <= 45°
distance_to_pile_edge >= 2～3 个栅格
local_valid_ratio >= 0.80
```

该规则用于避开土堆最高点、地面附近噪声和不稳定边缘。由于没有挖掘机方向，最终点可能位于土堆任意一侧，这是该最小方案的已知限制。

## 7. 局部特征计算

不要只使用单个栅格值。以候选点为中心，在半径 `local_window_radius` 内统计：

```text
local_soil_height = 有效栅格 soil_height 的均值
local_confidence = 有效栅格 confidence 的均值
local_roughness = 有效栅格 roughness 的均值
local_valid_ratio = 有效栅格数量 / 窗口总栅格数量
```

初始验证可以使用：

```yaml
local_window_radius: 0.30
minimum_local_valid_ratio: 0.80
```

## 8. 候选点评分

硬约束通过后，对候选点计算归一化分数：

```text
score =
    0.35 × local_soil_height_score
  + 0.25 × confidence_score
  + 0.20 × edge_clearance_score
  + 0.10 × relative_height_score
  - 0.10 × roughness_penalty
```

其中：

- `local_soil_height_score`：局部土层越厚，分数越高；
- `confidence_score`：观测置信度越高，分数越高；
- `edge_clearance_score`：距离土堆边缘越远，分数越高；
- `relative_height_score`：相对高度接近目标区间中心时分数越高；
- `roughness_penalty`：局部粗糙度越高，扣分越多。

各项必须归一化到 `[0, 1]`，最终分数限制在 `[0, 1]`。

## 9. 最终点生成

选择分数最高且超过最低分数阈值的候选点：

```text
x = 候选栅格中心的世界坐标 X
y = 候选栅格中心的世界坐标 Y
z = 该位置拟合或插值后的 surface_height
```

为了降低单栅格噪声，可在候选点附近使用 `3×3` 或 `5×5` 有效栅格拟合局部平面，并使用拟合平面高度作为最终 `z`。

如果没有候选点超过最低分数：

```text
valid = false
status = no_valid_candidate
```

## 10. 时间稳定性规则

为避免入铲点在相邻帧之间跳动，增加以下规则：

```text
候选点连续存在帧数 >= 3
相邻帧目标距离 <= 2 × map_resolution
新目标分数 >= 当前目标分数 + switch_score_margin 时才切换
```

如果当前目标仍然有效，应优先保持当前目标，而不是每帧重新选择分数略高的新点。

初始参数可以设置为：

```yaml
minimum_stable_frames: 3
maximum_target_jump_cells: 2
switch_score_margin: 0.10
```

## 11. 输出接口

### 11.1 最小验证接口

最简单的输出可以使用：

```text
geometry_msgs/msg/PointStamped
```

建议话题：

```text
/digging/entry_point
```

### 11.2 推荐接口

为了携带有效状态和评分，建议定义：

```text
std_msgs/Header header
geometry_msgs/Point point
float32 score
float32 confidence
uint32 pile_id
bool valid
string status
```

不建议在本阶段把未计算的姿态、攻击角和切削参数填入完整 `DigTarget`，否则下游可能把默认零值误认为有效规划输入。

## 12. RViz 调试输出

建议同时发布以下调试内容：

| 调试内容 | 建议形式 |
|---|---|
| 原始土堆掩膜 | GridMap 图层或 OccupancyGrid |
| 清理后的连通域 | GridMap 图层 |
| 所有候选点 | MarkerArray |
| 最终入铲点 | Sphere/Arrow Marker |
| 候选点分数 | Text Marker 或独立 GridMap 图层 |

RViz 中应能够同时观察高程图、土堆区域、候选点和最终结果。

## 13. 最小验证流程

```text
/terrain/grid_map
  -> 输入检查
  -> soil_height 计算
  -> 土堆掩膜
  -> 形态学清理
  -> 连通域提取
  -> 候选点生成
  -> 硬约束过滤
  -> 候选点评分
  -> 时间稳定处理
  -> /digging/entry_point
```

## 14. 验收标准

最小功能满足以下条件后，才进入规划层对接：

1. 空场高度图不生成有效入铲点；
2. 土堆出现后能够生成入铲点；
3. 入铲点始终位于清理后的土堆掩膜内部；
4. 入铲点 `z` 与局部拟合后的 `surface_height` 一致；
5. 静止土堆连续 100 帧中，点位抖动不超过 1～2 个栅格；
6. 少量 NaN、漏点和离群值不会导致目标大幅跳变；
7. 地图过期、图层缺失或有效栅格不足时输出 `valid=false`；
8. 土堆消失后不继续发布旧的有效目标；
9. 输出继承高度图的时间戳和坐标系；
10. RViz 能明确显示掩膜、候选点、最终点及失败原因。

## 15. 后续扩展

最小验证阶段只输出一个点。与规划层正式连接时，建议改为输出分数最高的前 `3～5` 个候选点，由规划层依次执行：

```text
可达性检查 -> 碰撞检查 -> 入铲姿态生成 -> 轨迹规划
```

如果规划层拒绝最高分候选点，可以继续尝试后续候选点，而不需要感知层立即重新处理整幅高度图。
