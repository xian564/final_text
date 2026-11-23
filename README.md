# my-project

## 装甲板尺寸与 PnP 约定

- 尺寸参数（`config/params.yaml`，单位米），来源考核文档：
  - 大装甲板：`armor_large_w=0.235`、`armor_large_h=0.127`（235×127 mm）
  - 小装甲板：`armor_small_w=0.120`、`armor_small_h=0.125`（120×125 mm）
- 代码位置：`src/detection_node_v3.cpp` 参数 63–66 行，热加载 272–275 行，PnP 使用 559–560 行。
- 顶点定义：取一对灯条外接框的 **TL → TR → BR → BL**（左上、右上、右下、左下），
  作为装甲板本体四角的近似；物体点按上述 Wb×bH 构造（米）。
- 装甲板坐标系：x 向右、y 向下、z 向前，右手系；单位为米。
- `header.frame_id` 使用 `camera_info.header.frame_id`（本数据为 `hk_camera_ca`）。
