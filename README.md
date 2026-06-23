# 折纸机器人仿真

基于 Godot 4.6 的八面折纸机器人水动力仿真项目。

---

## 项目简介

本项目通过 [ACDC4Robot](https://github.com/q-qo-o/ACDC4Robot) 工具链生成折纸机器人模型，导出为 SDF（Simulation Description Format）格式，并在 Godot 4.6 中构建物理仿真场景。

**核心特性：**
- 8 面闭合运动链
- 网格几何浮力计算（水面三角面裁剪）
- 正弦波关节驱动
- SDF → Godot 自动导入
- 视频录制导出

---

## 项目结构

```
origami-robot/
├── project.godot                     # Godot 4.6 项目文件
├── scenes/
│   ├── origami_robot.tscn            # 主场景：8 个 RigidBody3D + 8 个 HingeJoint3D
│   └── ocean.tscn                    # 外场景：水面 + 光照 + 相机
├── scripts/
│   ├── robot_setup.gd                # @tool 脚本：STL → ArrayMesh（视觉 + 碰撞）
│   ├── robot_motion.gd               # 电机控制器：正弦波驱动
│   ├── buoyancy/
│   │   └── buoyancy_mesh.gd          # 网格浮力计算器
│   └── sdf_loader.gd                 # 运行时 SDF 导入器（备用）
├── addons/sdf_importer/              # 编辑器插件：SDF → .tscn 导入
│   ├── plugin.gd
│   ├── sdf_parser.gd
│   ├── stl_parser.gd
│   └── sdf_importer.gd
└── sdf/origami_robot/
    ├── origami_robot.sdf             # SDF 1.7 模型定义
    ├── model.config                  # ACDC4Robot 元数据
    ├── origami_robot.tscn            # SDF 导入器自动生成
    └── meshes/
        └── face{0-7}.stl             # 二进制 STL 文件（每个 8 个三角面）
```

---

## 场景架构

### 主场景（`scenes/origami_robot.tscn`）

| 组件 | 数量 | 说明 |
|------|------|------|
| `RigidBody3D` | 8 | `face0`–`face7`，每个质量 1.325 kg |
| `HingeJoint3D` | 8 | 闭合链：`face0→…→face7→face0` |
| `MotionController` | 1 | 所有关节的正弦速度曲线 |

- **视觉**：`MeshInstance3D` 通过 `STLParser` 从二进制 STL 填充，每个面有独立颜色
- **碰撞**：`ConvexPolygonShape3D`，通过 `ArrayMesh.create_convex_shape()` 生成
- **浮力**：`buoyancy_mesh.gd` 将三角面沿水面 `y = 0` 裁剪后计算阿基米德力

### 外场景（`scenes/ocean.tscn`）

- **WaterSurface** — 半透明平面，位于 `y = 0`
- **Ground** — `StaticBody3D`，位于 `y = -0.05`，用于着陆
- **DirectionalLight3D** — 带阴影的平行光
- **Camera3D** — 位于 `y ≈ 1.0`，俯视角度

---

## 坐标转换

SDF → Godot：

| 轴 | SDF | → | Godot |
|------|-----|---|-------|
| X | X | → | X |
| Y | Y | → | -Z |
| Z | Z | → | Y |

基矩阵：`Basis(Vector3(1,0,0), Vector3(0,0,-1), Vector3(0,1,0))`

---

## 视频导出

```bash
# 1. 渲染为 AVI（例如 12 秒 @ 60 FPS = 720 帧）
# 不要使用 --headless — Movie Maker 需要真实显示驱动
GODOT="/c/Users/qdsyq/MyApps/Godot/Godot_v4.6.3-stable_mono_win64_console.exe"
mkdir -p output/videos
"$GODOT" --path . --write-movie output/videos/ocean_temp.avi --quit-after 720

# 2. 转码为 MP4
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
ffmpeg -y -i output/videos/ocean_temp.avi \
  -c:v libx264 -preset medium -crf 18 \
  -pix_fmt yuv420p -movflags +faststart \
  "output/videos/ocean_${TIMESTAMP}.mp4"

# 3. 清理临时文件
rm output/videos/ocean_temp.avi
```

---

## 开发命令

```bash
# 在 Godot 编辑器中打开
"$GODOT" --editor --path .

# 运行项目（默认场景：ocean.tscn）
"$GODOT" --path .

# 无头模式导入 / 验证（重新生成 .uid 文件）
"$GODOT" --headless --import --path .

# 构建 C#（如果添加 C# 脚本）
dotnet build
```

---

## 关键技术细节

- **物理引擎**：Jolt Physics
- **渲染管线**：Forward Plus，d3d12
- **网格缩放**：0.001（SDF 默认值，毫米级建模）
- **材质剔除模式**：`CULL_DISABLED` — 折纸薄壳两面可见
- **电机 API**：`joint.set("motor/target_velocity", val)` — Godot 4.6 嵌套属性路径
- **无头模式**：当 `DisplayServer.get_name() == "headless"` 时，HingeJoint3D 电机自动禁用

---

## 致谢

- 机器人模型由 [ACDC4Robot](https://github.com/q-qo-o/ACDC4Robot) 生成
- 浮力算法移植自 [q-qo-o/godot-floating-objects](https://github.com/q-qo-o/godot-floating-objects)

## 许可

MIT
