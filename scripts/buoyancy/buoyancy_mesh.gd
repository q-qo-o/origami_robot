extends RigidBody3D
##
## 基于网格几何的浮力 / 水动力学脚本
## ============================================================
## 来源：参考 https://github.com/q-qo-o/godot-floating-objects 的 buoyancy_mesh.gd
## 算法原理：
##
##   1. 在 _ready() 时遍历所有子 MeshInstance3D，提取其三角形
##      顶点（局部坐标），并缓存每个 mesh 相对刚体的变换。
##
##   2. 每物理帧（_integrate_forces）：
##      a. 将三角形顶点变换到世界坐标系。
##      b. 用水面平面 y = water_height 对每个三角形做"水线裁剪"，
##         得到完全位于水面以下的子三角形集合。
##      c. 对每个水下三角形与水面参考点构造四面体，用
##           V = (1/6) * r0 · (r1 × r2)
##         计算其有向体积，按体积权重累加形心位置。
##      d. 在累加得到的浮心处施加 F = ρ·g·V_sub。
##
##   3. 可选：在刚体局部系下计算线性 + 二次阻尼力 / 力矩
##      （Fossen 水下航行器阻尼模型）。
##
## 相对原版增加的旋钮：
##   - buoyancy_multiplier：浮力倍率（默认 1.0 = 纯阿基米德浮力）。
##     折纸机器人是薄壳结构，其实际排开水的体积（外廓包围体积）远大于
##     网格实体体积，因此乘一个 >1 的倍率在物理上是合理的，可让机器人
##     带正浮力浮在水面。
##
## 使用要求：
##   - 节点本身必须是 RigidBody3D。
##   - 必须有至少一个子 MeshInstance3D 提供几何信息。
##   - 网格应当是封闭的且法向朝外（基础形状和大多数美术资产满足）。
##
## 场景结构示例：
##   FloatingBox (RigidBody3D, 挂载本脚本)
##   ├── CollisionShape3D       # 物理碰撞
##   └── MeshInstance3D         # 浮力几何
##

# ============================================================
# 导出参数
# ============================================================

## 水面节点。其全局 Y 坐标作为水面高度（支持移动水面）。
## 若为 null，则在 _ready 时尝试查找名为 "WaterSurface" 的节点；
## 若仍找不到，水面高度回退到 _FALLBACK_WATER_LEVEL（默认 0.0）。
@export var water_surface_node: Node3D

# 找不到水面节点时的回退高度（一般不会用到）
const _FALLBACK_WATER_LEVEL := 0.0

## 流体密度，水为 1000 kg/m³
@export var fluid_density := 1000.0

## 浮力倍率（默认 1.0 = 纯阿基米德浮力）。
## 折纸薄壳的外廓排开体积大于网格实体体积，可调大此值获得正浮力。
@export var buoyancy_multiplier := 1.0

## 基础线性阻尼（按浸没比例缩放）
## 简单近似：每物理步将线速度乘以 (1 - water_drag * step * submerged_ratio)
@export var water_drag := 0.6

## 基础角阻尼（按浸没比例缩放）
@export var water_angular_drag := 0.6

@export_group("水动力学阻尼 (Body Frame)")
## 线性阻尼系数（刚体局部系）— Forward / Lateral / Vertical
## 物理含义：F_lin = -D · v，单位 N/(m/s)
@export var linear_damping_translational := Vector3(0.0, 0.0, 0.0)

## 角速度线性阻尼系数（刚体局部系）— Roll / Pitch / Yaw
## 物理含义：T_lin = -D · ω，单位 N·m/(rad/s)
@export var linear_damping_rotational := Vector3(0.0, 0.0, 0.0)

## 平移二次阻尼系数（形状阻力）
## 物理含义：F_quad = -D · |v| · v，单位 N/(m/s)²
@export var quadratic_damping_translational := Vector3(0.0, 0.0, 0.0)

## 转动二次阻尼系数
## 物理含义：T_quad = -D · |ω| · ω，单位 N·m/(rad/s)²
@export var quadratic_damping_rotational := Vector3(0.0, 0.0, 0.0)

## 平移附加质量系数
## 物理含义：F_am = -M · a，单位 kg
## 经验值：对于漂浮长方体，大致在 0.05~0.3 倍物体质量之间
@export var added_mass_translational := Vector3(0.0, 0.0, 0.0)

## 转动附加质量系数（附加惯性）
## 物理含义：T_am = -I · α，单位 kg·m²
@export var added_mass_rotational := Vector3(0.0, 0.0, 0.0)

@export_group("调试")
## 是否在调试器中暴露内部状态（保留接口，目前仅记录数据）
@export var debug_draw := false


# ============================================================
# 内部状态
# ============================================================

## 重力加速度（从 ProjectSettings 读取）
var _gravity: float

## 当前浸没比例 [0, 1]，供外部脚本读取（例如 water_surface.gd 判断尾迹）
var submerged_ratio: float = 0.0

## 当前垂直速度，供外部读取
var vertical_velocity: float = 0.0

## 上一帧局部坐标系下的线速度（用于一阶差分计算加速度，附加质量项使用）
var _prev_lin_vel_body := Vector3.ZERO

## 上一帧局部坐标系下的角速度
var _prev_ang_vel_body := Vector3.ZERO

## 缓存的网格三角形数据
## 每个元素为 Dictionary：
##   {
##     "tris": Array[Vector3],   # 每 3 个连续顶点构成一个三角形（局部坐标）
##     "xform": Transform3D,     # mesh 相对刚体的变换（_ready 时计算并缓存）
##     "owner": Node3D           # 来源 MeshInstance3D（仅供调试）
##   }
var _mesh_triangles: Array = []

## 所有 mesh 的总体积（用于 submerged_ratio 归一化）
var _total_mesh_volume: float = 0.0

## 调试用：最近一帧的浮心和浮力大小
var _last_buoyancy_center_world: Vector3
var _last_total_force: float = 0.0


# ============================================================
# 初始化
# ============================================================

func _ready() -> void:
	# 加入 floating_bodies 组，便于 water_surface.gd 等脚本检索
	add_to_group("floating_bodies")
	_gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

	# 若未指定水面节点，尝试从当前场景查找名为 "WaterSurface" 的节点
	if water_surface_node == null:
		var current := get_tree().current_scene
		if current != null:
			water_surface_node = current.get_node_or_null("WaterSurface")
	if water_surface_node == null:
		push_warning("buoyancy_mesh.gd: 未找到水面节点，将使用回退水面高度 %f" % _FALLBACK_WATER_LEVEL)

	_collect_meshes()

	# 初始化附加质量历史状态
	_prev_lin_vel_body = Vector3.ZERO
	_prev_ang_vel_body = Vector3.ZERO


## 收集所有子 MeshInstance3D 的三角形，并计算总体积
func _collect_meshes() -> void:
	_mesh_triangles.clear()
	_total_mesh_volume = 0.0
	_collect_meshes_recursive(self)

	# 用四面体体积分计算每个 mesh 的总体积（封闭网格假设，法向朝外）
	for entry in _mesh_triangles:
		_total_mesh_volume += _compute_mesh_volume(entry.tris)

	# 防止除零（极端情况：网格退化或未找到任何 mesh）
	if _total_mesh_volume < 1e-6:
		_total_mesh_volume = 1.0


## 递归遍历子节点，收集 MeshInstance3D
func _collect_meshes_recursive(node: Node) -> void:
	if node is MeshInstance3D:
		var mi := node as MeshInstance3D
		if mi.mesh != null:
			var tris := _extract_triangles_from_mesh(mi.mesh)
			if tris.size() >= 3:
				# 计算 mesh 相对刚体（self）的变换
				# 注意：使用 global_transform 是为了正确处理多层嵌套节点
				var xform := self.global_transform.affine_inverse() * mi.global_transform
				_mesh_triangles.append({
					"tris": tris,
					"xform": xform,
					"owner": mi
				})

	for child in node.get_children():
		_collect_meshes_recursive(child)


## 从 Mesh 资源中提取所有三角形顶点
## 返回值：Array[Vector3]，每 3 个连续元素构成一个三角形
## 支持带索引（ARRAY_INDEX）和不带索引两种格式
func _extract_triangles_from_mesh(mesh: Mesh) -> Array:
	var result: Array = []

	for surface_idx in range(mesh.get_surface_count()):
		var arrays := mesh.surface_get_arrays(surface_idx)
		if arrays.is_empty():
			continue

		var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var indices: PackedInt32Array
		if arrays[Mesh.ARRAY_INDEX] != null:
			indices = arrays[Mesh.ARRAY_INDEX]
		else:
			indices = PackedInt32Array()

		if indices.size() > 0:
			# 带索引：每 3 个索引构成一个三角形
			for i in range(0, indices.size(), 3):
				if i + 2 < indices.size():
					result.append(verts[indices[i]])
					result.append(verts[indices[i + 1]])
					result.append(verts[indices[i + 2]])
		else:
			# 无索引：每 3 个连续顶点构成一个三角形
			for i in range(0, verts.size(), 3):
				if i + 2 < verts.size():
					result.append(verts[i])
					result.append(verts[i + 1])
					result.append(verts[i + 2])

	return result


## 计算封闭网格的总体积
## 原理：对每个三角形 (v0, v1, v2) 与原点构成四面体，
##       有向体积 V = (1/6) v0 · (v1 × v2)
##       若三角形法向朝外，所有四面体有向体积之和即为网格体积。
## 要求：网格封闭且法向朝外。
func _compute_mesh_volume(tris: Array) -> float:
	var vol := 0.0
	var i := 0
	while i + 2 < tris.size():
		var v0: Vector3 = tris[i]
		var v1: Vector3 = tris[i + 1]
		var v2: Vector3 = tris[i + 2]
		vol += v0.dot(v1.cross(v2)) / 6.0
		i += 3
	# 取绝对值以应对法向反向的情况
	return absf(vol)


# ============================================================
# 物理积分（每物理帧调用）
# ============================================================

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if _mesh_triangles.is_empty():
		return

	var water_height := _get_water_height()

	# 累加器：浸没体积（有向）和体积加权的浮心位置
	var total_submerged_volume := 0.0
	var weighted_centroid := Vector3.ZERO

	# 使用 state.transform 而非 global_transform，确保读到的是
	# 物理引擎本帧的最新位姿（state.transform == global_transform 的物理快照）
	var body_xform := state.transform

	# 遍历所有 mesh，再遍历所有三角形
	for entry in _mesh_triangles:
		var local_tris: Array = entry.tris
		var mesh_xform: Transform3D = entry.xform
		# 三角形顶点的世界变换 = body * mesh_local_to_body
		var world_xform := body_xform * mesh_xform

		var i := 0
		while i + 2 < local_tris.size():
			# 将三角形顶点变换到世界坐标
			var v0w := world_xform * (local_tris[i] as Vector3)
			var v1w := world_xform * (local_tris[i + 1] as Vector3)
			var v2w := world_xform * (local_tris[i + 2] as Vector3)

			# 用水面 y = water_height 裁剪三角形，得到水下部分
			# 返回 0、1 或 2 个新三角形
			var clipped := _clip_triangle_to_water(v0w, v1w, v2w, water_height)

			for tri in clipped:
				var t0: Vector3 = tri[0]
				var t1: Vector3 = tri[1]
				var t2: Vector3 = tri[2]

				# 用水面上一个参考点（取水面与 y 轴交点）与三角形构造四面体
				# 参考点的水平坐标取 0 不影响结果，因为后续做的是有向体积求和
				var ref := Vector3(0.0, water_height, 0.0)
				var r0 := t0 - ref
				var r1 := t1 - ref
				var r2 := t2 - ref

				# 四面体的有向体积（标量三重积）
				#   若三角形法向朝外（指向"水"），dv 为正
				#   若朝内，dv 为负
				# 对封闭网格而言，所有水下三角形的 dv 之和等于该网格
				# 浸没部分的体积（带符号）。
				var dv := r0.dot(r1.cross(r2)) / 6.0

				# 四面体的几何形心（四顶点平均）
				var dc := (ref + t0 + t1 + t2) / 4.0

				total_submerged_volume += dv
				weighted_centroid += dc * dv

			i += 3

	# 取绝对值：法向朝向不会改变物理结果，只改变 dv 的符号
	if absf(total_submerged_volume) > 1e-9:
		# 加权平均得到真实浮心（世界坐标）
		var centroid_world := weighted_centroid / total_submerged_volume
		var submerged_volume := absf(total_submerged_volume)
		submerged_ratio = clampf(submerged_volume / _total_mesh_volume, 0.0, 1.0)

		# 阿基米德浮力：F = ρ · g · V，方向竖直向上
		# 乘以 buoyancy_multiplier 以补偿薄壳外廓排开体积
		var buoyancy_force := Vector3.UP * submerged_volume * fluid_density * _gravity * buoyancy_multiplier
		_last_total_force = buoyancy_force.length()
		_last_buoyancy_center_world = centroid_world

		# state.apply_force(force, position) 的 position 参数是
		# 相对于刚体原点的偏移（刚体局部坐标系），所以需要将世界
		# 浮心位置变换回局部
		var local_pos := body_xform.affine_inverse() * centroid_world
		state.apply_force(buoyancy_force, local_pos)

		# 基础阻尼：按浸没比例缩放速度
		# 这是简化的指数衰减模型，物理上不严谨但稳定且易调
		state.linear_velocity *= 1.0 - water_drag * state.step * submerged_ratio
		state.angular_velocity *= 1.0 - water_angular_drag * state.step * submerged_ratio

		# 可选的物理准确阻尼模型
		_apply_hydrodynamic_damping(state)
	else:
		# 完全出水
		submerged_ratio = 0.0
		_last_total_force = 0.0

	# 更新公开状态供外部读取
	vertical_velocity = state.linear_velocity.y
	set_meta("submerged_ratio", submerged_ratio)
	set_meta("vertical_velocity", vertical_velocity)


# ============================================================
# 水动力学阻尼（Fossen 模型）
# ============================================================
##
## 在刚体局部坐标系下计算线性 + 二次阻尼，再变换回世界系施加。
##
## 与基础阻尼的区别：
##   - 基础阻尼是按浸没比例直接缩放速度（隐式积分），简单稳定
##   - 此处的水动力学阻尼按物理公式 F = -D·v - D·|v|·v 计算实际力，
##     允许各轴向使用不同系数（例如船只 forward 方向阻力小、
##     lateral 方向阻力大）
##
## 默认所有系数为 0，此函数实际不产生效果；
## 需要精细物理时可在导出参数中配置。
##
func _apply_hydrodynamic_damping(state: PhysicsDirectBodyState3D) -> void:
	# 几乎不浸没时跳过，避免微小数值噪声
	if submerged_ratio <= 0.001:
		# 出水后清零历史速度，下次入水重新初始化
		_prev_lin_vel_body = Vector3.ZERO
		_prev_ang_vel_body = Vector3.ZERO
		return

	var body_xform := state.transform
	var rot := body_xform.basis

	# 将世界系速度变换到刚体局部系
	# Body Frame: rot.transposed() * v_world == v_body
	var lin_vel_body := rot.transposed() * state.linear_velocity
	var ang_vel_body := rot.transposed() * state.angular_velocity

	# ========== 1. 线性阻尼 F = -D · v（逐分量） ==========
	var f_lin := -linear_damping_translational * lin_vel_body
	var t_lin := -linear_damping_rotational * ang_vel_body

	# ========== 2. 二次阻尼 F = -D · |v| · v ==========
	var f_quad := -quadratic_damping_translational * Vector3(
		absf(lin_vel_body.x) * lin_vel_body.x,
		absf(lin_vel_body.y) * lin_vel_body.y,
		absf(lin_vel_body.z) * lin_vel_body.z
	)
	var t_quad := -quadratic_damping_rotational * Vector3(
		absf(ang_vel_body.x) * ang_vel_body.x,
		absf(ang_vel_body.y) * ang_vel_body.y,
		absf(ang_vel_body.z) * ang_vel_body.z
	)

	# ========== 3. 附加质量 F_am = -M · a ==========
	# 物体加速运动时需要带动周围流体，等效产生反作用力。
	# 用一阶前向差分近似加速度：a ≈ (v_current - v_previous) / dt
	var dt := maxf(state.step, 1e-6)
	var lin_acc_body := (lin_vel_body - _prev_lin_vel_body) / dt
	var ang_acc_body := (ang_vel_body - _prev_ang_vel_body) / dt

	var f_am := -added_mass_translational * lin_acc_body
	var t_am := -added_mass_rotational * ang_acc_body

	# ========== 汇总并施加 ==========
	# 按浸没比例缩放（部分浸没时阻尼/附加质量按比例减小）
	var total_force_body := (f_lin + f_quad + f_am) * submerged_ratio
	var total_torque_body := (t_lin + t_quad + t_am) * submerged_ratio

	# 变换回世界系并施加于质心
	state.apply_central_force(rot * total_force_body)
	state.apply_torque(rot * total_torque_body)

	# 保存当前速度用于下一帧的加速度差分
	_prev_lin_vel_body = lin_vel_body
	_prev_ang_vel_body = ang_vel_body


# ============================================================
# 三角形 - 水面裁剪
# ============================================================
##
## 将世界坐标系下的三角形 (v0, v1, v2) 按水平面 y = h 裁剪，
## 返回水面以下部分（y < h）的三角形列表。
##
## 三个顶点相对水面的"在下方"个数：
##   0 → 完全在水面之上，返回 []
##   3 → 完全在水面之下，返回 [[v0, v1, v2]]
##   1 → 一个顶点在水下，水下区域是三角形 → 返回 1 个
##   2 → 两个顶点在水下，水下区域是四边形 → 拆成 2 个三角形返回
##
## 此函数对裁剪后三角形的法向保持原三角形的朝向。
##
func _clip_triangle_to_water(v0: Vector3, v1: Vector3, v2: Vector3, h: float) -> Array:
	var pts := [v0, v1, v2]
	var below := [v0.y < h, v1.y < h, v2.y < h]

	# 统计在水下的顶点数
	var count := 0
	for b in below:
		if b:
			count += 1

	# 完全在水面之上：无浮力贡献
	if count == 0:
		return []
	# 完全在水面之下：整个三角形参与计算
	if count == 3:
		return [[v0, v1, v2]]

	# 辅助 lambda：求线段 (a -> b) 与平面 y = h 的交点
	# 用线性插值参数 t = (h - a.y) / (b.y - a.y)
	var intersect := func(a: Vector3, b: Vector3) -> Vector3:
		var dy := b.y - a.y
		# 避免除零（三角形边几乎平行于水面）
		if absf(dy) < 1e-9:
			return a
		var t: float = (h - a.y) / dy
		return a + (b - a) * t

	if count == 1:
		# 一个顶点在水下：找到该顶点，与两条边交点构成新三角形
		var i := below.find(true)
		var p0: Vector3 = pts[i]            # 水下顶点
		var p1: Vector3 = pts[(i + 1) % 3]  # 水上顶点
		var p2: Vector3 = pts[(i + 2) % 3]  # 水上顶点
		# 注意顶点顺序保持原绕向（p0, p0->p1 交点, p0->p2 交点）
		return [[p0, intersect.call(p0, p1), intersect.call(p0, p2)]]

	# count == 2：两个顶点在水下，水下区域是四边形
	# 找到那个在水上的顶点
	var j := below.find(false)
	var q_above: Vector3 = pts[j]
	var q1: Vector3 = pts[(j + 1) % 3]  # 水下顶点
	var q2: Vector3 = pts[(j + 2) % 3]  # 水下顶点
	# 两条与水上顶点相连的边各产生一个交点
	var i1a: Vector3 = intersect.call(q1, q_above)
	var i2a: Vector3 = intersect.call(q2, q_above)
	# 四边形 (q1, q2, i2a, i1a) 拆分为两个三角形
	# 保持绕向一致以正确计算有向体积
	return [[q1, q2, i2a], [q1, i2a, i1a]]


# ============================================================
# 工具函数
# ============================================================

## 获取当前水面高度（优先使用 water_surface_node，否则用回退值）
func _get_water_height() -> float:
	if water_surface_node != null:
		return water_surface_node.global_position.y
	return _FALLBACK_WATER_LEVEL


## 公开接口：若运行时改变了子 MeshInstance3D 或 Mesh 资源，
## 可调用此方法刷新缓存
func refresh_meshes() -> void:
	_collect_meshes()

	# 初始化附加质量历史状态
	_prev_lin_vel_body = Vector3.ZERO
	_prev_ang_vel_body = Vector3.ZERO
