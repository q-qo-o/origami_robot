extends Node3D

## 折纸机器人折叠/展开运动控制器
##
## 两种工作模式：
## - 自动模式（auto）：周期性折叠/展开循环
## - 手动模式（manual）：通过8个拖动条直接控制各折痕二面角
##
## 核心原理：
## - 折痕是相邻面片共享的边，折痕方向 = HingeJoint3D 的旋转轴
## - 面片法线 = 两条折痕方向的叉积
## - 二面角 δ = 相邻面法线绕折痕的有符号夹角
## - δ = 0 表示共面（完全展开），|δ| > 0 表示折叠


# ============================================================================
# 导出参数
# ============================================================================

@export_group("运动模式")

## 手动模式开关。启用时显示8个拖动条，直接控制各折痕角度
@export var manual_mode: bool = true:
	set(value):
		manual_mode = value
		_toggle_manual_ui(value)

## 运动周期（秒），仅自动模式有效
@export var cycle_period: float = 6.0

## 折叠深度：0 = 完全展开，1 = 折叠到初始深度
@export var fold_depth: float = 1.0

@export_group("主折痕控制参数")

## 主折痕位置控制增益（rad/s 每 rad 误差）
## joint1, joint3, joint5, joint7 使用此参数
@export var primary_gain: float = 5.0

## 主折痕阻尼增益
@export var primary_damping: float = 1.0

## 主折痕积分增益（rad/s 每 rad·s 累积误差）
@export var primary_integral: float = 0.5

## 主折痕积分饱和限幅（rad·s）
@export var primary_integral_max: float = 2.0

## 主折痕最大角速度（rad/s）
@export var primary_max_velocity: float = 3.0

## 主折痕马达冲量
@export var primary_impulse: float = 8.0

@export_group("从折痕控制参数")

## 从折痕位置控制增益（rad/s 每 rad 误差）
## joint0, joint2, joint4, joint6 使用此参数
@export var secondary_gain: float = 5.0

## 从折痕阻尼增益
@export var secondary_damping: float = 1.0

## 从折痕积分增益（rad/s 每 rad·s 累积误差）
@export var secondary_integral: float = 0.5

## 从折痕积分饱和限幅（rad·s）
@export var secondary_integral_max: float = 2.0

## 从折痕最大角速度（rad/s）
@export var secondary_max_velocity: float = 3.0

## 从折痕马达冲量
@export var secondary_impulse: float = 8.0

# ============================================================================
# 主从折痕配置
# ============================================================================

## 主折痕索引（奇数索引关节）
const PRIMARY_JOINTS: Array[int] = [1, 3, 5, 7]


# ============================================================================
# 通用
# ============================================================================

## 启用/暂停运动
@export var enable_motion: bool = true:
	set(value):
		enable_motion = value
		if value:
			_start_motors()
		else:
			_stop_motors()

## 物理稳定等待时间（秒），之后捕获初始二面角
@export var auto_capture_delay: float = 1.0


# ============================================================================
# 内部状态
# ============================================================================

var elapsed_time: float = 0.0
var joints: Array[HingeJoint3D] = []
var previous_deltas: Array[float] = []
var initial_deltas: Array[float] = []
var _initialized: bool = false
var _init_timer: float = 0.0
var _normal_flips: Dictionary = {}
var _joint_local_offsets: Array[Transform3D] = []
var _velocity_signs: Array[int] = []
var _cached_normals: Dictionary = {}

# 手动模式状态
var _manual_targets: Array[float] = []       # 手动目标二面角（弧度）
var _manual_ui: CanvasLayer = null           # UI 根节点
var _manual_sliders: Array[HSlider] = []     # 8 个拖动条
var _manual_labels: Array[Label] = []        # 8 个标签

# PID 积分项
var _integral_terms: Array[float] = []       # 各关节误差积分累积（rad·s）
var _prev_errors: Array[float] = []          # 上一帧误差（用于抗饱和判断）

# 循环预设模式
var _cycle_preset_mode: bool = false         # 是否启用循环预设
var _cycle_preset_timer: float = 0.0         # 循环计时器
var _cycle_preset_is_x: bool = true          # 当前循环目标是否为 X 形

# ============================================================================
# 状态空间模态切换
# ============================================================================

## 状态空间：8个折痕二面角构成的 ℝ⁸ 向量
## 状态向量 q = [δ₀, δ₁, δ₂, δ₃, δ₄, δ₅, δ₆, δ₇]ᵀ
##
## 标准模态：
## - 默认(完全展开): q_default = [0, 0, 0, 0, 0, 0, 0, 0]ᵀ
## - X形折叠:       q_x      = [0, −π/2, 0, −π/2, 0, −π/2, 0, −π/2]ᵀ

## 模态切换状态
var _transition_active: bool = false        # 是否正在进行模态过渡
var _transition_start_q: Array[float] = []  # 过渡起始状态（8个角度）
var _transition_target_q: Array[float] = [] # 过渡目标状态
var _transition_t: float = 0.0              # 当前过渡时间（秒）
var _transition_T: float = 3.0              # 过渡总时长（秒）


## 默认状态（完全展开，所有二面角为0）
static func _state_default() -> Array[float]:
	return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


## X形折叠状态（从折痕−180°，主折痕−90°）
## 注意：−180°与+180°在物理上等价（wrap后统一），对应完全翻转
static func _state_x_fold() -> Array[float]:
	var q: Array[float] = []
	q.resize(8)
	for i in range(8):
		q[i] = -PI if (i % 2 == 0) else -PI / 2.0
	return q


## 路径归一化参数 s(t) = t/T - sin(2πt/T)/(2π)，ease-in-out 启停
## ṡ(t) = (1 - cos(2πt/T)) / T
static func _path_s(t: float, T: float) -> float:
	if t <= 0.0:
		return 0.0
	if t >= T:
		return 1.0
	return t / T - sin(2.0 * PI * t / T) / (2.0 * PI)


static func _path_s_dot(t: float, T: float) -> float:
	if t <= 0.0 or t >= T:
		return 0.0
	return (1.0 - cos(2.0 * PI * t / T)) / T


## 启动模态过渡：从当前状态平滑插值到目标状态
func start_transition(target_q: Array[float], duration: float = 3.0) -> void:
	_transition_start_q.resize(8)
	_transition_target_q.resize(8)
	for i in range(8):
		if manual_mode and _initialized and i < len(_manual_targets):
			# 手动模式：以用户 Slider 上看到的 _manual_targets 作为起点。
			# 物理实际状态（previous_deltas）与 _manual_targets 之间常有跟踪
			# 误差；若用物理测量值作起点，Slider 会跳变到测量值，用户体验差。
			_transition_start_q[i] = _manual_targets[i]
		elif _initialized and i < len(previous_deltas):
			# 自动模式：以物理测量值作为起点
			_transition_start_q[i] = previous_deltas[i]
		else:
			_transition_start_q[i] = initial_deltas[i]
		_transition_target_q[i] = target_q[i]
	_transition_t = 0.0
	_transition_T = maxf(duration, 0.5)
	_transition_active = true
	_clear_integral()
	print("[OrigamiMotion] Start transition, duration=%.1fs" % _transition_T)
	print("[OrigamiMotion]   from: ", _format_angles(_transition_start_q))
	print("[OrigamiMotion]   to:   ", _format_angles(_transition_target_q))


## 最短角度差：结果在 [−π, π]，边界（|diff|=π）时优先向 0 靠拢
static func _angle_diff(to: float, from: float) -> float:
	var diff = wrapf(to - from, -PI, PI)
	# 当 |diff| 恰好为 π 时，wrapf 可能返回 −π 或 +π（取决于实现）。
	# 此时选择让 from + diff 向 0 靠近的方向，路径更自然。
	if absf(absf(diff) - PI) < 1e-6:
		if from < 0:
			diff = PI
		elif from > 0:
			diff = -PI
	return diff


## 获取路径插值目标状态（不修改 _transition_t），返回 {"angles": [...], "velocities": [...]}
func _get_transition_target() -> Dictionary:
	var s := _path_s(_transition_t, _transition_T)
	var s_dot := _path_s_dot(_transition_t, _transition_T)

	var target_angles: Array[float] = []
	target_angles.resize(8)
	var target_velocities: Array[float] = []
	target_velocities.resize(8)
	for i in range(8):
		var delta_q = _angle_diff(_transition_target_q[i], _transition_start_q[i])
		# 位置：q(t) = q_A + shortest_diff(q_B, q_A) * s(t)，结果 wrap 到 [−π, π]
		target_angles[i] = wrapf(_transition_start_q[i] + delta_q * s, -PI, PI)
		# 路径速度：q̇(t) = shortest_diff * ṡ(t)
		target_velocities[i] = delta_q * s_dot

	return {"angles": target_angles, "velocities": target_velocities}


# ============================================================================
# 生命周期
# ============================================================================

func _ready():
	_collect_joints()
	previous_deltas.resize(joints.size())
	previous_deltas.fill(0.0)
	initial_deltas.resize(joints.size())
	initial_deltas.fill(0.0)
	_velocity_signs.resize(joints.size())
	_velocity_signs.fill(1)
	_manual_targets.resize(joints.size())
	_manual_targets.fill(0.0)
	_integral_terms.resize(joints.size())
	_integral_terms.fill(0.0)
	_prev_errors.resize(joints.size())
	_prev_errors.fill(0.0)

	_joint_local_offsets.resize(joints.size())
	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue
		var body_a = joint.get_node(joint.node_a) as RigidBody3D
		if body_a:
			_joint_local_offsets[i] = body_a.global_transform.affine_inverse() * joint.global_transform

	if enable_motion and _motors_available():
		for i in range(len(joints)):
			var joint = joints[i]
			if not joint:
				continue
			var impulse = primary_impulse if (i in PRIMARY_JOINTS) else secondary_impulse
			joint.set("motor/enable", true)
			joint.set("motor/max_impulse", impulse)
			joint.set("motor/target_velocity", 0.0)

	# 创建手动控制 UI（初始状态根据 manual_mode 显示/隐藏）
	_create_manual_ui()
	_toggle_manual_ui(manual_mode)


func _physics_process(delta):
	if not enable_motion or joints.is_empty():
		return

	if not _motors_available():
		return

	if delta <= 0.0:
		return

	# 阶段一：等待物理稳定，捕获初始二面角
	if not _initialized:
		_init_timer += delta
		if _init_timer >= auto_capture_delay:
			_align_normals()
			_capture_initial_deltas()
			_initialized = true
			print("[OrigamiMotion] Initial dihedral angles: ", _format_angles(initial_deltas))
			print("[OrigamiMotion] Velocity signs: ", _velocity_signs)
			# 初始化手动目标为 0（完全展开）
			for i in range(len(joints)):
				_manual_targets[i] = 0.0
				_update_slider_from_target(i)
		return

	# 阶段二：同步所有 joint 的 transform 跟随其 parent body
	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue
		var body_a = joint.get_node(joint.node_a) as RigidBody3D
		if body_a:
			joint.global_transform = body_a.global_transform * _joint_local_offsets[i]

	# 阶段三：循环预设模式 — 使用模态过渡平滑切换
	if _cycle_preset_mode and not _transition_active:
		_cycle_preset_timer += delta
		if _cycle_preset_timer >= 5.0:
			_cycle_preset_timer = 0.0
			_cycle_preset_is_x = not _cycle_preset_is_x
			var target = _state_x_fold() if _cycle_preset_is_x else _state_default()
			start_transition(target, 2.0)

	# 阶段四：统一目标值更新 — _manual_targets 是 PID 的唯一目标来源
	# 过渡模式：_manual_targets 跟随路径规划值
	# 自动模式：_manual_targets 跟随周期性规划值
	# 手动模式：_manual_targets 保持用户 Slider 设定值
	var path_velocities: Array[float] = []
	path_velocities.resize(len(joints))
	path_velocities.fill(0.0)
	if _transition_active:
		var path_result = _get_transition_target()
		var transition_angles: Array[float] = path_result["angles"]
		var transition_velocities: Array[float] = path_result["velocities"]
		for i in range(len(joints)):
			# 保证 _manual_targets 始终在 [−π, π]，避免 Slider 显示 −360° 等异常值
			_manual_targets[i] = wrapf(transition_angles[i], -PI, PI)
			path_velocities[i] = transition_velocities[i]
			_update_slider_from_target(i)
	elif not manual_mode:
		# 自动模式：将周期性目标写入 _manual_targets（统一来源）
		for i in range(len(joints)):
			_manual_targets[i] = wrapf(_compute_auto_target_delta(i, elapsed_time), -PI, PI)
			_update_slider_from_target(i)

	# 阶段五：所有 8 个关节统一 PD + 前馈控制
	# PID 目标永远是 _manual_targets[i]，来源不再分支
	elapsed_time += delta

	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue

		var current_delta = _get_dihedral_angle(joint)
		if not is_finite(current_delta):
			continue

		var delta_diff = wrapf(current_delta - previous_deltas[i], -PI, PI)
		var delta_dot = delta_diff / delta
		previous_deltas[i] = current_delta

		# PID 目标：统一来自 _manual_targets（手动/自动/过渡三种模式在此统一）
		var target_delta: float = _manual_targets[i]
		var path_velocity: float = path_velocities[i]

		if not is_finite(target_delta):
			continue

		var is_primary = (i in PRIMARY_JOINTS)
		var kp = primary_gain if is_primary else secondary_gain
		var kd = primary_damping if is_primary else secondary_damping
		var ki = primary_integral if is_primary else secondary_integral
		var ki_max = primary_integral_max if is_primary else secondary_integral_max
		var v_max = primary_max_velocity if is_primary else secondary_max_velocity

		var error = wrapf(target_delta - current_delta, -PI, PI)

		# 积分项更新（带抗饱和：仅在输出未饱和且误差同向时累积）
		var saturated = absf(_prev_errors[i]) > 0.01 and absf(error) > absf(_prev_errors[i])
		if not saturated:
			_integral_terms[i] += error * delta
		_integral_terms[i] = clampf(_integral_terms[i], -ki_max, ki_max)

		# PD 反馈 + 路径速度前馈：ω = sign·(kp·e + ki·∫e − kd·δ̇ + δ̇_path)
		var velocity_target = _velocity_signs[i] * (kp * error + ki * _integral_terms[i] - kd * delta_dot + path_velocity)
		velocity_target = clampf(velocity_target, -v_max, v_max)

		_prev_errors[i] = error
		joint.set("motor/target_velocity", velocity_target)

	# 阶段六：累加过渡时间并检测完成
	if _transition_active:
		_transition_t += delta
		if _transition_t >= _transition_T:
			_transition_active = false
			print("[OrigamiMotion] Transition completed")


# ============================================================================
# 手动控制 UI
# ============================================================================

func _create_manual_ui():
	_manual_ui = CanvasLayer.new()
	_manual_ui.name = "ManualUI"
	add_child(_manual_ui)

	var panel = Panel.new()
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.position = Vector2(20, 20)
	panel.size = Vector2(280, 360)
	panel.modulate = Color(1, 1, 1, 0.85)
	_manual_ui.add_child(panel)

	var vbox = VBoxContainer.new()
	vbox.position = Vector2(10, 10)
	vbox.size = Vector2(260, 340)
	vbox.add_theme_constant_override("separation", 8)
	panel.add_child(vbox)

	var title = Label.new()
	title.text = "折纸机器人手动控制"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(title)

	var mode_label = Label.new()
	mode_label.text = "模式: " + ("手动" if manual_mode else "自动")
	mode_label.name = "ModeLabel"
	mode_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(mode_label)

	var sep = HSeparator.new()
	vbox.add_child(sep)

	_manual_sliders.resize(8)
	_manual_labels.resize(8)

	for i in range(8):
		var hbox = HBoxContainer.new()
		hbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		vbox.add_child(hbox)

		var name_label = Label.new()
		name_label.text = "J%d" % i
		name_label.custom_minimum_size = Vector2(30, 0)
		hbox.add_child(name_label)

		var slider = HSlider.new()
		slider.min_value = -180.0
		slider.max_value = 180.0
		slider.step = 1.0
		slider.value = 0.0
		slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		slider.value_changed.connect(_on_slider_changed.bind(i))
		hbox.add_child(slider)
		_manual_sliders[i] = slider

		var val_label = Label.new()
		val_label.text = "  0°"
		val_label.custom_minimum_size = Vector2(50, 0)
		val_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		hbox.add_child(val_label)
		_manual_labels[i] = val_label

	var btn_sep = HSeparator.new()
	vbox.add_child(btn_sep)

	var btn_hbox = HBoxContainer.new()
	btn_hbox.alignment = BoxContainer.ALIGNMENT_CENTER
	vbox.add_child(btn_hbox)

	var reset_btn = Button.new()
	reset_btn.text = "重置"
	reset_btn.pressed.connect(_on_reset_pressed)
	btn_hbox.add_child(reset_btn)

	var preset_btn = Button.new()
	preset_btn.text = "X形折叠"
	preset_btn.pressed.connect(_on_preset_pressed)
	btn_hbox.add_child(preset_btn)

	var auto_btn = Button.new()
	auto_btn.text = "循环模式"
	auto_btn.pressed.connect(_on_cycle_preset_pressed)
	btn_hbox.add_child(auto_btn)


func _toggle_manual_ui(show_ui: bool):
	if _manual_ui:
		_manual_ui.visible = show_ui

	var mode_label = _manual_ui.get_node_or_null("Panel/VBoxContainer/ModeLabel") if _manual_ui else null
	if mode_label:
		mode_label.text = "模式: " + ("手动" if show_ui else "自动")

	# 切换到手动模式时，同步 Slider 为当前实际角度
	if show_ui and _initialized:
		for i in range(len(joints)):
			var v = previous_deltas[i] if i < len(previous_deltas) else initial_deltas[i]
			_manual_targets[i] = wrapf(v, -PI, PI)
			_update_slider_from_target(i)


func _update_slider_from_target(joint_index: int):
	if joint_index >= _manual_sliders.size():
		return
	var slider = _manual_sliders[joint_index]
	var label = _manual_labels[joint_index]
	var deg = rad_to_deg(_manual_targets[joint_index])
	slider.set_value_no_signal(deg)
	label.text = "%5.0f°" % deg


func _on_slider_changed(value: float, joint_index: int):
	_manual_targets[joint_index] = deg_to_rad(value)
	if joint_index < _manual_labels.size():
		_manual_labels[joint_index].text = "%5.0f°" % value


func _on_reset_pressed():
	_cycle_preset_mode = false
	start_transition(_state_default(), 3.0)


func _on_preset_pressed():
	_cycle_preset_mode = false
	start_transition(_state_x_fold(), 3.0)


func _on_cycle_preset_pressed():
	_cycle_preset_mode = true
	_cycle_preset_timer = 0.0
	_cycle_preset_is_x = true
	start_transition(_state_x_fold(), 2.0)


## 应用预设：is_x=true → X形折叠，is_x=false → 默认展开(0°)
func _apply_preset(is_x: bool) -> void:
	for i in range(len(joints)):
		if is_x:
			_manual_targets[i] = deg_to_rad(-180.0) if (i % 2 == 0) else deg_to_rad(-90.0)
		else:
			_manual_targets[i] = 0.0
		_integral_terms[i] = 0.0
		_prev_errors[i] = 0.0
		_update_slider_from_target(i)


func _on_auto_pressed():
	manual_mode = false


func _clear_integral():
	_integral_terms.fill(0.0)
	_prev_errors.fill(0.0)


# ============================================================================
# 目标二面角计算（自动模式）
# ============================================================================

func _compute_auto_target_delta(joint_index: int, t: float) -> float:
	var phase = t / cycle_period
	var cycle_t = phase - floor(phase)

	var fold_factor: float
	if cycle_t < 0.30:
		fold_factor = 1.0
	elif cycle_t < 0.40:
		var u = (cycle_t - 0.30) / 0.10
		fold_factor = 1.0 - 0.5 * _ease_out_cubic(u)
	elif cycle_t < 0.60:
		fold_factor = 0.5
	elif cycle_t < 0.70:
		var u = (cycle_t - 0.60) / 0.10
		fold_factor = 0.5 + 0.5 * _ease_in_cubic(u)
	else:
		fold_factor = 1.0

	return initial_deltas[joint_index] * fold_factor * fold_depth


static func _ease_out_cubic(t: float) -> float:
	var u = 1.0 - clampf(t, 0.0, 1.0)
	return 1.0 - u * u * u


static func _ease_in_cubic(t: float) -> float:
	var u = clampf(t, 0.0, 1.0)
	return u * u * u


# ============================================================================
# 法线对齐
# ============================================================================

func _align_normals():
	var bodies: Array[RigidBody3D] = []
	for joint in joints:
		var ba = joint.get_node(joint.node_a) as RigidBody3D
		var bb = joint.get_node(joint.node_b) as RigidBody3D
		if ba and not (ba in bodies):
			bodies.append(ba)
		if bb and not (bb in bodies):
			bodies.append(bb)

	if bodies.is_empty():
		return

	var center = Vector3.ZERO
	for body in bodies:
		center += body.global_position
	center /= bodies.size()

	_normal_flips.clear()
	_cached_normals.clear()
	for body in bodies:
		var normal_local = _compute_face_normal_local(body)
		_cached_normals[body] = normal_local
		var normal_world = body.global_transform.basis * normal_local
		var outward = (body.global_position - center).normalized()
		_normal_flips[body] = normal_world.dot(outward) < 0.0


func _compute_face_normal_local(body: RigidBody3D) -> Vector3:
	var parent = body.get_parent()
	var body_name = body.name

	var face_idx = -1
	if body_name.begins_with("face"):
		face_idx = int(body_name.substr(4))

	if face_idx < 0:
		return Vector3.UP

	var joint_a = parent.get_node_or_null("joint" + str(face_idx)) as HingeJoint3D
	var joint_b = parent.get_node_or_null("joint" + str(wrapi(face_idx - 1, 0, 8))) as HingeJoint3D

	if not joint_a or not joint_b:
		return Vector3.UP

	var body_basis_inv = body.global_transform.basis.inverse()
	var axis_a_local = body_basis_inv * joint_a.global_transform.basis.z.normalized()
	var axis_b_local = body_basis_inv * joint_b.global_transform.basis.z.normalized()

	var cross = axis_a_local.cross(axis_b_local)
	if cross.length_squared() < 1e-8:
		return Vector3.UP

	return cross.normalized()


func _get_face_normal_world(body: RigidBody3D) -> Vector3:
	var normal_local = _cached_normals.get(body, Vector3.UP)
	var normal_world = body.global_transform.basis * normal_local
	if _normal_flips.get(body, false):
		normal_world = -normal_world
	return normal_world.normalized()


# ============================================================================
# 二面角测量
# ============================================================================

func _get_dihedral_angle(joint: HingeJoint3D) -> float:
	var body_a = joint.get_node(joint.node_a) as RigidBody3D
	var body_b = joint.get_node(joint.node_b) as RigidBody3D
	if not body_a or not body_b:
		return 0.0

	var axis = joint.global_transform.basis.z.normalized()
	if not axis.is_normalized():
		return 0.0

	var n_a = _get_face_normal_world(body_a)
	var n_b = _get_face_normal_world(body_b)

	if not n_a.is_normalized() or not n_b.is_normalized():
		return 0.0

	var cross = n_a.cross(n_b)
	var sin_val = axis.dot(cross)
	var cos_val = n_a.dot(n_b)

	var mag_sq = sin_val * sin_val + cos_val * cos_val
	if mag_sq < 1e-6:
		return 0.0

	return atan2(sin_val, cos_val)


func _compute_velocity_sign(joint: HingeJoint3D) -> int:
	var body_a = joint.get_node(joint.node_a) as RigidBody3D
	var body_b = joint.get_node(joint.node_b) as RigidBody3D
	if not body_a or not body_b:
		return 1

	var axis = joint.global_transform.basis.z.normalized()
	var n_a = _get_face_normal_world(body_a)
	var n_b = _get_face_normal_world(body_b)

	if not n_a.is_normalized() or not n_b.is_normalized():
		return 1

	var sin_val = axis.dot(n_a.cross(n_b))
	var cos_val = n_a.dot(n_b)
	var delta = atan2(sin_val, cos_val)

	var epsilon = 0.05
	var n_b_rot = n_b * cos(epsilon) + axis.cross(n_b) * sin(epsilon)
	n_b_rot = n_b_rot.normalized()

	var sin_val_rot = axis.dot(n_a.cross(n_b_rot))
	var cos_val_rot = n_a.dot(n_b_rot)
	var delta_rot = atan2(sin_val_rot, cos_val_rot)

	var delta_diff = wrapf(delta_rot - delta, -PI, PI)

	if delta_diff > 0:
		return 1
	else:
		return -1


# ============================================================================
# 马达控制
# ============================================================================

static func _motors_available() -> bool:
	return DisplayServer.get_name() != "headless"


func _start_motors():
	if not _motors_available():
		return
	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue
		var impulse = primary_impulse if (i in PRIMARY_JOINTS) else secondary_impulse
		joint.set("motor/enable", true)
		joint.set("motor/max_impulse", impulse)


func _stop_motors():
	if not _motors_available():
		return
	for joint in joints:
		if joint:
			joint.set("motor/enable", false)
			joint.set("motor/target_velocity", 0.0)
			joint.set("motor/max_impulse", 0.0)


# ============================================================================
# 初始化
# ============================================================================

func _collect_joints():
	joints.clear()
	for i in range(8):
		var j = get_node_or_null("../joint" + str(i)) as HingeJoint3D
		if j:
			joints.append(j)
		else:
			push_warning("[OrigamiMotion] Joint joint" + str(i) + " not found")


func _capture_initial_deltas():
	for i in range(len(joints)):
		var delta_angle = _get_dihedral_angle(joints[i])
		initial_deltas[i] = delta_angle
		previous_deltas[i] = delta_angle
		_velocity_signs[i] = _compute_velocity_sign(joints[i])
	print("[OrigamiMotion] Velocity signs: ", _velocity_signs)


# ============================================================================
# 辅助
# ============================================================================

static func _format_angles(angles: Array[float]) -> String:
	var parts: Array[String] = []
	for i in range(angles.size()):
		parts.append("j%d=%.2f°" % [i, rad_to_deg(angles[i])])
	return "[" + ", ".join(parts) + "]"


# ============================================================================
# 公共接口
# ============================================================================

func restart():
	elapsed_time = 0.0
	_initialized = false
	_init_timer = 0.0
	_clear_integral()


func set_cycle_period(p: float):
	cycle_period = maxf(p, 0.5)


func set_manual_target(joint_index: int, target_deg: float):
	if joint_index >= 0 and joint_index < _manual_targets.size():
		_manual_targets[joint_index] = deg_to_rad(target_deg)
		_update_slider_from_target(joint_index)
