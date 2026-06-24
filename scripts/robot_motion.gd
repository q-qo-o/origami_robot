extends Node3D

## 折纸机器人折叠/展开运动控制器
## 控制 8 个 HingeJoint3D 实现循环折叠与展开
##
## 运动学约束回顾：
## - 8 个面片 face0~face7 通过 8 个 HingeJoint3D 构成闭合环
## - 完全展开时相邻面共面（二面角 δ = 0°）
## - 折叠时二面角 δ > 0，面片绕折痕（关节轴）相对旋转
## - 控制策略：位置伺服 + 速度限制，目标角度在折叠角度与 0 之间周期性变化


# ============================================================================
# 导出参数
# ============================================================================

## 运动周期（秒），一个完整折叠→展开→折叠循环
@export var cycle_period: float = 6.0

## 折叠深度：0 = 完全不折叠（保持展平），1 = 完全折叠到初始角度
@export var fold_depth: float = 1.0

## 位置控制增益（rad/s 每 rad 误差）
@export var position_gain: float = 2.0

## 阻尼增益（rad/s 每 rad/s 角速度）
@export var damping_gain: float = 0.5

## 最大输出角速度（rad/s）
@export var max_velocity: float = 2.0

## 马达最大冲量
@export var motor_max_impulse: float = 5.0

## 启用/暂停运动
@export var enable_motion: bool = true:
	set(value):
		enable_motion = value
		if value:
			_start_motors()
		else:
			_stop_motors()

## 物理稳定等待时间（秒），之后捕获初始角度作为折叠基准
@export var auto_capture_delay: float = 1.0


# ============================================================================
# 内部状态
# ============================================================================

var elapsed_time: float = 0.0
var joints: Array[HingeJoint3D] = []
var previous_angles: Array[float] = []
var initial_angles: Array[float] = []
var _initialized: bool = false
var _init_timer: float = 0.0


# ============================================================================
# 生命周期
# ============================================================================

func _ready():
	_collect_joints()
	previous_angles.resize(joints.size())
	previous_angles.fill(0.0)
	initial_angles.resize(joints.size())
	initial_angles.fill(0.0)

	if enable_motion and _motors_available():
		for joint in joints:
			if joint:
				joint.set("motor/enable", true)
				joint.set("motor/max_impulse", motor_max_impulse)
				joint.set("motor/target_velocity", 0.0)


func _physics_process(delta):
	if not enable_motion or joints.is_empty():
		return

	if not _motors_available():
		return

	# 阶段一：等待物理稳定，捕获初始角度
	if not _initialized:
		_init_timer += delta
		if _init_timer >= auto_capture_delay:
			_capture_initial_angles()
			_initialized = true
			print("[OrigamiMotion] Initial angles captured: ", _format_angles(initial_angles))
		return

	# 阶段二：周期运动控制
	elapsed_time += delta

	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue

		var current_angle = _get_joint_angle(joint)
		if not is_finite(current_angle):
			continue

		var angle_velocity = (current_angle - previous_angles[i]) / delta
		previous_angles[i] = current_angle

		var target_angle = _compute_target_angle(i, elapsed_time)
		if not is_finite(target_angle):
			continue

		# 位置伺服 + 阻尼
		var error = target_angle - current_angle
		var velocity_target = position_gain * error - damping_gain * angle_velocity

		# 速度饱和
		velocity_target = clampf(velocity_target, -max_velocity, max_velocity)

		joint.set("motor/target_velocity", velocity_target)


# ============================================================================
# 目标角度计算
# ============================================================================

## 计算第 i 个关节在时刻 t 的目标角度
##
## 一个周期内的运动规划：
##   0.00 ~ 0.35: 保持折叠 → 开始展开（ease-out）
##   0.35 ~ 0.50: 完全展开（停留）
##   0.50 ~ 0.85: 开始折叠（ease-in） → 回到折叠
##   0.85 ~ 1.00: 保持折叠（停留）
func _compute_target_angle(joint_index: int, t: float) -> float:
	var phase = t / cycle_period
	var cycle_t = phase - floor(phase)

	var fold_factor: float
	if cycle_t < 0.35:
		var u = cycle_t / 0.35
		fold_factor = 1.0 - _ease_out_cubic(u)
	elif cycle_t < 0.50:
		fold_factor = 0.0
	elif cycle_t < 0.85:
		var u = (cycle_t - 0.50) / 0.35
		fold_factor = _ease_in_cubic(u)
	else:
		fold_factor = 1.0

	var initial = initial_angles[joint_index]
	return initial * fold_factor * fold_depth


## ease-out cubic: t∈[0,1] → 1-(1-t)³
static func _ease_out_cubic(t: float) -> float:
	var u = 1.0 - clampf(t, 0.0, 1.0)
	return 1.0 - u * u * u


## ease-in cubic: t∈[0,1] → t³
static func _ease_in_cubic(t: float) -> float:
	var u = clampf(t, 0.0, 1.0)
	return u * u * u


# ============================================================================
# 关节角度测量
# ============================================================================

static func _get_joint_angle(joint: HingeJoint3D) -> float:
	var body_a = joint.get_node(joint.node_a) as RigidBody3D
	var body_b = joint.get_node(joint.node_b) as RigidBody3D
	if not body_a or not body_b:
		return 0.0

	var axis = joint.global_transform.basis.z.normalized()
	if not axis.is_normalized():
		return 0.0

	# 在世界坐标中建立垂直于 axis 的正交基 (u, v)
	var u = _get_perpendicular(axis)
	var v = axis.cross(u).normalized()

	# 将正交基转换到 body_a 和 body_b 的局部坐标系
	var basis_a_inv = body_a.global_transform.basis.inverse()
	var basis_b_inv = body_b.global_transform.basis.inverse()

	var u_a = basis_a_inv * u
	var v_a = basis_a_inv * v
	var u_b = basis_b_inv * u

	var x = u_a.dot(u_b)
	var y = v_a.dot(u_b)

	# 数值安全：确保 x²+y² 合理
	var mag_sq = x * x + y * y
	if mag_sq < 1e-12:
		return 0.0

	return atan2(y, x)


static func _get_perpendicular(v: Vector3) -> Vector3:
	var abs_x = abs(v.x)
	var abs_y = abs(v.y)
	var abs_z = abs(v.z)

	var ref: Vector3
	if abs_x <= abs_y and abs_x <= abs_z:
		ref = Vector3(1, 0, 0)
	elif abs_y <= abs_x and abs_y <= abs_z:
		ref = Vector3(0, 1, 0)
	else:
		ref = Vector3(0, 0, 1)

	var perp = ref.cross(v)
	if perp.length_squared() < 1e-8:
		if ref == Vector3(1, 0, 0):
			ref = Vector3(0, 1, 0)
		else:
			ref = Vector3(1, 0, 0)
		perp = ref.cross(v)

	return perp.normalized()


# ============================================================================
# 马达控制
# ============================================================================

static func _motors_available() -> bool:
	return DisplayServer.get_name() != "headless"


func _start_motors():
	if not _motors_available():
		return
	for joint in joints:
		if joint:
			joint.set("motor/enable", true)
			joint.set("motor/max_impulse", motor_max_impulse)


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


func _capture_initial_angles():
	for i in range(len(joints)):
		var angle = _get_joint_angle(joints[i])
		initial_angles[i] = angle
		previous_angles[i] = angle


# ============================================================================
# 辅助
# ============================================================================

static func _format_angles(angles: Array[float]) -> String:
	var parts: Array[String] = []
	for i in range(angles.size()):
		parts.append("j%d=%.3f°" % [i, rad_to_deg(angles[i])])
	return "[" + ", ".join(parts) + "]"


# ============================================================================
# 公共接口
# ============================================================================

func restart():
	elapsed_time = 0.0
	_initialized = false
	_init_timer = 0.0


func set_cycle_period(p: float):
	cycle_period = maxf(p, 0.5)
