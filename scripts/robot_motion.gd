extends Node3D

## 折纸机器人运动控制器
## 驱动 8 个 HingeJoint3D 的马达，产生波浪形爬行运动

## 每个关节的马达目标速度振幅（°/s）
@export var speed_amplitude: float = 180.0

## 运动周期（秒）
@export var period: float = 2.0

## 相位偏移（0~1），相邻关节之间的相位差
@export var phase_offset: float = 0.125

## 启用/暂停运动
@export var enable_motion: bool = true:
	set(value):
		enable_motion = value
		if value:
			_start_motors()
		else:
			_stop_motors()

## 马达最大冲量（越大力量越大）
@export var motor_max_impulse: float = 10.0

var elapsed_time: float = 0.0
var joints: Array[HingeJoint3D] = []

func _ready():
	_collect_joints()
	if enable_motion:
		_start_motors()

func _collect_joints():
	joints.clear()
	for i in range(8):
		var j = get_node_or_null("../face" + str(i) + "/joint" + str(i)) as HingeJoint3D
		if j:
			joints.append(j)

func _process(delta):
	if not enable_motion or joints.is_empty():
		return

	# headless 模式不支持物理马达，静默跳过
	if not _motors_available():
		return

	elapsed_time += delta

	for i in range(len(joints)):
		var joint = joints[i]
		if not joint:
			continue

		# 正弦波速度目标（°/s）
		# 相邻关节之间相位偏移 phase_offset * 2π → 波浪运动
		var phase = float(i) * phase_offset * TAU
		var target_velocity = speed_amplitude * sin(elapsed_time * TAU / period + phase)

		joint.set("motor/target_velocity", target_velocity)

static func _motors_available() -> bool:
	# headless 模式使用 Dummy 显示服务器 → 不支持物理马达
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

func restart():
	elapsed_time = 0.0

func set_phase_offset(offset: float):
	phase_offset = clampf(offset, 0.0, 1.0)

func set_speed(amp: float):
	speed_amplitude = amp

func set_period(p: float):
	period = maxf(p, 0.1)
