extends Camera3D

## Camera3D 轨道控制器
## 滚轮缩放 / 中键拖拽平移 / Shift+中键拖拽旋转

@export_group("Speed")
@export var zoom_speed: float = 0.12
@export var pan_speed: float = 0.003
@export var rotate_speed: float = 0.005

@export_group("Limits")
@export var min_distance: float = 0.3
@export var max_distance: float = 20.0

var _pivot: Vector3 = Vector3.ZERO
var _distance: float = 2.0
var _yaw: float = 0.0
var _pitch: float = 0.4
var _initialised: bool = false

var _rotate_pivot: Vector3 = Vector3.ZERO    # Shift+中键拖拽时使用的旋转中心
var _is_rotating: bool = false               # 是否正在旋转拖拽中


func _ready():
	# 从当前相机位置反推初始轨道参数
	_reconstruct_orbit()


func _reconstruct_orbit():
	_pivot = _get_look_target()
	_distance = global_position.distance_to(_pivot)
	var local_pos: Vector3 = global_position - _pivot
	_yaw = atan2(local_pos.x, local_pos.z)
	_pitch = asin(clampf(local_pos.y / maxf(_distance, 0.001), -1.0, 1.0))
	_initialised = true


func _get_look_target() -> Vector3:
	# 沿相机朝向射线检测目标点，未命中则取前方 _distance 处
	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(
		global_position,
		global_position - global_transform.basis.z * 50.0
	)
	var result := space.intersect_ray(query)
	if result.has("position"):
		return result["position"]
	return global_position - global_transform.basis.z * _distance


func _get_mouse_hit_point(screen_pos: Vector2) -> Vector3:
	var ray_origin := project_ray_origin(screen_pos)
	var ray_dir := project_ray_normal(screen_pos)
	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(ray_origin, ray_origin + ray_dir * 1000.0)
	var result := space.intersect_ray(query)
	if result.has("position"):
		return result["position"]
	return Vector3.ZERO


func _input(event: InputEvent) -> void:
	if not _initialised:
		return

	# —— 滚轮缩放 ——
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP and event.pressed:
			_distance *= (1.0 - zoom_speed)
			_update_transform()
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN and event.pressed:
			_distance *= (1.0 + zoom_speed)
			_update_transform()

	# —— 中键按下：区分平移(无Shift)和旋转(有Shift) ——
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_MIDDLE and event.pressed:
		if Input.is_key_pressed(KEY_SHIFT):
			# Shift+中键按下：获取鼠标下的场景点作为旋转中心
			var mouse_pos := get_viewport().get_mouse_position()
			var hit := _get_mouse_hit_point(mouse_pos)
			if hit != Vector3.ZERO:
				_rotate_pivot = hit
				_distance = global_position.distance_to(_rotate_pivot)
			else:
				_rotate_pivot = _pivot
			_is_rotating = true

	# —— 中键释放：结束旋转 ——
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_MIDDLE and not event.pressed:
		_is_rotating = false

	# —— 中键拖拽平移（不加Shift） ——
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_MIDDLE) \
			and not Input.is_key_pressed(KEY_SHIFT):
		var delta: Vector2 = event.relative
		var right: Vector3 = global_transform.basis.x
		var up: Vector3 = global_transform.basis.y
		var scale_factor: float = _distance * pan_speed
		_pivot -= right * delta.x * scale_factor
		_pivot += up * delta.y * scale_factor
		_update_transform()

	# —— Shift+中键拖拽旋转 ——
	if event is InputEventMouseMotion and _is_rotating:
		var delta: Vector2 = event.relative
		_pivot = _rotate_pivot
		_yaw -= delta.x * rotate_speed
		_pitch += delta.y * rotate_speed
		_pitch = clampf(_pitch, -PI / 2.0 + 0.02, PI / 2.0 - 0.02)
		_update_transform()


func _update_transform() -> void:
	_distance = clampf(_distance, min_distance, max_distance)
	var pos: Vector3 = _pivot + Vector3(
		cos(_pitch) * sin(_yaw),
		sin(_pitch),
		cos(_pitch) * cos(_yaw)
	) * _distance
	look_at_from_position(pos, _pivot, Vector3.UP)
