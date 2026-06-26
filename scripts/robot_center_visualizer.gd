extends Node3D

## 实时显示折纸机器人整体重心与浮心
## ============================================================
## - 红色圆点 = 整体重心（Center of Mass, COM）
## - 蓝色圆点 = 整体浮心（Center of Buoyancy, COB）
##
## 重心：所有 face RigidBody3D 的 center_of_mass（世界坐标）按 mass 加权平均。
## 浮心：所有 face 的 buoyancy_mesh.gd 计算的 last_buoyancy_center_world
##       按 last_total_force 加权平均。

const SPHERE_RADIUS := 0.022
const FACE_COUNT := 8

## 始终在最上层渲染的球体 shader（禁用深度测试 + 自发光）
const _MARKER_SHADER_CODE := """
shader_type spatial;
render_mode unshaded, depth_test_disabled, cull_disabled;

uniform vec4 albedo : source_color = vec4(1.0, 0.0, 0.0, 1.0);
uniform float emission_energy : hint_range(0.0, 16.0) = 2.5;

void fragment() {
	ALBEDO = albedo.rgb * emission_energy;
	ALPHA = albedo.a;
}
"""

var _com_marker: MeshInstance3D   # 重心标记（红色）
var _cob_marker: MeshInstance3D   # 浮心标记（蓝色）

# 缓存 face 节点引用，避免每帧 get_node
var _faces: Array[RigidBody3D] = []


func _ready() -> void:
	_com_marker = _create_sphere_marker("CenterOfMass", Color(1.0, 0.15, 0.15, 1.0))
	_cob_marker = _create_sphere_marker("CenterOfBuoyancy", Color(0.15, 0.5, 1.0, 1.0))
	add_child(_com_marker)
	add_child(_cob_marker)

	# 缓存 face 引用
	for i in range(FACE_COUNT):
		var face = get_node_or_null("../face" + str(i)) as RigidBody3D
		if face:
			_faces.append(face)
		else:
			push_warning("[CenterVisualizer] face%d not found" % i)


func _process(_delta: float) -> void:
	if _faces.is_empty():
		return

	var total_mass := 0.0
	var weighted_com := Vector3.ZERO

	var total_buoyancy_force := 0.0
	var weighted_cob := Vector3.ZERO
	var any_buoyancy := false

	for face in _faces:
		# ---- 重心 ----
		var mass := face.mass
		# center_of_mass 是刚体局部坐标；用 global_transform 转到世界系
		var com_world := face.global_transform * face.center_of_mass
		total_mass += mass
		weighted_com += com_world * mass

		# ---- 浮心 ----
		# buoyancy_mesh.gd 中的公开变量
		var buoyancy_force: float = face.get("LastTotalForce")
		var buoyancy_center: Vector3 = face.get("LastBuoyancyCenterWorld")
		if buoyancy_force > 1e-6:
			total_buoyancy_force += buoyancy_force
			weighted_cob += buoyancy_center * buoyancy_force
			any_buoyancy = true

	# 更新重心标记
	if total_mass > 0.0:
		_com_marker.global_position = weighted_com / total_mass
		_com_marker.visible = true
	else:
		_com_marker.visible = false

	# 更新浮心标记
	if any_buoyancy and total_buoyancy_force > 0.0:
		_cob_marker.global_position = weighted_cob / total_buoyancy_force
		_cob_marker.visible = true
	else:
		_cob_marker.visible = false


## 创建一个始终在最上层渲染的自发光球体标记
func _create_sphere_marker(marker_name: String, color: Color) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.name = marker_name

	var sphere := SphereMesh.new()
	sphere.radius = SPHERE_RADIUS
	sphere.height = SPHERE_RADIUS * 2.0
	mi.mesh = sphere

	var shader := Shader.new()
	shader.code = _MARKER_SHADER_CODE

	var mat := ShaderMaterial.new()
	mat.shader = shader
	mat.set_shader_parameter("albedo", color)
	mat.set_shader_parameter("emission_energy", 2.5)
	mi.material_override = mat

	return mi
