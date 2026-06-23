@tool
extends Node3D

## SDF (Simulation Description Format) 场景导入器
##
## 从 SDF 文件读取模型定义（links + joints），自动创建 RigidBody3D 和 Joint 节点。
## 支持二进制 STL 网格的解析和 SDF→Godot 坐标转换。
##
## 使用方法：将本脚本挂载到任意 Node3D，设置 sdf_path 后运行。
##
## SDF 坐标系 (右手系, Z向上) → Godot 坐标系 (左手系, Y向上)：
##   位置: (x, y, z)_sdf → (x, z, -y)_godot
##   旋转: Basis_godot = C * Basis_sdf * C^T

signal build_completed

# =============================================================================
# 导出属性
# =============================================================================

@export var sdf_path: String = "res://sdf/origami_robot/origami_robot.sdf"

@export var auto_load: bool = true:
	set(value):
		auto_load = value
		if auto_load and Engine.is_editor_hint():
			load_and_build()

@export var mesh_scale: float = 0.001

@export var default_mass: float = 1.325

# =============================================================================
# 常量
# =============================================================================

# 每个面的颜色，便于区分
const FACE_COLORS := [
	Color(0.90, 0.25, 0.25),  # face0: 红
	Color(0.25, 0.70, 0.25),  # face1: 绿
	Color(0.25, 0.25, 0.90),  # face2: 蓝
	Color(0.90, 0.90, 0.25),  # face3: 黄
	Color(0.90, 0.25, 0.90),  # face4: 紫
	Color(0.25, 0.90, 0.90),  # face5: 青
	Color(0.95, 0.55, 0.20),  # face6: 橙
	Color(0.60, 0.60, 0.60),  # face7: 灰
]

# SDF → Godot 坐标变换矩阵
#   SDF: X右, Y前, Z上  (右手系)
#   Godot: X右, Y上, Z后 (左手系)
#   映射: SDF(X,Y,Z) → Godot(X,Z,-Y)
const COORD_T: Basis = Basis(
	Vector3(1, 0, 0),   # SDF X → Godot X
	Vector3(0, 0, -1),  # SDF Y → Godot -Z
	Vector3(0, 1, 0)    # SDF Z → Godot Y
)

# =============================================================================
# 数据
# =============================================================================

var links: Dictionary = {}   # name → link_data
var joints: Array = []       # array of joint_data dicts

# =============================================================================
# 生命周期
# =============================================================================

func _ready():
	if auto_load:
		load_and_build()

# =============================================================================
# 主入口
# =============================================================================

func load_and_build():
	clear_scene()
	var content := _read_sdf_file()
	if content.is_empty():
		return
	_parse_sdf(content)
	_build_scene()
	build_completed.emit()
	print("SDF 导入完成: %d 个 link, %d 个 joint" % [links.size(), joints.size()])

func clear_scene():
	for child in get_children():
		child.queue_free()
	links.clear()
	joints.clear()

# =============================================================================
# SDF 文件读取
# =============================================================================

func _read_sdf_file() -> String:
	var file := FileAccess.open(sdf_path, FileAccess.READ)
	if not file:
		push_error("无法打开 SDF 文件: " + sdf_path)
		return ""
	var content := file.get_as_text()
	file.close()
	return content

# =============================================================================
# SDF XML 解析
# =============================================================================

func _parse_sdf(content: String):
	_parse_links(content)
	_parse_joints(content)

# ---- Links ----

func _parse_links(content: String):
	var re := RegEx.new()
	re.compile("<link name=\"([^\"]+)\">([\\s\\S]*?)</link>")
	var matches := re.search_all(content)

	var idx := 0
	for m in matches:
		var link_name := m.get_string(1)
		var link_xml := m.get_string(2)

		# 去掉 inertial 子块，避免匹配到其中的 pose
		var xml_no_inertial := _remove_tag(link_xml, "inertial")

		var data := {
			"name": link_name,
			"pose": _parse_pose(_extract_tag(xml_no_inertial, "pose")),
			"mass": default_mass,
			"mesh_uri": "",
			"mesh_scale": Vector3.ONE,
			"inertial_pose": null,
			"color_index": idx % FACE_COLORS.size(),
		}
		idx += 1

		# mass
		var mass_str := _extract_tag(link_xml, "mass")
		if not mass_str.is_empty():
			data["mass"] = float(mass_str)

		# mesh URI (第一个 <uri>)
		var uri := _extract_tag(link_xml, "uri")
		if not uri.is_empty():
			data["mesh_uri"] = uri.replace("model://", "res://sdf/")

		# mesh scale (第一个 <scale>)
		var scale_str := _extract_tag(link_xml, "scale")
		if not scale_str.is_empty():
			data["mesh_scale"] = _parse_vector3(scale_str)

		# inertial pose (质心偏移)
		var inertial_pose_str := _extract_inertial_pose(link_xml)
		if not inertial_pose_str.is_empty():
			data["inertial_pose"] = _parse_pose(inertial_pose_str)

		links[link_name] = data

# ---- Joints ----

func _parse_joints(content: String):
	var re := RegEx.new()
	re.compile("<joint name=\"([^\"]+)\" type=\"([^\"]+)\">([\\s\\S]*?)</joint>")
	var matches := re.search_all(content)

	for m in matches:
		var jxml := m.get_string(3)
		var data := {
			"name": m.get_string(1),
			"type": m.get_string(2),
			"pose": _parse_pose(_extract_tag(jxml, "pose")),
			"parent": _extract_tag(jxml, "parent"),
			"child": _extract_tag(jxml, "child"),
			"axis": _parse_vector3(_extract_tag(jxml, "xyz")),
		}
		joints.append(data)

# =============================================================================
# 场景构建
# =============================================================================

func _build_scene():
	# 第一遍：创建所有 link (RigidBody3D)
	for link_name in links.keys():
		_create_link(links[link_name])

	# 第二遍：创建所有 joint (HingeJoint3D)
	for jdata in joints:
		_create_joint(jdata)

# ---- 创建单个 Link ----

func _create_link(d: Dictionary):
	var body := RigidBody3D.new()
	body.name = d["name"]
	body.mass = d["mass"]

	# --- 位置 ---
	var sdf_pos: Vector3 = d["pose"]["position"]
	body.position = COORD_T * sdf_pos

	# --- 旋转: SDF (roll, pitch, yaw) → Godot Basis ---
	var sdf_rot: Vector3 = d["pose"]["rotation"]
	var basis_sdf := _euler_xyz_to_basis(sdf_rot.x, sdf_rot.y, sdf_rot.z)
	body.transform.basis = COORD_T * basis_sdf * COORD_T.transposed()

	# --- 质心偏移 ---
	if d.has("inertial_pose") and d["inertial_pose"] != null:
		var com_sdf: Vector3 = d["inertial_pose"]["position"]
		body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		body.center_of_mass = COORD_T * com_sdf

	_add_child_owned(body)

	# ---- Visual mesh ----
	if not d["mesh_uri"].is_empty():
		_create_visual_mesh(body, d)

	# ---- Collision shape ----
	if not d["mesh_uri"].is_empty():
		_create_collision(body, d)

func _create_visual_mesh(body: RigidBody3D, d: Dictionary):
	var mi := MeshInstance3D.new()
	mi.name = d["name"] + "_visual"

	var mesh := _parse_stl(d["mesh_uri"], mesh_scale)
	if not mesh:
		push_warning("无法加载 STL mesh: " + d["mesh_uri"])
		return
	mi.mesh = mesh

	# 材质颜色
	var mat := StandardMaterial3D.new()
	mat.albedo_color = FACE_COLORS[d["color_index"]]
	mat.metallic = 0.1
	mat.roughness = 0.6
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mi.material_override = mat

	_add_child_owned(body, mi)

func _create_collision(body: RigidBody3D, d: Dictionary):
	var col := CollisionShape3D.new()
	col.name = d["name"] + "_collision"

	var mesh := _parse_stl(d["mesh_uri"], mesh_scale)
	if mesh:
		var shape := mesh.create_convex_shape()
		if shape:
			col.shape = shape
		else:
			push_warning("无法为 %s 创建碰撞形状" % d["name"])

	_add_child_owned(body, col)

# ---- 创建单个 Joint ----

func _create_joint(d: Dictionary):
	if d["type"] != "revolute":
		push_warning("不支持的关节类型 '%s'，跳过 joint %s" % [d["type"], d["name"]])
		return

	var child := get_node_or_null(d["child"]) as RigidBody3D
	if not child:
		push_warning("找不到 child link '%s'，跳过 joint %s" % [d["child"], d["name"]])
		return

	var hinge := HingeJoint3D.new()
	hinge.name = d["name"]

	# 连接到 parent 和 child
	hinge.node_a = NodePath("../" + d["parent"])
	hinge.node_b = NodePath("../" + d["child"])

	# --- Joint 位置 ---
	# SDF 中 joint pose 在 child link 的坐标系中
	var local_sdf: Vector3 = d["pose"]["position"]
	var local_godot: Vector3 = COORD_T * local_sdf
	# 世界位置 = child 位置 + child 旋转 × 局部偏移
	hinge.position = child.position + child.transform.basis * local_godot

	# --- Joint 轴 ---
	# 将 SDF model 坐标系的轴转换到 Godot 世界坐标系
	var sdf_axis: Vector3 = d["axis"]
	if sdf_axis.length_squared() > 0.0:
		var godot_axis: Vector3 = (COORD_T * sdf_axis).normalized()
		# HingeJoint3D 默认旋转轴为 Z
		_align_basis_to_axis(hinge, godot_axis)

	_add_child_owned(hinge)

# =============================================================================
# STL 二进制解析 (小端序)
# =============================================================================

static func _parse_stl(path: String, scale: float = 0.001) -> ArrayMesh:
	var file := FileAccess.open(path, FileAccess.READ)
	if not file:
		push_error("无法打开 STL 文件: " + path)
		return null

	var bytes := file.get_buffer(file.get_length())
	file.close()

	if bytes.size() < 84:
		push_error("STL 文件太小: " + path)
		return null

	var stream := StreamPeerBuffer.new()
	stream.set_data_array(bytes)
	stream.big_endian = false  # 二进制 STL 小端序

	stream.seek(80)  # 跳过 80 字节头部
	var num_triangles := stream.get_u32()

	if bytes.size() < 80 + 4 + num_triangles * 50:
		push_error("STL 文件大小不匹配: " + path)
		return null

	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()

	for _i in range(num_triangles):
		# 法线 (SDF 坐标系)
		var nx := stream.get_float()
		var ny := stream.get_float()
		var nz := stream.get_float()
		# SDF(X,Y,Z) → Godot(X,Z,-Y)
		var normal := Vector3(nx, nz, -ny).normalized()

		for _j in range(3):
			var vx := stream.get_float()
			var vy := stream.get_float()
			var vz := stream.get_float()
			var vertex := Vector3(vx, vz, -vy) * scale
			vertices.append(vertex)
			normals.append(normal)

		stream.get_u16()  # 属性字节

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals

	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh

# =============================================================================
# 工具函数
# =============================================================================

## 将 HingeJoint3D 的 Basis 对齐到指定轴 (默认旋转轴为 Z)
static func _align_basis_to_axis(hinge: HingeJoint3D, target_axis: Vector3):
	var default_axis := Vector3(0, 0, 1)
	var cross := default_axis.cross(target_axis)
	if cross.length_squared() > 1e-6:
		cross = cross.normalized()
		var angle := acos(clampf(default_axis.dot(target_axis), -1.0, 1.0))
		hinge.transform.basis = Basis(cross, angle)
	elif default_axis.dot(target_axis) < 0.0:
		# 方向相反 → 绕 X 旋转 180°
		hinge.transform.basis = Basis(Vector3(1, 0, 0), PI)

## SDF 欧拉角 (XYZ 顺序) → Godot Basis
static func _euler_xyz_to_basis(roll: float, pitch: float, yaw: float) -> Basis:
	var rx := Basis(Vector3(1, 0, 0), roll)
	var ry := Basis(Vector3(0, 1, 0), pitch)
	var rz := Basis(Vector3(0, 0, 1), yaw)
	return rz * ry * rx  # XYZ intrinsic

## 移除 XML 标签块
static func _remove_tag(xml: String, tag: String) -> String:
	var re := RegEx.new()
	re.compile("<" + tag + "[^>]*>[\\s\\S]*?</" + tag + ">")
	return re.sub(xml, "", true)

## 提取 XML 标签内的文本内容
static func _extract_tag(xml: String, tag: String) -> String:
	var re := RegEx.new()
	re.compile("<" + tag + "[^>]*>([\\s\\S]*?)</" + tag + ">")
	var m := re.search(xml)
	return m.get_string(1).strip_edges() if m else ""

## 提取 <inertial> 块内的 <pose>
static func _extract_inertial_pose(xml: String) -> String:
	var re := RegEx.new()
	re.compile("<inertial>[\\s\\S]*?<pose>([\\s\\S]*?)</pose>[\\s\\S]*?</inertial>")
	var m := re.search(xml)
	return m.get_string(1).strip_edges() if m else ""

## 按空白符分割
static func _split_whitespace(s: String) -> PackedStringArray:
	var re := RegEx.new()
	re.compile("\\S+")
	var ms := re.search_all(s)
	var out := PackedStringArray()
	for m in ms:
		out.append(m.get_string())
	return out

## 解析 SDF pose 字符串 "x y z roll pitch yaw"
static func _parse_pose(s: String) -> Dictionary:
	var parts := _split_whitespace(s)
	if parts.size() >= 6:
		return {
			"position": Vector3(float(parts[0]), float(parts[1]), float(parts[2])),
			"rotation": Vector3(float(parts[3]), float(parts[4]), float(parts[5])),
		}
	return {"position": Vector3.ZERO, "rotation": Vector3.ZERO}

## 解析 Vector3 字符串 "x y z"
static func _parse_vector3(s: String) -> Vector3:
	var parts := _split_whitespace(s)
	if parts.size() >= 3:
		return Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
	return Vector3.ONE

## 添加子节点并设置 owner（编辑模式下使节点在场景树中可见）
func _add_child_owned(parent: Node, child: Node = null):
	if child:
		parent.add_child(child)
	else:
		child = parent
		parent = self
		parent.add_child(child)

	if Engine.is_editor_hint():
		var root := get_tree().edited_scene_root
		if root:
			_recursive_set_owner(child, root)

static func _recursive_set_owner(node: Node, owner: Node):
	node.owner = owner
	for c in node.get_children():
		_recursive_set_owner(c, owner)
