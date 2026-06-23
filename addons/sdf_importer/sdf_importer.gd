extends RefCounted
class_name SDFImporter

## SDF (Simulation Description Format) → Godot 场景导入器
##
## 将 .sdf 文件导入为 .tscn (PackedScene) 文件。
## 由 plugin.gd 在右键菜单中调用。

# =============================================================================
# 常量
# =============================================================================

const FACE_COLORS := [
	Color(0.90, 0.25, 0.25),
	Color(0.25, 0.70, 0.25),
	Color(0.25, 0.25, 0.90),
	Color(0.90, 0.90, 0.25),
	Color(0.90, 0.25, 0.90),
	Color(0.25, 0.90, 0.90),
	Color(0.95, 0.55, 0.20),
	Color(0.60, 0.60, 0.60),
]

const COORD_T: Basis = Basis(
	Vector3(1, 0, 0),   # SDF X → Godot X
	Vector3(0, 0, -1),  # SDF Y → Godot -Z
	Vector3(0, 1, 0)    # SDF Z → Godot Y
)

# =============================================================================
# 公开接口
# =============================================================================

static func import_file(
	sdf_path: String,
	mesh_scale: float = 0.001,
	default_mass: float = 1.325,
	generate_collision: bool = true,
	generate_visual_mesh: bool = true,
	use_face_colors: bool = true,
) -> int:
	var sdf_data: Dictionary = SDFParser.parse_sdf(sdf_path)
	if sdf_data.is_empty() or sdf_data["links"].is_empty():
		push_error("SDFImporter: 无法解析 SDF 文件: " + sdf_path)
		return ERR_PARSE_ERROR

	var model_name: String = sdf_data["model_name"]
	if model_name.is_empty():
		model_name = "SDFModel"

	var sdf_dir: String = sdf_path.get_base_dir()

	var root := Node3D.new()
	root.name = model_name

	var link_nodes: Dictionary = {}
	var color_index: int = 0

	for link_data in sdf_data["links"]:
		var body := RigidBody3D.new()
		body.name = link_data["name"]

		var mass: float = float(link_data["mass"]) if link_data["mass"] != 1.0 else default_mass
		body.mass = mass

		var pose: Dictionary = link_data["pose"]
		var sdf_pos: Vector3 = pose["position"]
		body.position = COORD_T * sdf_pos

		var sdf_rot: Vector3 = pose["rotation"]
		var basis_sdf: Basis = _euler_xyz_to_basis(sdf_rot.x, sdf_rot.y, sdf_rot.z)
		body.transform.basis = COORD_T * basis_sdf * COORD_T.transposed()

		if link_data.has("inertial_pose") and link_data["inertial_pose"] != null:
			var inertial: Dictionary = link_data["inertial_pose"]
			var com_sdf: Vector3 = inertial["position"]
			body.center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
			body.center_of_mass = COORD_T * com_sdf

		root.add_child(body)
		link_nodes[link_data["name"]] = body

		if generate_visual_mesh and not link_data["mesh_uri"].is_empty():
			_create_visual_mesh(body, link_data, sdf_dir, mesh_scale, use_face_colors, color_index)

		if generate_collision and not link_data["mesh_uri"].is_empty():
			_create_collision(body, link_data, sdf_dir, mesh_scale)

		color_index += 1

	for joint_data in sdf_data["joints"]:
		_create_joint(root, joint_data, link_nodes)

	# 设置 owner（子节点的 owner 为根节点，根节点 owner 为 null）
	for c in root.get_children():
		_recursive_set_owner(c, root)

	var scene := PackedScene.new()
	var pack_result: int = scene.pack(root)
	if pack_result != OK:
		push_error("SDFImporter: 场景打包失败 (错误码: %d)" % pack_result)
		return pack_result

	var output_path: String = sdf_path.get_basename() + ".tscn"
	var save_result: int = ResourceSaver.save(scene, output_path)
	if save_result != OK:
		push_error("SDFImporter: 场景保存失败: " + output_path)
		return save_result

	print("SDFImporter: 成功导入 %s → %s (%d links, %d joints)" % [
		sdf_path, output_path, sdf_data["links"].size(), sdf_data["joints"].size()])
	return OK

# =============================================================================
# Link 创建
# =============================================================================

static func _create_visual_mesh(
	body: RigidBody3D,
	d: Dictionary,
	sdf_dir: String,
	mesh_scale: float,
	use_colors: bool,
	color_idx: int,
):
	var mi := MeshInstance3D.new()
	mi.name = d["name"] + "_visual"

	var mesh_path: String = _resolve_mesh_uri(d["mesh_uri"], sdf_dir)
	var mesh: ArrayMesh = STLParser.parse_stl(mesh_path, mesh_scale)
	if not mesh:
		push_warning("SDFImporter: 无法加载 STL mesh: " + mesh_path)
		return
	mi.mesh = mesh

	if use_colors:
		var mat := StandardMaterial3D.new()
		mat.albedo_color = FACE_COLORS[color_idx % FACE_COLORS.size()]
		mat.metallic = 0.1
		mat.roughness = 0.6
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		mi.material_override = mat

	body.add_child(mi)

static func _create_collision(
	body: RigidBody3D,
	d: Dictionary,
	sdf_dir: String,
	mesh_scale: float,
):
	var col := CollisionShape3D.new()
	col.name = d["name"] + "_collision"

	var mesh_path: String = _resolve_mesh_uri(d["mesh_uri"], sdf_dir)
	var mesh: ArrayMesh = STLParser.parse_stl(mesh_path, mesh_scale)
	if mesh:
		var shape := mesh.create_convex_shape()
		if shape:
			col.shape = shape
		else:
			push_warning("SDFImporter: 无法为 %s 创建碰撞形状" % d["name"])

	body.add_child(col)

# =============================================================================
# Joint 创建
# =============================================================================

static func _create_joint(
	root: Node3D,
	d: Dictionary,
	link_nodes: Dictionary,
):
	if d["type"] != "revolute":
		push_warning("SDFImporter: 跳过不支持的关节 '%s'" % d["type"])
		return

	var child := link_nodes.get(d["child"]) as RigidBody3D
	if not child:
		return

	var parent := link_nodes.get(d["parent"]) as RigidBody3D
	if not parent:
		return

	var hinge := HingeJoint3D.new()
	hinge.name = d["name"]
	hinge.node_a = NodePath("../" + d["parent"])
	hinge.node_b = NodePath("../" + d["child"])

	var pose: Dictionary = d["pose"]
	var local_sdf: Vector3 = pose["position"]
	var local_godot: Vector3 = COORD_T * local_sdf
	hinge.position = child.position + child.transform.basis * local_godot

	var sdf_axis: Vector3 = d["axis"]
	if sdf_axis.length_squared() > 0.0:
		var godot_axis: Vector3 = (COORD_T * sdf_axis).normalized()
		_align_basis_to_axis(hinge, godot_axis)

	root.add_child(hinge)

# =============================================================================
# 工具函数
# =============================================================================

static func _align_basis_to_axis(hinge: HingeJoint3D, target_axis: Vector3):
	var default_axis := Vector3(0, 0, 1)
	var cross := default_axis.cross(target_axis)
	if cross.length_squared() > 1e-6:
		cross = cross.normalized()
		var angle := acos(clampf(default_axis.dot(target_axis), -1.0, 1.0))
		hinge.transform.basis = Basis(cross, angle)
	elif default_axis.dot(target_axis) < 0.0:
		hinge.transform.basis = Basis(Vector3(1, 0, 0), PI)

static func _euler_xyz_to_basis(roll: float, pitch: float, yaw: float) -> Basis:
	var rx := Basis(Vector3(1, 0, 0), roll)
	var ry := Basis(Vector3(0, 1, 0), pitch)
	var rz := Basis(Vector3(0, 0, 1), yaw)
	return rz * ry * rx

static func _resolve_mesh_uri(uri: String, sdf_dir: String) -> String:
	if uri.begins_with("model://"):
		var path_after_model: String = uri.substr(uri.find("/", 8) + 1)
		return sdf_dir.path_join(path_after_model)
	return uri

static func _recursive_set_owner(node: Node, owner: Node):
	node.owner = owner
	for c in node.get_children():
		_recursive_set_owner(c, owner)
