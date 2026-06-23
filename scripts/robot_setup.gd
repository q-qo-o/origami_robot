@tool
extends Node3D

## 运行时/编辑器 STL mesh 加载器
## 为每个 face 子节点加载对应的 STL 文件，设置 visual mesh 和 collision shape

@export var auto_setup: bool = true:
	set(value):
		auto_setup = value
		if auto_setup and Engine.is_editor_hint():
			setup_robot()

@export var mesh_scale: float = 0.001

## 为每个 face 分配不同的颜色以便区分
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

func _ready():
	if auto_setup:
		setup_robot()

func setup_robot():
	for i in range(8):
		var face_name = "face" + str(i)
		var face = get_node_or_null(face_name)
		if not face:
			push_warning("找不到节点: " + face_name)
			continue

		var stl_path = "res://sdf/origami_robot/meshes/" + face_name + ".stl"
		var mesh = parse_stl(stl_path, mesh_scale)
		if not mesh:
			continue

		# 设置 visual mesh
		var visual = face.get_node_or_null("visual")
		if visual and visual is MeshInstance3D:
			visual.mesh = mesh
			# 分配带颜色的材质以便区分不同 face
			var mat = StandardMaterial3D.new()
			mat.albedo_color = FACE_COLORS[i]
			mat.metallic = 0.1
			mat.roughness = 0.6
			mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			visual.material_override = mat

		# 设置 collision shape —— 使用 convex hull（适合 RigidBody3D）
		var collision = face.get_node_or_null("collision")
		if collision and collision is CollisionShape3D:
			var shape = mesh.create_convex_shape()
			if shape:
				collision.shape = shape
			else:
				push_warning("无法为 " + face_name + " 创建碰撞形状")

## 解析二进制 STL 文件（小端序）并返回 ArrayMesh
## 同时进行 SDF (右手 Z-up) → Godot (左手 Y-up) 顶点坐标转换
static func parse_stl(path: String, mesh_scale: float = 0.001) -> ArrayMesh:
	var file = FileAccess.open(path, FileAccess.READ)
	if not file:
		push_error("无法打开 STL 文件: " + path)
		return null

	var bytes = file.get_buffer(file.get_length())
	file.close()

	if bytes.size() < 84:
		push_error("STL 文件太小（<84 字节）: " + path)
		return null

	var stream = StreamPeerBuffer.new()
	stream.set_data_array(bytes)
	stream.big_endian = false  # STL 为小端序

	stream.seek(80)
	var num_triangles = stream.get_u32()
	var expected_size = 80 + 4 + num_triangles * 50
	if bytes.size() < expected_size:
		push_error("STL 文件大小不匹配: " + path)
		return null

	var vertices = PackedVector3Array()
	var normals = PackedVector3Array()

	for i in range(num_triangles):
		# 法线 (SDF 坐标系)
		var nx = stream.get_float()
		var ny = stream.get_float()
		var nz = stream.get_float()
		# SDF (X,Y,Z) → Godot (X,Z,-Y)
		var normal = Vector3(nx, nz, -ny).normalized()

		for j in range(3):
			var vx = stream.get_float()
			var vy = stream.get_float()
			var vz = stream.get_float()
			# SDF (X,Y,Z) → Godot (X,Z,-Y)，再应用 mesh_scale
			var vertex = Vector3(vx, vz, -vy) * mesh_scale
			vertices.append(vertex)
			normals.append(normal)

		stream.get_u16()  # 跳过属性字节

	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals

	var mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	return mesh
