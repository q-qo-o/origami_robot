extends RefCounted
class_name STLParser

## 二进制 STL 文件解析器
##
## 从二进制 STL 文件读取三角形网格数据，同时进行 SDF (右手 Z-up) → Godot (左手 Y-up) 坐标转换。
## 坐标映射: (x, y, z)_sdf → (x, z, -y)_godot
##
## 用法:
##   var mesh := STLParser.parse_stl("res://path/to/model.stl", 0.001)
##   if mesh:
##       mesh_instance.mesh = mesh

static func parse_stl(path: String, scale: float = 0.001) -> ArrayMesh:
	var file := FileAccess.open(path, FileAccess.READ)
	if not file:
		push_error("STLParser: 无法打开 STL 文件: " + path)
		return null

	var bytes := file.get_buffer(file.get_length())
	file.close()

	if bytes.size() < 84:
		push_error("STLParser: STL 文件太小 (<84 字节): " + path)
		return null

	var stream := StreamPeerBuffer.new()
	stream.set_data_array(bytes)
	stream.big_endian = false  # 二进制 STL 小端序

	stream.seek(80)  # 跳过 80 字节头部
	var num_triangles := stream.get_u32()

	var expected_size := 80 + 4 + num_triangles * 50
	if bytes.size() < expected_size:
		push_error("STLParser: STL 文件大小不匹配 (期望 %d 字节, 实际 %d): %s" % [expected_size, bytes.size(), path])
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
			# SDF(X,Y,Z) → Godot(X,Z,-Y)，再应用 scale
			var vertex := Vector3(vx, vz, -vy) * scale
			vertices.append(vertex)
			normals.append(normal)

		stream.get_u16()  # 跳过属性字节

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals

	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh
