extends RefCounted
class_name SDFParser

## SDF (Simulation Description Format) XML 解析器
##
## 从 SDF 1.7 XML 文件中提取 link 和 joint 定义数据。
## 返回结构化的 Dictionary，供 SDFImporter 用于构建 Godot 场景。
##
## 用法:
##   var data := SDFParser.parse_sdf("res://path/to/model.sdf")
##   for link in data["links"]:
##       print(link["name"], " → ", link["pose"]["position"])

## 解析 SDF 文件，返回模型数据 Dictionary
##
## 返回值结构:
##   {
##     "model_name": String,         # SDF 模型名称
##     "links": [                    # Array[Dictionary]
##       {
##         "name": String,           # 如 "face0"
##         "pose": {                 # SDF 坐标系下的位姿
##           "position": Vector3,    # (x, y, z)
##           "rotation": Vector3     # (roll, pitch, yaw) 弧度
##         },
##         "mass": float,            # 质量
##         "mesh_uri": String,       # 网格 URI (如 "model://.../face0.stl")
##         "mesh_scale": Vector3,    # 网格缩放
##         "inertial_pose": Dictionary or null  # 质心偏移
##       },
##       ...
##     ],
##     "joints": [                   # Array[Dictionary]
##       {
##         "name": String,           # 如 "joint0"
##         "type": String,           # 如 "revolute"
##         "parent": String,         # parent link 名称
##         "child": String,          # child link 名称
##         "pose": { "position": Vector3, "rotation": Vector3 },
##         "axis": Vector3           # 旋转轴 (SDF 坐标系)
##       },
##       ...
##     ]
##   }
static func parse_sdf(sdf_path: String) -> Dictionary:
	var file := FileAccess.open(sdf_path, FileAccess.READ)
	if not file:
		push_error("SDFParser: 无法打开 SDF 文件: " + sdf_path)
		return {}

	var content := file.get_as_text()
	file.close()

	if content.is_empty():
		push_error("SDFParser: SDF 文件为空: " + sdf_path)
		return {}

	# 提取 model name
	var model_name := ""
	var re_model := RegEx.new()
	re_model.compile("<model\\s+name=\"([^\"]+)\"")
	var m_model := re_model.search(content)
	if m_model:
		model_name = m_model.get_string(1)

	var result := {
		"model_name": model_name,
		"links": [],
		"joints": [],
	}

	# 解析 links
	result["links"] = _parse_links(content)

	# 解析 joints
	result["joints"] = _parse_joints(content)

	return result

# =============================================================================
# Links 解析
# =============================================================================

static func _parse_links(content: String) -> Array:
	var re := RegEx.new()
	re.compile("<link name=\"([^\"]+)\">([\\s\\S]*?)</link>")
	var matches := re.search_all(content)

	var links: Array = []
	var idx := 0
	for m in matches:
		var link_name := m.get_string(1)
		var link_xml := m.get_string(2)

		# 去掉 inertial 子块，避免匹配到其中的 pose
		var xml_no_inertial := _remove_tag(link_xml, "inertial")

		var data := {
			"name": link_name,
			"pose": _parse_pose(_extract_tag(xml_no_inertial, "pose")),
			"mass": 1.0,
			"mesh_uri": "",
			"mesh_scale": Vector3.ONE,
			"inertial_pose": null,
		}

		# mass
		var mass_str := _extract_tag(link_xml, "mass")
		if not mass_str.is_empty():
			data["mass"] = float(mass_str)

		# mesh URI (第一个 <uri>)
		var uri := _extract_tag(link_xml, "uri")
		if not uri.is_empty():
			data["mesh_uri"] = uri

		# mesh scale (第一个 <scale>)
		var scale_str := _extract_tag(link_xml, "scale")
		if not scale_str.is_empty():
			data["mesh_scale"] = _parse_vector3(scale_str)

		# inertial pose (质心偏移)
		var inertial_pose_str := _extract_inertial_pose(link_xml)
		if not inertial_pose_str.is_empty():
			data["inertial_pose"] = _parse_pose(inertial_pose_str)

		links.append(data)
		idx += 1

	return links

# =============================================================================
# Joints 解析
# =============================================================================

static func _parse_joints(content: String) -> Array:
	var re := RegEx.new()
	re.compile("<joint name=\"([^\"]+)\" type=\"([^\"]+)\">([\\s\\S]*?)</joint>")
	var matches := re.search_all(content)

	var joints: Array = []
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

	return joints

# =============================================================================
# 工具函数
# =============================================================================

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
