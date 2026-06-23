@tool
extends EditorPlugin

## SDF Importer 插件入口
##
## 在编辑器顶部菜单栏添加 "Import SDF as tscn" 工具菜单。
## 点击后弹出文件选择对话框，选中 .sdf 文件即可导入为 .tscn 场景。

const IMPORT_ACTION := "SDF Import/Import SDF as tscn"

var _dialog: EditorFileDialog


func _enter_tree():
	add_tool_menu_item(IMPORT_ACTION, _on_import_menu)


func _exit_tree():
	remove_tool_menu_item(IMPORT_ACTION)
	_free_dialog()


func _on_import_menu():
	_free_dialog()
	_dialog = EditorFileDialog.new()
	_dialog.title = "Select SDF File"
	_dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
	_dialog.file_mode = EditorFileDialog.FILE_MODE_OPEN_FILE
	_dialog.add_filter("*.sdf", "SDF Model File")
	_dialog.file_selected.connect(_on_file_selected, CONNECT_ONE_SHOT)
	_dialog.close_requested.connect(_free_dialog)

	var base := EditorInterface.get_base_control()
	base.add_child(_dialog)
	_dialog.popup_centered_ratio.call_deferred(0.4)


func _free_dialog():
	if _dialog:
		_dialog.queue_free()
		_dialog = null


func _on_file_selected(sdf_path: String):
	print("SDF Importer: 开始导入: %s" % sdf_path)
	var result: int = SDFImporter.import_file(sdf_path)
	if result == OK:
		EditorInterface.get_resource_filesystem().scan()
		print("SDF Importer: 导入完成: %s" % sdf_path)
	else:
		push_error("SDF Importer: 导入失败 (错误码 %d): %s" % [result, sdf_path])
	_free_dialog()
