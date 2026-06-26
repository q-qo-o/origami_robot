shader_type canvas_item;

uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_nearest;

uniform float intensity : hint_range(0.0, 1.0) = 1.0;
uniform vec4 water_tint : source_color = vec4(0.05, 0.25, 0.45, 0.35);
uniform float darkness : hint_range(0.0, 1.0) = 0.25;

void fragment() {
	vec4 screen_color = texture(screen_texture, SCREEN_UV);
	// 整体暗化
	screen_color.rgb *= (1.0 - darkness * intensity);
	// 向蓝色偏移
	screen_color.rgb = mix(screen_color.rgb, water_tint.rgb, water_tint.a * intensity);
	COLOR = screen_color;
}
