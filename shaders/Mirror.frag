#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
    vec3 w0 = normalize(u_cam_pos - vec3(v_position));
    vec3 wi = -w0 + 2.0 * dot(w0, vec3(v_normal)) * vec3(v_normal);
    out_color = texture(u_texture_cubemap, wi);
    
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
//  out_color.a = 1;
}
