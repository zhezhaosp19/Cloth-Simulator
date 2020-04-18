#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv)[0];
}

void main() {
  // YOUR CODE HERE
    vec3 n = vec3(v_normal);
    vec3 t = vec3(v_tangent);
    vec3 b = cross(n, t);
    mat3 tbn = mat3(t, b, n);

    float du = 5 * (h(vec2(v_uv[0] + 1 / u_texture_3_size[0], v_uv[1])) - h(v_uv)) * u_normal_scaling * u_height_scaling;
    float dv = 5 * (h(vec2(v_uv[0],  v_uv[1] + 1 / u_texture_3_size[1])) - h(v_uv)) * u_normal_scaling * u_height_scaling;

    vec3 n0 = vec3(-du, -dv, 1);
    vec3 nd = tbn * n0;

    //phong
    float ka = 0.5;
    float kd = 1.0;
    float ks = 1.0;
    float p = 30.0;
    vec3 ia = vec3(0.4, 0.4, 0.4);

    vec3 la= ka * ia;

    float r = distance(u_light_pos, vec3(v_position));
    vec3 i = u_light_intensity / (r * r);

    vec3 ld = kd * i * max(0, dot(nd, normalize(u_light_pos - vec3(v_position))));

    vec3 l = normalize(u_light_pos - vec3(v_position));
    vec3 v = normalize(u_cam_pos - - vec3(v_position));
    vec3 h = normalize(l + v);
    vec3 ls = ks * i * pow(max(0, dot(nd, h)), p);

    vec3 total_l = la + ld + ls;
    out_color = vec4(total_l, 1);
    
  // (Placeholder code. You will want to replace it.)
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
//  out_color.a = 1;
}

