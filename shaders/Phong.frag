#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
    float ka = 0.5;
    float kd = 1.0;
    float ks = 1.0;
    float p = 30.0;
    vec3 ia = vec3(0.7, 0.7, 0.7);
    
    vec3 la= ka * ia;
    
    float r = distance(u_light_pos, vec3(v_position));
    vec3 i = u_light_intensity / (r * r);
    
    vec3 ld = kd * i * max(0, dot(vec3(v_normal), normalize(u_light_pos - vec3(v_position))));
    
    vec3 l = normalize(u_light_pos - vec3(v_position));
    vec3 v = normalize(u_cam_pos - - vec3(v_position));
    vec3 h = normalize(l + v);
    vec3 ls = ks * i * pow(max(0, dot(vec3(v_normal), h)), p);
    
    vec3 total_l = la + ld + ls;
    out_color = vec4(total_l, 1);
    
  
  // (Placeholder code. You will want to replace it.)
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
//  out_color.a = 1;
}

