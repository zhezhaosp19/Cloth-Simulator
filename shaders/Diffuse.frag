#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // YOUR CODE HERE
    float r = distance(u_light_pos, vec3(v_position));
    vec3 i = u_light_intensity / (r * r);
    vec3 light = i * max(0, dot(vec3(v_normal), normalize(u_light_pos - vec3(v_position))));
    out_color = vec4(light, 1);
  // (Placeholder code. You will want to replace it.)
//    out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
//    out_color.a = 1;
}
