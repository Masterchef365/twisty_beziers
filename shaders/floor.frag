#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;

layout(binding = 1) uniform Animation {
    float anim;
};

layout(location = 0) out vec4 outColor;

void main() {
    vec2 st = fragColor.xy;
    st.y *= 25.;
    //bvec2 tile = greaterThan(fract(st * 2.), vec2(.5));
    //vec3 color = vec3(tile.x != tile.y);
    vec3 color = vec3(fract(st.y + abs(.5 - st.x) - anim) > .5);
    outColor = vec4(color, 1.0);
}
