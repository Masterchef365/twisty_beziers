#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;

layout(binding = 1) uniform Animation {
    float anim;
};

layout(location = 0) out vec4 outColor;

void main() {
    vec2 st = fragColor.xy;

    bvec2 tile = greaterThan(fract(st * 60.), vec2(.5));
    vec3 color = vec3(tile.x != tile.y);

    //float v = st.y * 10. + abs(.5 - st.x);
    //vec3 color = vec3(fract(v - anim * 10.) > .5);
    
    //outColor = vec4(vec2(st), 0., 1.0);

    outColor = vec4(color, 1.0);
}
