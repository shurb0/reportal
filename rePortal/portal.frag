#version 460

uniform sampler2D p3d_Texture0;
uniform vec4 p3d_ClipPlane[2];

in vec4 vpos;
in vec4 gl_FragCoord;

out vec4 p3d_FragColor;

void main()
{
    if (dot(p3d_ClipPlane[0], vpos) < 0) {
        discard;
    }
    if (dot(p3d_ClipPlane[1], vpos) < 0) {
        discard;
    }
    vec2 resolution = vec2(2050,2050);
    vec2 uv = gl_FragCoord.xy / resolution;
    p3d_FragColor = texture(p3d_Texture0,uv);
};
