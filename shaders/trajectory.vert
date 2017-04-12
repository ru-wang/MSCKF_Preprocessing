#version 330 core
in vec3 position;

uniform mat4 MVP;
uniform mat4 Rt;

void main() {
  gl_Position = MVP * Rt * vec4(position, 1);
}
