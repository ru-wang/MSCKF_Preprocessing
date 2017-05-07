#version 330 core
#extension GL_ARB_separate_shader_objects : require
layout (location = 0) in vec4 color_in;
out vec4 color;

void main() {
  color = color_in;
}
