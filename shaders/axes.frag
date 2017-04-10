#version 420 core
layout (location = 0) in vec4 color_in;
out vec4 color;

void main() {
  color = color_in;
}
