#include "SLAMTrajectoryDrawer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>

#include <iostream>
#include <cstring>

#ifndef OPENGL_MAJOR_VERSION
#define OPENGL_MAJOR_VERSION 4
#endif

#ifndef OPENGL_MINOR_VERSION
#define OPENGL_MINOR_VERSION 2
#endif

namespace {

GLuint position_loc, color_in_loc, MVP_loc, Rt_loc;
GLuint position_loc2, color_in_loc2, MVP_loc2;

static const GLfloat node[] = { 0.0f, 0.0f, 0.0f,
                                0.25f, 0.5f, -0.25f,
                                0.25f, 0.5f, 0.25f,
                                0.25f, -0.5f, 0.25f,
                                0.25f, -0.5f, -0.25f,
                                0.25f, 0.5f, -0.25f };

static const GLfloat axes[] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X-axis
                                2.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y-axis
                                0.0f, 2.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // Z-axis
                                0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 1.0f };
}

float* SLAMTrajectoryDrawer::locations_;
float* SLAMTrajectoryDrawer::quanternions_;
int SLAMTrajectoryDrawer::num_of_locations_;
float* SLAMTrajectoryDrawer::locations2_;
float* SLAMTrajectoryDrawer::quanternions2_;
int SLAMTrajectoryDrawer::num_of_locations2_;
GLuint SLAMTrajectoryDrawer::program_;
GLuint SLAMTrajectoryDrawer::VAO_;
GLuint SLAMTrajectoryDrawer::VBO_;
GLuint SLAMTrajectoryDrawer::program2_;
GLuint SLAMTrajectoryDrawer::VAO2_;
GLuint SLAMTrajectoryDrawer::VBO2_;
glm::mat4 SLAMTrajectoryDrawer::model_ = glm::mat4(1.0f);
glm::mat4 SLAMTrajectoryDrawer::view_ = glm::lookAt(glm::vec3(0.0f, 0.0f, 10000.0f),
                                                    glm::vec3(0.0f, 0.0f, 0.0f),
                                                    glm::vec3(1.0f, 0.0f, 0.0f));
glm::mat4 SLAMTrajectoryDrawer::projection_;
SLAMTrajectoryDrawer::Mouse SLAMTrajectoryDrawer::mouse_;
SLAMTrajectoryDrawer::Transform SLAMTrajectoryDrawer::transform_;
SLAMTrajectoryDrawer::Window SLAMTrajectoryDrawer::window_;

void SLAMTrajectoryDrawer::SetupGLUT(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);

  glutInitContextVersion(OPENGL_MAJOR_VERSION, OPENGL_MINOR_VERSION);
  glutInitContextProfile(GLUT_CORE_PROFILE);
  glutInitWindowSize(window_.width, window_.height);
  glutInitWindowPosition(window_.pos_x, window_.pos_y);
  glutCreateWindow("Trajectory - Simple SLAM Trajectory Drawer");

  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MouseMotionFunc);
  glutDisplayFunc(DisplayFunc);
  glutReshapeFunc(ReshapeFunc);
  glutIdleFunc(IdleFunc);
}

void SLAMTrajectoryDrawer::SetupGLSL() {
  glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  program_ = glCreateProgram();
  LoadShaders("shaders/trajectory.vert", GL_VERTEX_SHADER, program_);
  LoadShaders("shaders/trajectory.frag", GL_FRAGMENT_SHADER, program_);

  program2_ = glCreateProgram();
  LoadShaders("shaders/axes.vert", GL_VERTEX_SHADER, program2_);
  LoadShaders("shaders/axes.frag", GL_FRAGMENT_SHADER, program2_);

  position_loc = glGetAttribLocation(program_, "position");
  color_in_loc = glGetUniformLocation(program_, "color_in");
  MVP_loc = glGetUniformLocation(program_, "MVP");
  Rt_loc = glGetUniformLocation(program_, "Rt");
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(node), node, GL_STATIC_DRAW);

  position_loc2 = glGetAttribLocation(program2_, "position");
  color_in_loc2 = glGetAttribLocation(program2_, "color_in");
  MVP_loc2 = glGetUniformLocation(program2_, "MVP");
  glGenVertexArrays(1, &VAO2_);
  glBindVertexArray(VAO2_);
  glGenBuffers(1, &VBO2_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO2_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(axes), axes, GL_STATIC_DRAW);
}

void SLAMTrajectoryDrawer::SetupGLEW() {
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    exit(1);
  }
}

void SLAMTrajectoryDrawer::StartDrawing() {
  glutMainLoop();
}

void SLAMTrajectoryDrawer::DisplayFunc() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::mat4 translate = glm::translate(glm::vec3(transform_.x, transform_.y, transform_.z));
  glm::mat4 pitch = glm::rotate(transform_.angle_h, glm::vec3(1.0f, 0.0f, 0.0f));
  glm::mat4 roll = glm::rotate(transform_.angle_v, glm::vec3(0.0f, 1.0f, 0.0f));
  glm::mat4 rotation = roll * pitch;
  glm::mat4 scale = glm::mat4(transform_.scale);
  scale[3][3] = 1.0f;
  glm::mat4 MVP = projection_ * view_ * translate * rotation * model_ * scale;

  /* Draw the camera poses */
  glLineWidth(2.0f);
  glUseProgram(program_);
  glUniformMatrix4fv(MVP_loc, 1, GL_FALSE, &MVP[0][0]);
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);
  float Rt[16] = {0};
  Rt[0] = Rt[5] = Rt[10] = Rt[15] = 1.0f;
  for (int i = 0; i < num_of_locations_; i += 10) {
    glm::mat3 R = glm::mat3(1.0f);
    JPLQuatToMat(&quanternions_[i * 4], &R[0][0]);
    glm::vec3 t = glm::vec3(locations_[i * 3 + 0],
                            locations_[i * 3 + 1],
                            locations_[i * 3 + 2]);
    memcpy(&Rt[0], &R[0][0], sizeof(float) * 3);
    memcpy(&Rt[4], &R[1][0], sizeof(float) * 3);
    memcpy(&Rt[8], &R[2][0], sizeof(float) * 3);
    memcpy(&Rt[12], &t[0], sizeof(float) * 3);
    Rt[15] = 1.0f;
    glUniform4f(color_in_loc, 0.6f, 0.0f, 0.0f, 1.0f);
    glUniformMatrix4fv(Rt_loc, 1, GL_FALSE, Rt);
    glDrawArrays(GL_TRIANGLE_FAN, 0, sizeof(node) / sizeof(GLfloat) / 3);
  }
  for (int i = 0; i < num_of_locations2_; ++i) {
    glm::mat3 R = glm::mat3(1.0f);
    JPLQuatToMat(&quanternions2_[i * 4], &R[0][0]);
    glm::vec3 t = glm::vec3(locations2_[i * 3 + 0],
                            locations2_[i * 3 + 1],
                            locations2_[i * 3 + 2]);
    memcpy(&Rt[0], &R[0][0], sizeof(float) * 3);
    memcpy(&Rt[4], &R[1][0], sizeof(float) * 3);
    memcpy(&Rt[8], &R[2][0], sizeof(float) * 3);
    memcpy(&Rt[12], &t[0], sizeof(float) * 3);
    Rt[15] = 1.0f;
    glUniform4f(color_in_loc, 0.0f, 0.6f, 0.0f, 1.0f);
    glUniformMatrix4fv(Rt_loc, 1, GL_FALSE, Rt);
    glDrawArrays(GL_TRIANGLE_FAN, 0, sizeof(node) / sizeof(GLfloat) / 3);
  }
  glDisableVertexAttribArray(position_loc);

  /* Draw the axes */
  glLineWidth(4.0f);
  glUseProgram(program2_);
  glUniformMatrix4fv(MVP_loc2, 1, GL_FALSE, &MVP[0][0]);

  glEnableVertexAttribArray(position_loc2);
  glBindBuffer(GL_ARRAY_BUFFER, VBO2_);
  glVertexAttribPointer(position_loc2, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);

  glEnableVertexAttribArray(color_in_loc2);
  glBindBuffer(GL_ARRAY_BUFFER, VBO2_);
  glVertexAttribPointer(color_in_loc2, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, (void*)(sizeof(GLfloat) * 3));

  glDrawArrays(GL_LINES, 0, 6);

  glDisableVertexAttribArray(color_in_loc2);
  glDisableVertexAttribArray(position_loc2);

  glutSwapBuffers();
}

void SLAMTrajectoryDrawer::ReshapeFunc(int w, int h) {
  glViewport(0, 0, w, h);
  window_.width = w;
  window_.height = h;
  projection_ = glm::ortho(-w / 2.0f, w / 2.0f, -h / 2.0f, h / 2.0f, 0.1f, 100000.0f);
}

void SLAMTrajectoryDrawer::KeyboardFunc(unsigned char key, int /* x */, int /* y */) {
  switch (key) {
    case 'a' : { transform_.auto_rotation ^= 1; } break;
    case 'q' :
    case 27  : { exit(0); } break;
  }
}

void SLAMTrajectoryDrawer::MouseFunc(int button,int state,int x,int y) {
  switch (button) {
    case GLUT_LEFT_BUTTON : {
      if (state == GLUT_DOWN) {
        if (!mouse_.left_pressed) {
          mouse_.x = x;
          mouse_.y = y;
        }
        mouse_.left_pressed = true;
      } else if (state == GLUT_UP) {
        mouse_.left_pressed = false;
      }
    } break;
    case GLUT_RIGHT_BUTTON : {
      if (state == GLUT_DOWN) {
        if (!mouse_.right_pressed) {
          mouse_.x = x;
          mouse_.y = y;
        }
        mouse_.right_pressed = true;
      } else if (state == GLUT_UP) {
        mouse_.right_pressed = false;
      }
    } break;
    case GLUT_MIDDLE_BUTTON : {
      if (state == GLUT_DOWN) {
        if (!mouse_.middle_pressed) {
          mouse_.x = x;
          mouse_.y = y;
        }
        mouse_.middle_pressed = true;
      } else if (state == GLUT_UP) {
        mouse_.middle_pressed = false;
      }
    } break;
  }
}

void SLAMTrajectoryDrawer::MouseMotionFunc(int x, int y) {
  if (mouse_.left_pressed) {
    transform_.angle_h += (x - mouse_.x) / 500.0f;
    transform_.angle_v += (mouse_.y - y) / 500.0f;
    mouse_.x = x;
    mouse_.y = y;
  } else if (mouse_.right_pressed) {
    transform_.y -= (x - mouse_.x) / 1.0f;
    transform_.x += (mouse_.y - y) / 1.0f;
    mouse_.x = x;
    mouse_.y = y;
  } else if (mouse_.middle_pressed) {
    transform_.scale += (x - mouse_.x) / 10.0f;
    if (transform_.scale < 0.1f) transform_.scale = 0.1f;
    mouse_.x = x;
    mouse_.y = y;
  }
  if (mouse_.left_pressed || mouse_.right_pressed || mouse_.middle_pressed)
    glutPostRedisplay();
}

void SLAMTrajectoryDrawer::IdleFunc() {
  if (transform_.auto_rotation && !mouse_.left_pressed) {
    transform_.angle_h = transform_.angle_h == 0 ? 359 : transform_.angle_h - 1 / 100.0f;
    glutPostRedisplay();
  }
}
