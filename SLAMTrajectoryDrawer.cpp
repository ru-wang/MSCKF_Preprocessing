#include "SLAMTrajectoryDrawer.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include <iostream>
#include <cstring>

#ifndef OPENGL_MAJOR_VERSION
#define OPENGL_MAJOR_VERSION 3
#endif

#ifndef OPENGL_MINOR_VERSION
#define OPENGL_MINOR_VERSION 3
#endif

namespace {

GLuint position_loc, MVP_loc, RT_loc;

static const GLfloat node[] = { -1.0f, 1.0f, 0.0f,
                                -1.0f, -1.0f, 0.0f,
                                0.0f, 0.0f, 1.0f,
                                -1.0f, 1.0f, 0.0f,
                                1.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 1.0f,
                                1.0f, -1.0f, 0.0f,
                                -1.0f, -1.0f, 0.0f,
                                1.0f, 1.0f, 0.0f,
                                1.0f, -1.0f, 0.0f };

}

float* SLAMTrajectoryDrawer::locations_;
float* SLAMTrajectoryDrawer::quaternions_;
int SLAMTrajectoryDrawer::num_of_locations_;
GLuint SLAMTrajectoryDrawer::program_;
GLuint SLAMTrajectoryDrawer::VAO_;
GLuint SLAMTrajectoryDrawer::VBO_;
glm::mat4 SLAMTrajectoryDrawer::model_ = glm::mat4(1.0f);
glm::mat4 SLAMTrajectoryDrawer::view_ = glm::lookAt(glm::vec3(0.0f, 0.0f, 20.0f),
                                                    glm::vec3(0.0f, 0.0f, 0.0f),
                                                    glm::vec3(0.0f, 1.0f, 0.0f));
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
  glClearColor(0.6f, 0.6f, 0.6f, 1.0f);

  LoadShaders();

  position_loc = glGetAttribLocation(program_, "position");
  MVP_loc = glGetUniformLocation(program_, "MVP");
  RT_loc = glGetUniformLocation(program_, "RT");

  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(node), node, GL_STATIC_DRAW);
}

void SLAMTrajectoryDrawer::SetupGLEW() {
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    fprintf(stderr, "Failed to initialize GLEW\n");
    exit(1);
  }
}

void SLAMTrajectoryDrawer::StartDrawing() {
  glutMainLoop();
}

void SLAMTrajectoryDrawer::DisplayFunc() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::mat4 translate = glm::translate(glm::vec3(transform_.x, transform_.y, 0));
  glm::mat4 pitch = glm::rotate(transform_.angle_h, glm::vec3(0.0f, 1.0f, 0.0f));
  glm::mat4 roll = glm::rotate(transform_.angle_v, glm::vec3(-1.0f, 0.0f, 0.0f));
  glm::mat4 rotation = roll * pitch;
  glm::mat4 scale = glm::mat4(1.0f + transform_.z);
  scale[3][3] = 1.0;

  glm::mat4 MVP = projection_ * view_ * translate * model_ * rotation * scale;
  glUniformMatrix4fv(MVP_loc, 1, GL_FALSE, &MVP[0][0]);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  float RT[16] = {0};
  for (int i = 0; i < num_of_locations_; ++i) {
    float R[9];
    QuatToMat(&quaternions_[i * 4], R);
    memcpy(&RT[0 * 4], &R[0 * 3], sizeof(float) * 3);
    memcpy(&RT[1 * 4], &R[1 * 3], sizeof(float) * 3);
    memcpy(&RT[2 * 4], &R[2 * 3], sizeof(float) * 3);
    memcpy(&RT[3 * 4], &locations_[i * 3], sizeof(float) * 3);
    RT[15] = 1.0;
    glUniformMatrix4fv(RT_loc, 1, GL_FALSE, RT);
    glDrawArrays(GL_LINE_LOOP, 0, sizeof(node) / sizeof(GLfloat) / 3);
  }

  glDisableVertexAttribArray(position_loc);

  glutSwapBuffers();
}

void SLAMTrajectoryDrawer::ReshapeFunc(int w, int h) {
  glViewport(0, 0, w, h);
  window_.width = w;
  window_.height = h;
  //projection_ = glm::ortho(-w / 10.0f, w / 10.0f, -h / 10.0f, h / 10.0f, 0.1f, 100.0f);
  projection_ = glm::perspective(80.0f, (float)w / (float)h, 0.1f, 100.0f);
  projection_[0][0] *= -1;
  projection_[1][1] *= -1;
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
    transform_.angle_h += (x - mouse_.x) / 100.0;
    transform_.angle_v -= (y - mouse_.y) / 100.0;
    while (transform_.angle_h >= 360) transform_.angle_h -= 360;
    while (transform_.angle_v >= 360) transform_.angle_v -= 360;
    while (transform_.angle_h < 0) transform_.angle_h += 360;
    while (transform_.angle_v < 0) transform_.angle_v += 360;
    mouse_.x = x;
    mouse_.y = y;
  } else if (mouse_.right_pressed) {
    transform_.x += (x - mouse_.x) / 100.0;
    transform_.y -= (y - mouse_.y) / 100.0;
    mouse_.x = x;
    mouse_.y = y;
  } else if (mouse_.middle_pressed) {
    transform_.z += (x - mouse_.x) / 100.0;
    if (transform_.z < -0.9) transform_.z = -0.9;
    mouse_.x = x;
    mouse_.y = y;
  }

  if (mouse_.left_pressed || mouse_.right_pressed || mouse_.middle_pressed)
    glutPostRedisplay();
}

void SLAMTrajectoryDrawer::IdleFunc() {
  if (transform_.auto_rotation && !mouse_.left_pressed) {
    transform_.angle_h = transform_.angle_h == 0 ? 359 : transform_.angle_h - 1 / 100.0;
    glutPostRedisplay();
  }
}

/* Unit test */
int main(int argc, char* argv[]) {
  char* tractory_file = argv[1];

  SLAMTrajectoryDrawer::ReadTractoryFromFile(tractory_file);

  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);

  SLAMTrajectoryDrawer::SetupGLEW();

  SLAMTrajectoryDrawer::SetupGLSL();

  SLAMTrajectoryDrawer::StartDrawing();

  SLAMTrajectoryDrawer::FreeTractory();

  return 0;
}
