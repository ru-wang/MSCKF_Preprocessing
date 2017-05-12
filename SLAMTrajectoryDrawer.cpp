#include "SLAMTrajectoryDrawer.h"
#include "Utils.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>

#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <vector>

#ifndef OPENGL_MAJOR_VERSION
#define OPENGL_MAJOR_VERSION 3
#endif

#ifndef OPENGL_MINOR_VERSION
#define OPENGL_MINOR_VERSION 3
#endif

namespace {

GLuint position_loc0, color_in_loc0, MVP_loc0;
GLuint position_loc1, color_in_loc1, MVP_loc1, Rt_loc1;
GLuint position_loc2, color_in_loc2, MVP_loc2;

static const GLfloat node[] = {  0.0f,  0.0f, 0.0f,
                                 1.0f, -0.5f, 0.5f,
                                -1.0f, -0.5f, 0.5f,
                                -1.0f,  0.5f, 0.5f,
                                 1.0f,  0.5f, 0.5f,
                                 1.0f, -0.5f, 0.5f };
/*
static const GLfloat node[] = { 0.0f,  0.0f,  0.0f,
                                0.5f, -1.0f,  0.5f,
                                0.5f, -1.0f, -0.5f,
                                0.5f,  1.0f, -0.5f,
                                0.5f,  1.0f,  0.5f,
                                0.5f, -1.0f,  0.5f };
                                */

static const GLfloat axes[] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X-axis
                                5.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y-axis
                                0.0f, 5.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // Z-axis
                                0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f };

class RandomColorPicker {
  public:
   RandomColorPicker() : color_indicator_(0x00), uniform_(0, 0.8), mt_(0) {}
   void GenColor(float color4fv[]) {
     color_indicator_ += 0x01;
     color4fv[0] = ((color_indicator_ & 0x01) >> 0) * 0.7f;  /* R */
     color4fv[1] = ((color_indicator_ & 0x02) >> 1) * 0.7f;  /* G */
     color4fv[2] = ((color_indicator_ & 0x04) >> 2) * 0.7f;  /* B */
     color4fv[3] = 1.0f;
   }
   void GenRandomColor(float color4fv[]) {
     color4fv[0] = uniform_(mt_);  /* R */
     color4fv[1] = uniform_(mt_);  /* G */
     color4fv[2] = uniform_(mt_);  /* B */
     color4fv[3] = 1.0f;
   }
   void GenRandomColors(float color4fv[], int num) {
     for (int i = 0; i < num; ++i)
       GenRandomColor(color4fv + i * 4);
   }
  private:
   unsigned char color_indicator_;
   std::uniform_real_distribution<float> uniform_;
   std::mt19937 mt_;
};

}

float* SLAMTrajectoryDrawer::landmarks_;
float* SLAMTrajectoryDrawer::landmark_colors_;
int SLAMTrajectoryDrawer::num_of_landmarks_ = 0;
std::vector<Trajectory*> SLAMTrajectoryDrawer::trajectories_;
std::vector<GLSLParam> SLAMTrajectoryDrawer::glsl_params_;
glm::mat4 SLAMTrajectoryDrawer::model_ = glm::mat4(1.0f);
glm::mat4 SLAMTrajectoryDrawer::view_ = glm::lookAt(glm::vec3(0.0f, 0.0f, 10000.0f),
                                                    glm::vec3(0.0f, 0.0f, 0.0f),
                                                    glm::vec3(1.0f, 0.0f, 0.0f));
glm::mat4 SLAMTrajectoryDrawer::projection_;
Mouse SLAMTrajectoryDrawer::mouse_;
Transform SLAMTrajectoryDrawer::transform_;
Window SLAMTrajectoryDrawer::window_;

void SLAMTrajectoryDrawer::ReadLandmarksFrom(const std::vector<glm::vec3>& landmarks) {
  num_of_landmarks_ = landmarks.size();
  landmarks_ = new float[num_of_landmarks_ * 3];
  for (int i = 0; i < num_of_landmarks_; ++i) {
    landmarks_[i * 3 + 0] = landmarks[i].x;
    landmarks_[i * 3 + 1] = landmarks[i].y;
    landmarks_[i * 3 + 2] = landmarks[i].z;
  }
  landmark_colors_ = new float[num_of_landmarks_ * 4];
  RandomColorPicker color_picker;
  color_picker.GenRandomColors(landmark_colors_, num_of_landmarks_);
}

void SLAMTrajectoryDrawer::FreeLandmarks() {
  delete [] landmarks_;
  delete [] landmark_colors_;
  landmarks_ = nullptr;
  landmark_colors_ = nullptr;
  num_of_landmarks_ = 0;
}

void SLAMTrajectoryDrawer::ReadTrajectoryFrom(const std::vector<glm::vec3>& locations,
                                              const std::vector<glm::vec4>& quanternions) {
  assert(locations.size() != 0 && quanternions.size() != 0 && locations.size() == quanternions.size());
  Trajectory* traj = new Trajectory;
  traj->num_of_locations = locations.size();
  traj->locations = new float[traj->num_of_locations * 3];
  traj->quanternions = new float[traj->num_of_locations * 4];
  for (size_t i = 0; i < traj->num_of_locations; ++i) {
    traj->locations[i * 3 + 0] = locations[i].x;
    traj->locations[i * 3 + 1] = locations[i].y;
    traj->locations[i * 3 + 2] = locations[i].z;
    traj->quanternions[i * 4 + 0] = quanternions[i].x;
    traj->quanternions[i * 4 + 1] = quanternions[i].y;
    traj->quanternions[i * 4 + 2] = quanternions[i].z;
    traj->quanternions[i * 4 + 3] = quanternions[i].w;
  }
  trajectories_.push_back(traj);
}

void SLAMTrajectoryDrawer::ReadTrajectoryFromFile(const char filename[]) {
  assert(filename != nullptr);
  Trajectory* traj = new Trajectory;
  std::stringstream ss;
  std::string line;
  std::ifstream ifs(filename);
  while (true) {
    utils::SafelyGetLine(ifs, &line);
    if (ifs.eof()) {
      break;
    } else {
      ss << line << "\n";
      ++(traj->num_of_locations);
    }
  }
  traj->locations = new float[traj->num_of_locations * 3];
  traj->quanternions = new float[traj->num_of_locations * 4];
  for (size_t i = 0; i < traj->num_of_locations; ++i) {
    ss >> traj->locations[i * 3 + 0]
       >> traj->locations[i * 3 + 1]
       >> traj->locations[i * 3 + 2]
       >> traj->quanternions[i * 4 + 0]
       >> traj->quanternions[i * 4 + 1]
       >> traj->quanternions[i * 4 + 2]
       >> traj->quanternions[i * 4 + 3];
  }
  ifs.close();
  trajectories_.push_back(traj);
}

void SLAMTrajectoryDrawer::WriteTrajectoryToFile(const std::vector<glm::vec3>& locations,
                                                 const std::vector<glm::vec4>& quanternions,
                                                 const char filename[]) {
  assert(locations.size() != 0 && quanternions.size() != 0 && locations.size() == quanternions.size());
  assert(filename != nullptr);
  std::ofstream ofs(filename);
  size_t num_of_locations = locations.size();
  for (size_t i = 0; i < num_of_locations; ++i) {
    const glm::vec3& p = locations[i];
    const glm::vec4& q = quanternions[i];
    ofs << p.x << " " << p.y << " " << p.z << " "
        << q.x << " " << q.y << " " << q.z << " " << q.w << "\n";
  }
  ofs.close();
}

void SLAMTrajectoryDrawer::FreeTrajectory() {
  for (auto& traj : trajectories_) {
    delete traj;
    traj = nullptr;
  }
}

void SLAMTrajectoryDrawer::SetupGLUT(int& argc, char* argv[]) {
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

  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
}

void SLAMTrajectoryDrawer::SetupGLSL() {
  glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glsl_params_.push_back(GLSLParam());
  if (num_of_landmarks_) {
    GLSLParam& glsl0 = glsl_params_.back();
    glsl0.program = glCreateProgram();
    LoadShaders("shaders/landmark.vert", GL_VERTEX_SHADER, glsl0.program);
    LoadShaders("shaders/landmark.frag", GL_FRAGMENT_SHADER, glsl0.program);

    position_loc0 = glGetAttribLocation(glsl0.program, "position");
    color_in_loc0 = glGetUniformLocation(glsl0.program, "color_in");
    MVP_loc0 = glGetUniformLocation(glsl0.program, "MVP");
    glGenVertexArrays(1, glsl0.VAO);
    glBindVertexArray(glsl0.VAO[0]);
    glGenBuffers(2, glsl0.VBO);
    glBindBuffer(GL_ARRAY_BUFFER, glsl0.VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * num_of_landmarks_ * 3, landmarks_, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, glsl0.VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * num_of_landmarks_ * 4, landmark_colors_, GL_STATIC_DRAW);
  }

  glsl_params_.push_back(GLSLParam());
  GLSLParam& glsl1 = glsl_params_.back();
  glsl1.program = glCreateProgram();
  LoadShaders("shaders/trajectory.vert", GL_VERTEX_SHADER, glsl1.program);
  LoadShaders("shaders/trajectory.frag", GL_FRAGMENT_SHADER, glsl1.program);

  position_loc1 = glGetAttribLocation(glsl1.program, "position");
  color_in_loc1 = glGetUniformLocation(glsl1.program, "color_in");
  MVP_loc1 = glGetUniformLocation(glsl1.program, "MVP");
  Rt_loc1 = glGetUniformLocation(glsl1.program, "Rt");
  glGenVertexArrays(1, glsl1.VAO);
  glBindVertexArray(glsl1.VAO[0]);
  glGenBuffers(1, glsl1.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, glsl1.VBO[0]);
  glBufferData(GL_ARRAY_BUFFER, sizeof(node), node, GL_STATIC_DRAW);

  glsl_params_.push_back(GLSLParam());
  GLSLParam& glsl2 = glsl_params_.back();
  glsl2.program = glCreateProgram();
  LoadShaders("shaders/axes.vert", GL_VERTEX_SHADER, glsl2.program);
  LoadShaders("shaders/axes.frag", GL_FRAGMENT_SHADER, glsl2.program);

  position_loc2 = glGetAttribLocation(glsl2.program, "position");
  color_in_loc2 = glGetAttribLocation(glsl2.program, "color_in");
  MVP_loc2 = glGetUniformLocation(glsl2.program, "MVP");
  glGenVertexArrays(1, glsl2.VAO);
  glBindVertexArray(glsl2.VAO[0]);
  glGenBuffers(1, glsl2.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, glsl2.VBO[0]);
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

/* for GLSL functions */
void SLAMTrajectoryDrawer::ReadShaderSource(const char filename[], char* source[]) {
  std::ifstream ifs(filename);
  ifs.seekg(0, ifs.end);
  int length = ifs.tellg();
  ifs.seekg(0, ifs.beg);
  *source = new char[length + 1];
  (*source)[length] = 0;
  ifs.read(*source, length);
  ifs.close();
}

void SLAMTrajectoryDrawer::FreeShaderSource(const char source[]) {
  delete [] source;
}

void SLAMTrajectoryDrawer::LoadShaders(const char shader_name[], GLenum shader_type, GLuint program) {
  GLuint s;
  char* s_src;
  GLint compile_status;
  GLchar* info_log;
  int info_log_len;

  s = glCreateShader(shader_type);
  ReadShaderSource(shader_name, &s_src);
  glShaderSource(s, 1, const_cast<const GLchar**>(&s_src), nullptr);
  FreeShaderSource(s_src);
  glCompileShader(s);

  glGetShaderiv(s, GL_COMPILE_STATUS, &compile_status);
  if (compile_status == GL_FALSE) {
    glGetShaderiv(s, GL_INFO_LOG_LENGTH, &info_log_len);
    info_log = new GLchar[info_log_len + 1];
    glGetShaderInfoLog(s, info_log_len, nullptr, info_log);
    info_log[info_log_len] = 0;
    std::cerr << info_log << std::endl;
    delete [] info_log;
    exit(1);
  }

  glAttachShader(program, s);
  glLinkProgram(program);
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

  /* draw the landmarks */
  if (num_of_landmarks_) {
    glPointSize(1.0f);
    glUseProgram(glsl_params_[0].program);
    glUniformMatrix4fv(MVP_loc0, 1, GL_FALSE, &MVP[0][0]);
    glEnableVertexAttribArray(position_loc0);
    glBindBuffer(GL_ARRAY_BUFFER, glsl_params_[0].VBO[0]);
    glVertexAttribPointer(position_loc0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, glsl_params_[0].VBO[1]);
    glVertexAttribPointer(color_in_loc0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glDrawArrays(GL_POINTS, 0, num_of_landmarks_);
  }

  /* draw the camera poses */
  glLineWidth(2.0f);
  glUseProgram(glsl_params_[1].program);
  glUniformMatrix4fv(MVP_loc1, 1, GL_FALSE, &MVP[0][0]);
  glEnableVertexAttribArray(position_loc1);
  glBindBuffer(GL_ARRAY_BUFFER, glsl_params_[1].VBO[0]);
  glVertexAttribPointer(position_loc1, 3, GL_FLOAT, GL_FALSE, 0, 0);
  RandomColorPicker color_picker;
  float color[4];
  float Rt[16] = {0};
  Rt[0] = Rt[5] = Rt[10] = Rt[15] = 1.0f;
  for (const auto& traj : trajectories_) {
    color_picker.GenColor(color);
    for (size_t i = 0; i < traj->num_of_locations; i += 10) {
      glm::mat3 R = glm::mat3(1.0f);
      utils::JPLQuatToMat(&traj->quanternions[i * 4], &R[0][0]);
      glm::vec3 t = glm::vec3(traj->locations[i * 3 + 0],
                              traj->locations[i * 3 + 1],
                              traj->locations[i * 3 + 2]) * traj->scale;
      memcpy(&Rt[0], &R[0][0], sizeof(float) * 3);
      memcpy(&Rt[4], &R[1][0], sizeof(float) * 3);
      memcpy(&Rt[8], &R[2][0], sizeof(float) * 3);
      memcpy(&Rt[12], &t[0], sizeof(float) * 3);
      Rt[15] = 1.0f;
      glUniform4fv(color_in_loc1, 1, color);
      glUniformMatrix4fv(Rt_loc1, 1, GL_FALSE, Rt);
      glDrawArrays(GL_TRIANGLE_FAN, 0, sizeof(node) / sizeof(GLfloat) / 3);
    }
  }
  glDisableVertexAttribArray(position_loc1);

  /* draw the axes */
  glLineWidth(4.0f);
  glUseProgram(glsl_params_[2].program);
  glUniformMatrix4fv(MVP_loc2, 1, GL_FALSE, &MVP[0][0]);

  glEnableVertexAttribArray(position_loc2);
  glBindBuffer(GL_ARRAY_BUFFER, glsl_params_[2].VBO[0]);
  glVertexAttribPointer(position_loc2, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, 0);

  glEnableVertexAttribArray(color_in_loc2);
  glBindBuffer(GL_ARRAY_BUFFER, glsl_params_[2].VBO[0]);
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
    case 27  : { glutLeaveMainLoop(); } break;
  }
}

void SLAMTrajectoryDrawer::MouseFunc(int button, int state, int x, int y) {
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
    case 3 : {
      if (state == GLUT_UP) {
        trajectories_[1]->scale += 0.01;
        glutPostRedisplay();
      }
    } break;
    case 4 : {
      if (state == GLUT_UP) {
        trajectories_[1]->scale -= 0.01;
        glutPostRedisplay();
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
