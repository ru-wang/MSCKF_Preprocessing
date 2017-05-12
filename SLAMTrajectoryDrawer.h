#ifndef MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_
#define MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include <cassert>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

/* UI stuff */
struct Mouse {
  Mouse() : left_pressed(false),
            right_pressed(false),
            middle_pressed(false),
            x(0), y(0) {}
  bool left_pressed;
  bool right_pressed;
  bool middle_pressed;
  int x;
  int y;
};

struct Transform {
  Transform() : auto_rotation(false),
                angle_v(0), angle_h(0),
                x(0), y(0), z(0), scale(1) {}
  bool auto_rotation;
  float angle_v;
  float angle_h;
  float x;
  float y;
  float z;
  float scale;
};

struct Window {
  Window() : width(1200), height(900),
             pos_x(100), pos_y(100) {}
  float width;
  float height;
  float pos_x;
  float pos_y;
};

/* Trajectory structure */
struct Trajectory {
  Trajectory() : locations(nullptr), quanternions(nullptr), num_of_locations(0), scale(1) {}
  ~Trajectory() {
    if (locations) delete [] locations;
    if (quanternions) delete [] quanternions;
    locations = nullptr;
    quanternions = nullptr;
    num_of_locations = 0;
  }
  float* locations;
  float* quanternions;
  size_t num_of_locations;
  float scale;
};

/* GLSL parameter structure */
struct GLSLParam {
  GLuint program;
  GLuint VAO[10];
  GLuint VBO[10];
};

class SLAMTrajectoryDrawer {
 public:
  static void ReadLandmarksFrom(const std::vector<glm::vec3>& landmarks);
  static void ReadLandmarksFromFile(const char filename[]);
  static void FreeLandmarks();
  static void WriteLandmarksToFile(const std::vector<glm::vec3>& landmarks,
                                   const char filename[]);

  static void ReadTrajectoryFrom(const std::vector<glm::vec3>& locations,
                                 const std::vector<glm::vec4>& quanternions);
  static void ReadTrajectoryFromFile(const char filename[]);
  static void FreeTrajectory();
  static void WriteTrajectoryToFile(const std::vector<glm::vec3>& locations,
                                    const std::vector<glm::vec4>& quanternions,
                                    const char filename[]);

  static void SetupGLUT(int& argc, char* argv[]);
  static void SetupGLEW();
  static void SetupGLSL();
  static void StartDrawing();

 private:
  /* for GLSL functions */
  static void ReadShaderSource(const char filename[], char* source[]);
  static void FreeShaderSource(const char source[]);
  static void LoadShaders(const char shader_name[], GLenum shader_type, GLuint program);

  /* for GLUT functions */
  static void DisplayFunc();
  static void ReshapeFunc(int w, int h);
  static void KeyboardFunc(unsigned char key, int x, int y);
  static void SpecialKeyFunc(int key, int x, int y);
  static void MouseFunc(int button, int state, int x, int y);
  static void MouseMotionFunc(int x, int y);
  static void IdleFunc();

 private:
  /* Landmarks */
  static float* landmarks_;
  static float* landmark_colors_;
  static int num_of_landmarks_;

  /* Trajectory */
  static std::vector<Trajectory*> trajectories_;

  /* GLSL parameters */
  static std::vector<GLSLParam> glsl_params_;

  /* matrix */
  static glm::mat4 model_;
  static glm::mat4 view_;
  static glm::mat4 projection_;

  /* UI members */
  static Mouse mouse_;
  static Transform transform_;
  static Window window_;

 /* singleton implementation */
 private:
  SLAMTrajectoryDrawer();
  ~SLAMTrajectoryDrawer();
  SLAMTrajectoryDrawer(const SLAMTrajectoryDrawer& other);
  void operator=(const SLAMTrajectoryDrawer& other);
};

#endif  // MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_
