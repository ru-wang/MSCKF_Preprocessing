#ifndef MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_
#define MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>

class SLAMTrajectoryDrawer {
 public:
  static void ReadTractoryFromFile(const char filename[]) {
    num_of_locations_ = 0;
    std::stringstream ss;
    std::string line;
    std::ifstream ifs(filename);
    while (true) {
      SafelyGetLine(ifs, line);
      if (ifs.eof()) {
        break;
      } else {
        ss << line;
        ++num_of_locations_;
      }
    }
    locations_ = new float[num_of_locations_ * 3];
    quaternions_ = new float[num_of_locations_ * 4];
    for (int i = 0; i < num_of_locations_; ++i) {
      ss >> locations_[i * 3 + 0]
         >> locations_[i * 3 + 1]
         >> locations_[i * 3 + 2]
         >> quaternions_[i * 4 + 0]
         >> quaternions_[i * 4 + 1]
         >> quaternions_[i * 4 + 2]
         >> quaternions_[i * 4 + 3];
    }
    ifs.close();
  }

  static void FreeTractory() {
    delete [] locations_;
    delete [] quaternions_;
    locations_ = nullptr;
    quaternions_ = nullptr;
  }

  static void SetupGLUT(int argc, char* argv[]);
  static void SetupGLEW();
  static void SetupGLSL();
  static void StartDrawing();

 private:
  /* For GLSL functions */
  static void ReadShaderSource(const char filename[], char* source[]) {
    std::ifstream ifs(filename);
    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    *source = new char[length + 1];
    (*source)[length] = 0;
    ifs.read(*source, length);
    ifs.close();
  }

  static void FreeShaderSource(const char source[]) {
    delete [] source;
  }

  static void LoadShaders() {
    GLuint v, f;
    char *vs, *fs;
    v = glCreateShader(GL_VERTEX_SHADER);
    f = glCreateShader(GL_FRAGMENT_SHADER); 
    ReadShaderSource("shaders/trajectory.vert", &vs);
    ReadShaderSource("shaders/trajectory.frag", &fs);
    glShaderSource(v, 1, &vs, nullptr);
    glShaderSource(f, 1, &fs, nullptr);
    FreeShaderSource(vs);
    FreeShaderSource(fs);
    glCompileShader(v);
    glCompileShader(f);

    GLint compile_status;
    GLchar* info_log;
    int info_log_len;

    glGetShaderiv(v, GL_COMPILE_STATUS, &compile_status);
    if (compile_status == GL_FALSE) {
      glGetShaderiv(v, GL_INFO_LOG_LENGTH, &info_log_len);
      info_log = new GLchar[info_log_len + 1];
      glGetShaderInfoLog(v, info_log_len, nullptr, info_log);
      info_log[info_log_len] = 0;
      std::cerr << info_log << std::endl;
      delete [] info_log;
      exit(1);
    }

    glGetShaderiv(f, GL_COMPILE_STATUS, &compile_status);
    if (compile_status == GL_FALSE) {
      glGetShaderiv(f, GL_INFO_LOG_LENGTH, &info_log_len);
      info_log = new GLchar[info_log_len + 1];
      glGetShaderInfoLog(v, info_log_len, nullptr, info_log);
      info_log[info_log_len] = 0;
      std::cerr << info_log << std::endl;
      delete [] info_log;
      exit(1);
    }

    program_ = glCreateProgram();
    glAttachShader(program_, v);
    glAttachShader(program_, f);
    glLinkProgram(program_);
    glUseProgram(program_);
  }

  /* For GLUT functions */
  static void DisplayFunc();
  static void ReshapeFunc(int w, int h);
  static void KeyboardFunc(unsigned char key, int x, int y);
  static void SpecialKeyFunc(int key, int x, int y);
  static void MouseFunc(int button,int state,int x,int y);
  static void MouseMotionFunc(int x, int y);
  static void IdleFunc();

  static std::istream& SafelyGetLine(std::istream& is, std::string& str) {
    str.clear();
    std::streambuf* sb = is.rdbuf();
    while (true) {
      char ch = sb->sbumpc();
      switch (ch) {
        case '\n': return is;
        case '\r': if (sb->sgetc() == '\n')
                     sb->sbumpc();
                   return is;
        case EOF:  if (str.empty())
                     is.setstate(std::ifstream::eofbit);
                   return is;
        default:   str += ch;
      }
    }
  }

  static void QuatToMat(const float quanternion[], float matrix[]) {
    const float x = quanternion[0];
    const float y = quanternion[1];
    const float z = quanternion[2];
    const float w = quanternion[3];

    /* Matrice are assigned in coloumn major order */
    matrix[0] = 1 - 2 * (y * y + z * z);
    matrix[1] = 2 * (x * y + z * w);
    matrix[2] = 2 * (x * z - y * w);
    matrix[3] = 2 * (x * y - z * w);
    matrix[4] = 1 - 2 * (x * x + z * z);
    matrix[5] = 2 * (y * z + x * w);
    matrix[6] = 2 * (x * z + y * w);
    matrix[7] = 2 * (y * z - x * w);
    matrix[8] = 1 - 2 * (x * x + y * y);
  }

 private:
  static float* locations_;
  static float* quaternions_;
  static int num_of_locations_;

  /* GLSL members */
  static GLuint program_;
  static GLuint VAO_;
  static GLuint VBO_;

  /* Matrix */
  static glm::mat4 model_;
  static glm::mat4 view_;
  static glm::mat4 projection_;

  /* UI members */
  static struct Mouse {
    Mouse() : left_pressed(false),
              right_pressed(false),
              middle_pressed(false),
              x(0), y(0) {}
    bool left_pressed;
    bool right_pressed;
    bool middle_pressed;
    int x;
    int y;
  } mouse_;

  static struct Transform {
    Transform() : auto_rotation(false),
                  angle_v(0), angle_h(0),
                  x(0), y(0), z(0) {}
    bool auto_rotation;
    float angle_v;
    float angle_h;
    float x;
    float y;
    float z;
  } transform_;

  static struct Window {
    Window() : width(1200), height(900),
               pos_x(100), pos_y(100) {}
    float width;
    float height;
    float pos_x;
    float pos_y;
  } window_;

 /* Singleton implementation */
 private:
  SLAMTrajectoryDrawer();
  ~SLAMTrajectoryDrawer();
  SLAMTrajectoryDrawer(const SLAMTrajectoryDrawer& other);
  void operator=(const SLAMTrajectoryDrawer& other);
};

#endif  // MSCKF_POSTPROCESSING_SLAMTRAJECTORY_DRAWER_H_
