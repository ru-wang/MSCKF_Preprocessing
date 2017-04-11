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
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

class SLAMTrajectoryDrawer {
 public:
  static void ReadTrajectoryFrom(
      const std::vector<glm::vec3>& locations,
      const std::vector<glm::vec4>& quanternions,
      const std::vector<glm::vec3>& locations2 = std::vector<glm::vec3>(),
      const std::vector<glm::vec4>& quanternions2 = std::vector<glm::vec4>()) {
    assert(locations.size() != 0 && quanternions.size() != 0 && locations.size() == quanternions.size());
    num_of_locations_ = locations.size();
    locations_ = new float[num_of_locations_ * 3];
    quanternions_ = new float[num_of_locations_ * 4];
    for (int i = 0; i < num_of_locations_; ++i) {
      locations_[i * 3 + 0] = locations[i].x;
      locations_[i * 3 + 1] = locations[i].y;
      locations_[i * 3 + 2] = locations[i].z;
      quanternions_[i * 4 + 0] = quanternions[i].x;
      quanternions_[i * 4 + 1] = quanternions[i].y;
      quanternions_[i * 4 + 2] = quanternions[i].z;
      quanternions_[i * 4 + 3] = quanternions[i].w;
    }
    
    if (locations2.size() != 0 && quanternions2.size() != 0 && locations2.size() == quanternions2.size()) {
      num_of_locations2_ = locations2.size();
      locations2_ = new float[num_of_locations_ * 3];
      quanternions2_ = new float[num_of_locations_ * 4];
      for (int i = 0; i < num_of_locations2_; ++i) {
        locations2_[i * 3 + 0] = locations2[i].x;
        locations2_[i * 3 + 1] = locations2[i].y;
        locations2_[i * 3 + 2] = locations2[i].z;
        quanternions2_[i * 4 + 0] = quanternions2[i].x;
        quanternions2_[i * 4 + 1] = quanternions2[i].y;
        quanternions2_[i * 4 + 2] = quanternions2[i].z;
        quanternions2_[i * 4 + 3] = quanternions2[i].w;
      }
    }
  }

  static void ReadTrajectoryFromFile(const char filename[], const char filename2[] = nullptr) {
    assert(filename != nullptr);

    num_of_locations_ = 0;
    std::stringstream ss;
    std::string line;
    std::ifstream ifs(filename);
    while (true) {
      SafelyGetLine(ifs, line);
      if (ifs.eof()) {
        break;
      } else {
        ss << line << "\n";
        ++num_of_locations_;
      }
    }
    locations_ = new float[num_of_locations_ * 3];
    quanternions_ = new float[num_of_locations_ * 4];
    for (int i = 0; i < num_of_locations_; ++i) {
      ss >> locations_[i * 3 + 0]
         >> locations_[i * 3 + 1]
         >> locations_[i * 3 + 2]
         >> quanternions_[i * 4 + 0]
         >> quanternions_[i * 4 + 1]
         >> quanternions_[i * 4 + 2]
         >> quanternions_[i * 4 + 3];
    }
    ifs.close();

    if (filename2) {
      num_of_locations2_ = 0;
      std::stringstream ss;
      std::string line;
      std::ifstream ifs(filename2);
      while (true) {
        SafelyGetLine(ifs, line);
        if (ifs.eof()) {
          break;
        } else {
          ss << line << "\n";
          ++num_of_locations2_;
        }
      }
      locations2_ = new float[num_of_locations2_ * 3];
      quanternions2_ = new float[num_of_locations2_ * 4];
      for (int i = 0; i < num_of_locations2_; ++i) {
        ss >> locations2_[i * 3 + 0]
           >> locations2_[i * 3 + 1]
           >> locations2_[i * 3 + 2]
           >> quanternions2_[i * 4 + 0]
           >> quanternions2_[i * 4 + 1]
           >> quanternions2_[i * 4 + 2]
           >> quanternions2_[i * 4 + 3];
      }
      ifs.close();
    }
  }

  static void FreeTrajectory() {
    delete [] locations_;
    delete [] quanternions_;
    locations_ = nullptr;
    quanternions_ = nullptr;
    num_of_locations_ = 0;

    delete [] locations2_;
    delete [] quanternions2_;
    locations2_ = nullptr;
    quanternions2_ = nullptr;
    num_of_locations2_ = 0;
  }

  static void SetupGLUT(int argc, char* argv[]);
  static void SetupGLEW();
  static void SetupGLSL();
  static void StartDrawing();

  static int num_of_locations() { return num_of_locations_; }
  static int num_of_locations2() { return num_of_locations2_; }

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

  static void LoadShaders(const char shader_name[], GLenum shader_type, GLuint program) {
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

  static void JPLQuatToMat(const float quanternion[], float matrix[]) {
    /* Matrice are assigned in coloumn major order */
    glm::vec4 q_bar = glm::normalize(glm::vec4(quanternion[0],
                                               quanternion[1],
                                               quanternion[2],
                                               quanternion[3]));
    const float q1 = q_bar.x, q2 = q_bar.y, q3 = q_bar.z, q4 = q_bar.w;
    glm::mat3x4 PHI = glm::mat3x4(q4, -q3, q2, -q1,
                                  q3, q4, -q1, -q2,
                                  -q2, q1, q4, -q3);
    glm::mat3x4 THETA = glm::mat3x4(q4, q3, -q2, -q1,
                                    -q3, q4, q1, -q2,
                                    q2, -q1, q4, -q3);
    glm::mat3 rot = glm::transpose(THETA) * PHI;
    memcpy(matrix, &rot[0][0], sizeof(float) * 9);
  }

 private:
  static float* locations_;
  static float* quanternions_;
  static int num_of_locations_;

  static float* locations2_;
  static float* quanternions2_;
  static int num_of_locations2_;

  /* GLSL members */
  static GLuint program_;
  static GLuint VAO_;
  static GLuint VBO_;

  static GLuint program2_;
  static GLuint VAO2_;
  static GLuint VBO2_;

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
                  x(0), y(0), z(0), scale(1) {}
    bool auto_rotation;
    float angle_v;
    float angle_h;
    float x;
    float y;
    float z;
    float scale;
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
