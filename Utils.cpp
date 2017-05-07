#include "Utils.h"

#include <boost/algorithm/string.hpp>

#include <istream>
#include <streambuf>
#include <string>
#include <vector>

using namespace std;
namespace bs = ::boost;

namespace utils {

const double EARTH_RADIUS = 6378137;
const double PI = 3.14159;

template<>
vector<string> SplitToVector<string>(const string& str,
                                     const string& delimiters,
                                     const bool token_compress) {
  vector<string> strs;
  bs::split(strs, str, bs::is_any_of(delimiters),
            token_compress ? bs::token_compress_on :
                             bs::token_compress_off);
  return strs;
}

template<>
vector<int> SplitToVector<int>(const string& str,
                               const string& delimiters,
                               const bool token_compress) {
  vector<string> strs;
  bs::split(strs, str, bs::is_any_of(delimiters),
            token_compress ? bs::token_compress_on :
                             bs::token_compress_off);
  vector<int> vector;
  for (const auto& v : strs)
    vector.push_back(stoi(v));
  return vector;
}

template<>
vector<long> SplitToVector<long>(const string& str,
                                 const string& delimiters,
                                 const bool token_compress) {
  vector<string> strs;
  bs::split(strs, str, bs::is_any_of(delimiters),
            token_compress ? bs::token_compress_on :
                             bs::token_compress_off);
  vector<long> vector;
  for (const auto& v : strs)
    vector.push_back(stol(v));
  return vector;
}

template<>
vector<float> SplitToVector<float>(const string& str,
                                   const string& delimiters,
                                   const bool token_compress) {
  vector<string> strs;
  bs::split(strs, str, bs::is_any_of(delimiters),
            token_compress ? bs::token_compress_on :
                             bs::token_compress_off);
  vector<float> vector;
  for (const auto& v : strs)
    vector.push_back(stof(v));
  return vector;
}

template<>
vector<double> SplitToVector<double>(const string& str,
                                     const string& delimiters,
                                     const bool token_compress) {
  vector<string> strs ;
  bs::split(strs, str, bs::is_any_of(delimiters),
            token_compress ? bs::token_compress_on :
                             bs::token_compress_off);
  vector<double> vector;
  for (const auto& v : strs)
    vector.push_back(stod(v));
  return vector;
}

istream& SafelyGetLine(istream& is, string* str) {
  str->clear();
  streambuf* sb = is.rdbuf();
  while (true) {
    char ch = sb->sbumpc();
    switch (ch) {
      case '\n': return is;
      case '\r': if (sb->sgetc() == '\n')
                   sb->sbumpc();
                 return is;
      case EOF:  if (str->empty())
                   is.setstate(istream::eofbit);
                 return is;
      default:   (*str) += ch;
    }
  }
}

}
