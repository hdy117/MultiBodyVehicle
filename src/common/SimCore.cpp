#include "SimCore.h"
#include <fstream>

namespace tx_car {
/* dump string to file*/
bool dumpToFile(const std::string &content, const std::string &file_path) {
  std::ofstream out(file_path, std::ios::out);

  if (out.is_open()) {
    out << content;
    out.flush();
    out.close();
    LOG_INFO << "dump to file done " << file_path << ".\n";
  } else {
    LOG_ERROR << "fail to dump " << file_path << "\n";
    return false;
  }

  return true;
}

/* load string from file path*/
bool loadFromFile(std::string &content, const std::string &file_path) {
  std::ifstream in(file_path, std::ios::in);
  content.clear();

  if (in.is_open()) {
    content = std::string((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
    LOG_INFO << "json content:" << content << "\n";
    in.close();
    return true;
  }

  return false;
}
} // namespace tx_car