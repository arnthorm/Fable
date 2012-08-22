
#include <ros/ros.h>
#include <vector>

bool directoryExists(const char* pzPath);
std::string random_filename();
void createDirectory(std::string path);
int findIdx(std::vector<std::string>, std::string);
std::vector<std::string> split(const std::string &s, char delim);
