

#include "helpers.h"
#include <dirent.h>

bool directoryExists( const char* pzPath )
{
  if ( pzPath == NULL) return false;

  DIR *pDir;
  bool bExists = false;

  pDir = opendir (pzPath);

  if (pDir != NULL)
  {
      bExists = true;    
      (void) closedir (pDir);
  }

  return bExists;
}

int random(int mod)
{
    int numb = 0;
    srand((time(NULL)*mod));
    numb = rand()%25 + 97;
    return numb;
}

std::string random_filename()
{
  char str[10];
  for(int i = 0; i < 10; i++)
  {
    str[i] = random(i+1);
  }
  return std::string(str).substr(0,10);
}


int findIdx(std::vector<std::string> list, std::string value)
{
  std::vector<std::string>::iterator it;
  it = std::find(list.begin(), list.end(), value);
  if (it != list.end())
    return it - list.begin();
  else
    return -1;
}

void createDirectory(std::string path)
{
  if (!directoryExists(path.c_str()))
  {
    std::string cmd = "mkdir " + path;
    int tmp = system(cmd.c_str());
    if (tmp)
      tmp = 0;
  }
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}
