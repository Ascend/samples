#ifndef _GET_CONFIG_H_
#define _GET_CONFIG_H_
 
#include <string>
#include <map>
using namespace std;
  
bool ReadConfig(map<string, string>& config,  const char* configFile);
void PrintConfig(const map<string, string> & m);
#endif
