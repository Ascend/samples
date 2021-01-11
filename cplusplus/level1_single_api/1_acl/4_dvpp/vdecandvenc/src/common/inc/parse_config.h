#pragma once
 
#include <string>
#include <map>
using namespace std;
  
bool ReadConfig(map<string, string>& config,  const char* configFile);
void PrintConfig(const map<string, string> & m);

