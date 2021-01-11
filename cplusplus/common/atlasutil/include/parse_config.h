#pragma once
 
#include <string>
#include <map>
  
bool ReadConfig(std::map<std::string, std::string>& config,  
                const char* configFile);
void PrintConfig(const std::map<std::string, std::string> & m);

