#include "parse_config.h"

#include <fstream>
#include <iostream>
#include <string>
#include <map>

using namespace std;

#define COMMENT_CHAR '#'
#define EQUALS_CHAR  '='
#define BLANK_SPACE_CHAR ' '
#define TABLE_CHAR '\t'

bool IsSpace(char c)
{
	return (BLANK_SPACE_CHAR == c || TABLE_CHAR == c);
}

void Trim(string& str)
{
    if (str.empty()) {
        return;
    }
    uint32_t i, start_pos, end_pos;
    for (i = 0; i < str.size(); ++i) {
        if (!IsSpace(str[i])) {
            break;
        }
    }
    if (i == str.size()) { // is all blank space
        str = "";
        return;
    }
    
    start_pos = i;
    
    for (i = str.size() - 1; i >= 0; --i) {
        if (!IsSpace(str[i])) {
            break;
        }
    }
    end_pos = i;
    
    str = str.substr(start_pos, end_pos - start_pos + 1);
}

bool AnalyseLine(const string & line, string & key, string & value)
{
    if (line.empty())
        return false;
    int start_pos = 0, end_pos = line.size() - 1, pos;
    if ((pos = line.find(COMMENT_CHAR)) != -1) {
        if (0 == pos) {  //the first charactor is #
            return false;
        }
        end_pos = pos - 1;
    }
    string new_line = line.substr(start_pos, start_pos + 1 - end_pos);  // delete comment
    
    if ((pos = new_line.find(EQUALS_CHAR)) == -1)
        return false;  // has no =
        
    key = new_line.substr(0, pos);
    value = new_line.substr(pos + 1, end_pos + 1- (pos + 1));
    
    Trim(key);
    if (key.empty()) {
        return false;
    }
    Trim(value);
    return true;
}

bool ReadConfig(map<string, string>& config, const char* configFile)
{
	config.clear();
    ifstream infile(configFile);
    if (!infile) {
        cout << "file open error" << endl;
        return false;
    }
    string line, key, value;
    while (getline(infile, line)) {
        if (AnalyseLine(line, key, value)) {
			config[key] = value;
        }
    }
    
    infile.close();
    return true;
}

void PrintConfig(const map<string, string>& config)
{
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        cout << mIter->first << "=" << mIter->second << endl;
    }
}
