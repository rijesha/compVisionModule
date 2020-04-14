#ifndef CVM_ARGUMENT_PARSER_H
#define CVM_ARGUMENT_PARSER_H

#include <argparse.hpp>
#include <cstring>
#include <string>
#include <iostream>
using namespace std;


class CVMArgumentParser
{
private:
 
public:
    CVMArgumentParser(int argc, const char** argv, bool requireCalib, bool requireOutputFile, bool requireInputFile, bool requireSize);
    string deviceID;
    int height;
    int width;
    bool saveData;
    bool saveVideo;
    bool saveTiming;
    bool quiet;
    string calib_file_path;
    string inputpath;
    string outputpath;
};

#endif /* CVM_ARGUMENT_PARSER_H */