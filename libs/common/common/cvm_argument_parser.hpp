#pragma once

#include <argparse.hpp>
#include <cstring>
#include <iostream>
#include <string>
using namespace std;

class CVMArgumentParser {
 private:
 public:
  CVMArgumentParser(int argc, const char** argv, bool requireCalib,
                    bool requireOutputFile, bool requireInputFile,
                    bool requireSize);
  string deviceID;
  int height;
  int width;
  bool saveData;
  bool saveVideo;
  bool saveTiming;
  bool quiet;
  string calib_file_path_1;
  string calib_file_path_2;
  string fisheye_calib_file_path;
  string inputpath;
  string outputpath;
};
