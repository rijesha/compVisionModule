#include "cvm_argument_parser.hpp"

CVMArgumentParser::CVMArgumentParser(int argc, const char** argv,
                                     bool requireCalib, bool requireOutputPath,
                                     bool requireInputPath, bool requireSize) {
  // make a new ArgumentParser
  ArgumentParser parser;
  parser.addArgument("-c", "--calib_file", 1, !requireCalib);
  parser.addArgument("-d", "--devices", 1, true);
  parser.addArgument("-h", "--height", 1, !requireSize);
  parser.addArgument("-w", "--width", 1, !requireSize);
  parser.addArgument("-o", "--output", 1, !requireOutputPath);
  parser.addArgument("-i", "--input", 1, !requireInputPath);
  parser.addArgument("-s", "--saveData", 1, true);
  parser.addArgument("-v", "--saveVideo", 1, true);
  parser.addArgument("-t", "--debugTiming", 1, true);
  parser.addArgument("-q", "--quiet", 1, true);
  parser.parse(argc, argv);

  outputpath = parser.retrieve<string>("o");

  inputpath = parser.retrieve<string>("i");

  calib_file_path = parser.retrieve<string>("c");
  cout << calib_file_path << endl;

  deviceID = "/dev/video0";
  string devicestring = parser.retrieve<string>("d");
  if (devicestring.length() != 0) deviceID = devicestring;

  height = 960;
  string heightstring = parser.retrieve<string>("h");
  if (heightstring.length() != 0) height = stoi(heightstring);

  width = 1280;
  string widthstring = parser.retrieve<string>("w");
  if (widthstring.length() != 0) width = stoi(widthstring);

  saveData = false;
  string saveDatastr = parser.retrieve<string>("s");
  if (saveDatastr.length() != 0) saveData = (stoi(saveDatastr) == 1);

  saveVideo = false;
  string saveVideostr = parser.retrieve<string>("v");
  if (saveVideostr.length() != 0) saveVideo = (stoi(saveVideostr) == 1);

  saveTiming = false;
  string saveTimingstr = parser.retrieve<string>("t");
  if (saveTimingstr.length() != 0) saveTiming = (stoi(saveTimingstr) == 1);

  quiet = false;
  string quietstr = parser.retrieve<string>("q");
  if (quietstr.length() != 0) quiet = (stoi(quietstr) == 1);
}
