#include "SurfaceLearning.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfaceLearning");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;

  SurfaceLearning::Mode mode;

  if(argc == 4) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "l")
    {
      mode = SurfaceLearning::Mode::LOGGING;
    }
    else if(std::string(argv[2]) == "-m" && std::string(argv[3]) == "t")
    {
      mode = SurfaceLearning::Mode::TESTING;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Wrong number of arguments, the command line arguments should be: fileName -m(mode) l(logging) or t(testing)");
    return 0;
  }

  SurfaceLearning surfaceLearning(n,frequency,fileName,mode);

  if (!surfaceLearning.init()) 
  {
    return -1;
  }
  else
  {
   
    surfaceLearning.run();
  }

  return 0;
}

