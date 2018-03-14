#include "SurfaceLearning.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfaceLearning");
  ros::NodeHandle n;
  float frequency = 500.0f;
  
  std::string fileName;

  if(argc == 2) 
  {
    fileName = std::string(argv[1]);
  }
  else
  {
    fileName = "test" ;
  }

  SurfaceLearning surfaceLearning(n,frequency,fileName);

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

