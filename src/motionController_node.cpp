#include "MotionController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motionController");
  ros::NodeHandle n;
  float frequency = 1000.0f;

  MotionController motionController(n,frequency);
 
  if (!motionController.init()) 
  {
    return -1;
  }
  else
  {
   
    motionController.run();
  }

  return 0;
}

