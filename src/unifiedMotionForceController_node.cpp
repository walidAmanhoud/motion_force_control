#include "UnifiedMotionForceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UnifiedMotionForceController");
  ros::NodeHandle n;
  float frequency = 500.0f;

  UnifiedMotionForceController unifiedMotionForceController(n,frequency);
 
  if (!unifiedMotionForceController.init()) 
  {
    return -1;
  }
  else
  {
   
    unifiedMotionForceController.run();
  }

  return 0;
}

