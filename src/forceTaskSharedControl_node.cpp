#include "ForceTaskSharedControl.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "forceTaskSharedControl");
  ros::NodeHandle n;
  float frequency = 500.0f;

  ForceTaskSharedControl ftsc(n,frequency);

  if (!ftsc.init()) 
  {
    return -1;
  }
  else
  {
   
    ftsc.run();
  }

  return 0;
}
