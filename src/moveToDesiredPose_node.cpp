#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredPose.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "moveToDesiredPose");

  Eigen::Vector3f desiredPosition;

  // Initialize desired position
  desiredPosition.setConstant(0.0f);



  // Check if desired position is specified with the command line
  if(argc == 4)
  {
    for(int k = 0; k < 3; k++)
    {
      desiredPosition(k) = atof(argv[k+1]);
    }
  }

  std::cerr << desiredPosition << std::endl;
  ros::NodeHandle n;
  float frequency = 200.0f;

  MoveToDesiredPose moveToDesiredPose(n,frequency);

  if (!moveToDesiredPose.init()) 
  {
    return -1;
  }
  else
  {
    moveToDesiredPose.setDesiredPose(desiredPosition);
    moveToDesiredPose.run();
  }

  return 0;

}
