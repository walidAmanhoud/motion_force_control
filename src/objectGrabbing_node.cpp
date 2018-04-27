#include "ObjectGrabbing.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objectGrabbing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  float targetVelocity;
  float targetForce;

  // rosrun motion_force_control modualtedDS fileName -u y/n -o c/a -m r/fr -f f1/f2/f3 -c v/a 
  if(argc == 5) 
  {

    if(std::string(argv[1]) == "-v" && atof(argv[2])> 0.0f)
    {
      targetVelocity = atof(argv[2]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity arguments, the command line arguments should be: -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[3]) == "-f" && atof(argv[4])> 0.0f)
    {
      targetForce = atof(argv[4]);
    }
    else
    {
      ROS_ERROR("Wrong target force arguments, the command line arguments should be: -v(target velocity) value -f(target force) value");
      return 0;
    } 

  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: -v(target velocity) value -f(target force) value");
    return 0;
  }


  ObjectGrabbing objectGrabbing(n,frequency,targetVelocity,targetForce);

  if (!objectGrabbing.init()) 
  {
    return -1;
  }
  else
  {
   
    objectGrabbing.run();
  }

  return 0;
}

