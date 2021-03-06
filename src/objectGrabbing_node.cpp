#include "ObjectGrabbing.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "objectGrabbing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  float targetVelocity;
  float targetForce;

  std::string filename;
  ObjectGrabbing::ContactDynamics contactDynamics;

  // rosrun motion_force_control modualtedDS fileName -u y/n -o c/a -m r/fr -f f1/f2/f3 -c v/a 
  if(argc == 8) 
  {
    filename = std::string(argv[1]);

    if(std::string(argv[2]) == "-c" && std::string(argv[3])=="n")
    {
      contactDynamics = ObjectGrabbing::ContactDynamics::NONE;
    }
    else if(std::string(argv[2]) == "-c" && std::string(argv[3])=="l")
    {
      contactDynamics = ObjectGrabbing::ContactDynamics::LINEAR;
    }
    else
    {
      ROS_ERROR("Wrong contact dynamics arguments, the command line arguments should be: fileName -c(contact dynamics) n(none)/l(linear) -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[4]) == "-v" && atof(argv[5])> 0.0f)
    {
      targetVelocity = atof(argv[5]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity arguments, the command line arguments should be: fileName -c(contact dynamics) n(none)/l(linear) -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[6]) == "-f" && atof(argv[7])> 0.0f)
    {
      targetForce = atof(argv[7]);
    }
    else
    {
      ROS_ERROR("Wrong target force arguments, the command line arguments should be: fileName -c(contact dynamics) n(none)/l(linear) -v(target velocity) value -f(target force) value");
      return 0;
    } 

  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -c(contact dynamics) n(none)/l(linear) -v(target velocity) value -f(target force) value");
    return 0;
  }


  ObjectGrabbing objectGrabbing(n,frequency,filename,contactDynamics,targetVelocity,targetForce);

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

