#include "ObjectGrabbing.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objectGrabbing");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  ObjectGrabbing::SurfaceType surfaceType;  
  ObjectGrabbing::OriginalDynamics originalDynamics;
  ObjectGrabbing::ModulationType modulationType;
  ObjectGrabbing::Formulation formulation;
  ObjectGrabbing::Constraint constraint;
  bool useOptitrack;
  float targetVelocity;
  float targetForce;

  std::ostringstream ss;
  std::string temp;

  // rosrun motion_force_control modualtedDS fileName -u y/n -o c/a -m r/fr -f f1/f2/f3 -c v/a 
  if(argc == 8) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = ObjectGrabbing::SurfaceType::PLANE;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "po")
    {
      surfaceType = ObjectGrabbing::SurfaceType::PLANE_OPTITRACK;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "ls")
    {
      surfaceType = ObjectGrabbing::SurfaceType::LEARNED_SURFACE;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "c")
    {
      originalDynamics = ObjectGrabbing::OriginalDynamics::CONSTANT;
    }
    else if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "a")
    {
      originalDynamics = ObjectGrabbing::OriginalDynamics::ARBITRARY;
    }
    else
    {
      ROS_ERROR("Wrong original dynamics arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "r")
    {
      modulationType = ObjectGrabbing::ModulationType::ROTATION;
    }
    else if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "rf")
    {
      modulationType = ObjectGrabbing::ModulationType::ROTATION_AND_FORCE;
    }
    else
    {
      ROS_ERROR("Wrong modulation type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }
    
    formulation = ObjectGrabbing::Formulation::F2;
    constraint = ObjectGrabbing::Constraint::VELOCITY_NORM;
    targetVelocity = 0.2f;
    targetForce = 10.0f;

    ss << "_" << targetVelocity << "_" << targetForce;
    fileName += "_"+std::string(argv[3])+"_"+std::string(argv[5])+"_"+std::string(argv[7])+"_f2_v"+ss.str();
  }
  else if(argc == 16) 
  {
    fileName = std::string(argv[1]);

    if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "p")
    {
      surfaceType = ObjectGrabbing::SurfaceType::PLANE;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "po")
    {
      surfaceType = ObjectGrabbing::SurfaceType::PLANE_OPTITRACK;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "ls")
    {
      surfaceType = ObjectGrabbing::SurfaceType::LEARNED_SURFACE;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "c")
    {
      originalDynamics = ObjectGrabbing::OriginalDynamics::CONSTANT;
    }
    else if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "a")
    {
      originalDynamics = ObjectGrabbing::OriginalDynamics::ARBITRARY;
    }
    else
    {
      ROS_ERROR("Wrong original dynamics arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "r")
    {
      modulationType = ObjectGrabbing::ModulationType::ROTATION;
    }
    else if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "rf")
    {
      modulationType = ObjectGrabbing::ModulationType::ROTATION_AND_FORCE;
    }
    else
    {
      ROS_ERROR("Wrong modulation type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f1")
    {
      formulation = ObjectGrabbing::Formulation::F1;
    }
    else if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f2")
    {
      formulation = ObjectGrabbing::Formulation::F2;
    }
    else if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f3")
    {
      formulation = ObjectGrabbing::Formulation::F3;
    }
    else
    {
      ROS_ERROR("Wrong formulation arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm)");
      return 0;
    }

    if(std::string(argv[10]) == "-c" && std::string(argv[11]) == "v")
    {
      constraint = ObjectGrabbing::Constraint::VELOCITY_NORM;
    }
    else if(std::string(argv[10]) == "-c" && std::string(argv[11]) == "a")
    {
      constraint = ObjectGrabbing::Constraint::APPARENT_VELOCITY_NORM;
    }
    else
    {
      ROS_ERROR("Wrong constraint arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[12]) == "-v" && atof(argv[13])> 0.0f)
    {
      targetVelocity = atof(argv[13]);
    }
    else
    {
      ROS_ERROR("Wrong target velocity arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }  

    if(std::string(argv[14]) == "-f" && atof(argv[15])> 0.0f)
    {
      targetForce = atof(argv[15]);
    }
    else
    {
      ROS_ERROR("Wrong target force arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    } 

    ss << "_" << targetVelocity << "_" << targetForce;
    fileName += "_"+std::string(argv[3])+"_"+std::string(argv[5])+"_"+std::string(argv[7])+"_"+std::string(argv[9])+"_"+std::string(argv[11])+ss.str();
  }
  else
  {
    ROS_ERROR("You are missing arguments: the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
    return 0;
  }


  ObjectGrabbing objectGrabbing(n,frequency,fileName,surfaceType,originalDynamics,modulationType,formulation,constraint,targetVelocity,targetForce);

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

