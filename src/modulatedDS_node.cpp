#include "ModulatedDS.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modulatedDS");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  std::string fileName;
  ModulatedDS::SurfaceType surfaceType;  
  ModulatedDS::OriginalDynamics originalDynamics;
  ModulatedDS::ModulationType modulationType;
  ModulatedDS::Formulation formulation;
  ModulatedDS::Constraint constraint;
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
      surfaceType = ModulatedDS::SurfaceType::PLANE;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "po")
    {
      surfaceType = ModulatedDS::SurfaceType::PLANE_OPTITRACK;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "ls")
    {
      surfaceType = ModulatedDS::SurfaceType::LEARNED_SURFACE;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "c")
    {
      originalDynamics = ModulatedDS::OriginalDynamics::CONSTANT;
    }
    else if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "a")
    {
      originalDynamics = ModulatedDS::OriginalDynamics::ARBITRARY;
    }
    else
    {
      ROS_ERROR("Wrong original dynamics arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "r")
    {
      modulationType = ModulatedDS::ModulationType::ROTATION;
    }
    else if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "rf")
    {
      modulationType = ModulatedDS::ModulationType::ROTATION_AND_FORCE;
    }
    else
    {
      ROS_ERROR("Wrong modulation type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }
    
    formulation = ModulatedDS::Formulation::F2;
    constraint = ModulatedDS::Constraint::VELOCITY_NORM;
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
      surfaceType = ModulatedDS::SurfaceType::PLANE;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "po")
    {
      surfaceType = ModulatedDS::SurfaceType::PLANE_OPTITRACK;
    }
    else if(std::string(argv[2]) == "-s" && std::string(argv[3]) == "ls")
    {
      surfaceType = ModulatedDS::SurfaceType::LEARNED_SURFACE;
    }
    else
    {
      ROS_ERROR("Wrong surface type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "c")
    {
      originalDynamics = ModulatedDS::OriginalDynamics::CONSTANT;
    }
    else if(std::string(argv[4]) == "-o" && std::string(argv[5]) == "a")
    {
      originalDynamics = ModulatedDS::OriginalDynamics::ARBITRARY;
    }
    else
    {
      ROS_ERROR("Wrong original dynamics arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "r")
    {
      modulationType = ModulatedDS::ModulationType::ROTATION;
    }
    else if(std::string(argv[6]) == "-m" && std::string(argv[7]) == "rf")
    {
      modulationType = ModulatedDS::ModulationType::ROTATION_AND_FORCE;
    }
    else
    {
      ROS_ERROR("Wrong modulation type arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm) -v(target velocity) value -f(target force) value");
      return 0;
    }

    if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f1")
    {
      formulation = ModulatedDS::Formulation::F1;
    }
    else if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f2")
    {
      formulation = ModulatedDS::Formulation::F2;
    }
    else if(std::string(argv[8]) == "-f" && std::string(argv[9]) == "f3")
    {
      formulation = ModulatedDS::Formulation::F3;
    }
    else
    {
      ROS_ERROR("Wrong formulation arguments, the command line arguments should be: fileName -s(surface type) p(plane) or po(plane optitrack) or ls(learned surface) -o(original dynamics) c(constant) or a(arbitrary) -m(modulation type) r(rotation) or rf(rotation and force) -f(formulation) f1 or f2 or f3 -c(constraint) v (velocity norm) or a (apparent velocity norm)");
      return 0;
    }

    if(std::string(argv[10]) == "-c" && std::string(argv[11]) == "v")
    {
      constraint = ModulatedDS::Constraint::VELOCITY_NORM;
    }
    else if(std::string(argv[10]) == "-c" && std::string(argv[11]) == "a")
    {
      constraint = ModulatedDS::Constraint::APPARENT_VELOCITY_NORM;
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


  ModulatedDS modulatedDS(n,frequency,fileName,surfaceType,originalDynamics,modulationType,formulation,constraint,targetVelocity,targetForce);

  if (!modulatedDS.init()) 
  {
    return -1;
  }
  else
  {
   
    modulatedDS.run();
  }

  return 0;
}

