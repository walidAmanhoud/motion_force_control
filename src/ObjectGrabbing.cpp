#include "ObjectGrabbing.h"
#include "Utils.h"

ObjectGrabbing* ObjectGrabbing::me = NULL;

ObjectGrabbing::ObjectGrabbing(ros::NodeHandle &n, double frequency, std::string filename, ContactDynamics contactDynamics, float targetVelocity, float targetForce):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename),
  _contactDynamics(contactDynamics),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce),
  _xCFilter(3,3,6,1.0f/frequency),
  _xLFilter(3,3,6,1.0f/frequency),
  _zDirFilter(3,3,6,1.0f/frequency)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.035f;
  _toolOffset = 0.13f;
  _loadMass = 0.2f;
  // _objectDim << 0.31f, 0.21f, 0.21f;
  // _objectDim << 0.22f, 0.41f, 0.22f;
  _objectDim << 0.41f, 0.22f, 0.22f;

  _smax = 4.0f;
  for(int k= 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _xd[k].setConstant(0.0f);
    _vdOrig[k].setConstant(0.0f);
    _vdR[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _Fc[k].setConstant(0.0f);
    _Tc[k].setConstant(0.0f);
    _normalForce[k] = 0.0f;
    _Fd[k] = 0.0f;
    _wrenchBias[k].setConstant(0.0f);
    _wrench[k].setConstant(0.0f);
    _filteredWrench[k].setConstant(0.0f);
    _wrenchCount[k] = 0;
    _wrenchBiasOK[k] = false;
    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = false;
    _lambda1[k] = 1.0f;
    _s[k] = _smax;
    _alpha[k] = 0.0f;
    _beta[k] = 0.0f;
    _betap[k] = 0.0f;
    _gamma[k] = 0.0f;
    _gammap[k] = 0.0f;
    _ut[k] = 0.0f;
    _vt[k] = 0.0f;
    _dW[k] = 0.0f;
  }

  _vdC.setConstant(0.0f);
  _vdD.setConstant(0.0f);
  _eD = 0.0f;
  _eoD = 0.0f;
  _eC = 0.0f;
  _eoC = 0.0f;

  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = false;
    _markersPosition.setConstant(0.0f);
    _markersPosition0.setConstant(0.0f);
  }
  _optitrackOK = false;
  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _leftRobotOrigin << 0.066f, 0.9f, 0.0f;
  _xoC << -0.3f, _leftRobotOrigin(1)/2.0f, _objectDim(2)/2.0f;
  _xdC = _xoC;
  Eigen::Vector3f x, y, z;
  x << 1.0f, 0.0f, 0.0f;
  y << 0.0f, 1.0f, 0.0f;
  z << 0.0f, 0.0f, 1.0f;
  _p1 = _xdC-_objectDim(0)/2.0f*x+_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p2 = _xdC+_objectDim(0)/2.0f*x+_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p3 = _xdC+_objectDim(0)/2.0f*x-_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _p4 = _xdC-_objectDim(0)/2.0f*x-_objectDim(1)/2.0f*y+_objectDim(2)/2.0f*z;
  _xoD = (_p3+_p4-_p1-_p2)/2.0f;
  _xdD = _xoD;

  _taskAttractor << -0.4f, 0.5f, 0.6f;

  _objectReachable = false;
  _objectGrabbed = false;
  _firstObjectPose = false;
  _stop = false;
  _ensurePassivity = true;
  _goHome = false;

  _filteredForceGain = 0.9f;
  _grabbingForceThreshold = 4.0f;
  _offset.setConstant(0.0f);

  _averageCount = 0;
  _sequenceID = 0;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "object_shape";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::CUBE;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _xdC(0);
  _msgMarker.pose.position.y = _xdC(1);
  _msgMarker.pose.position.z = _xdC(2);
  _msgMarker.pose.orientation.x = 0.0;
  _msgMarker.pose.orientation.y = 0.0;
  _msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 1.0;
  _msgMarker.scale.x = _objectDim(0);
  _msgMarker.scale.y = _objectDim(1);
  _msgMarker.scale.z = _objectDim(2);
  _msgMarker.color.a = 1.0;

  _msgMarker.color.r = 0.1f;
  _msgMarker.color.g = 0.3f;
  _msgMarker.color.b = 0.9f;
  _msgMarker.color.a = 1.0;
}


bool ObjectGrabbing::init() 
{
  // Subscriber definitions
  _subRobotPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&ObjectGrabbing::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _n.subscribe<geometry_msgs::Twist>("/lwr/joint_controllers/twist", 1, boost::bind(&ObjectGrabbing::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[RIGHT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&ObjectGrabbing::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&ObjectGrabbing::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&ObjectGrabbing::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _n.subscribe<geometry_msgs::Twist>("/lwr2/joint_controllers/twist", 1, boost::bind(&ObjectGrabbing::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[LEFT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&ObjectGrabbing::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&ObjectGrabbing::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackPose[ROBOT_BASIS_RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,ROBOT_BASIS_RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS_LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,ROBOT_BASIS_LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P4] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p4/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P4),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench[RIGHT] = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench[RIGHT] = _n.advertise<geometry_msgs::WrenchStamped>("ObjectGrabbing/filteredWrenchRight", 1);
  _pubNormalForce[RIGHT] = _n.advertise<std_msgs::Float32>("ObjectGrabbing/normalForceRight", 1);

  _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench[LEFT] = _n.advertise<geometry_msgs::Wrench>("/lwr2/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench[LEFT] = _n.advertise<geometry_msgs::WrenchStamped>("ObjectGrabbing/filteredWrenchLeft", 1);
  _pubNormalForce[LEFT] = _n.advertise<std_msgs::Float32>("ObjectGrabbing/normalForceLeft", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ObjectGrabbing/cube", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ObjectGrabbing::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ObjectGrabbing::stopNode);

  _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_grabbing/"+_filename+".txt");
  if(!_outputFile.is_open())
  {
    ROS_ERROR("Cannot open data file");
    return false;
  }

  if(_contactDynamics == NONE)
  {
    ROS_INFO("Contact dynamics: NONE");
  }
  else if(_contactDynamics == LINEAR)
  {
    ROS_INFO("Contact dynamics: LINEAR");
  }
  else
  {
    ROS_ERROR("Contact dynamics not recognized");
  }

  if(_targetVelocity>0.0f)
  {
    ROS_INFO("Target velocity: %f", _targetVelocity);
  }
  else
  {
    ROS_ERROR("Target velocity should be positive");
    return false;
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("Target force should be positive");
    return false;
  }

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_lambda1[RIGHT]))
  {
    ROS_ERROR("Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }

  if(!_n.getParamCached("/lwr2/ds_param/damping_eigval0",_lambda1[LEFT]))
  {
    ROS_ERROR("Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if(!_workspace.init())
  {
    ROS_ERROR("Cannot initialize robots' workspace");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The object grabbing node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void ObjectGrabbing::run()
{
  while (!_stop) 
  {
    if(_firstRobotPose[RIGHT] && _firstRobotPose[LEFT] && _wrenchBiasOK[RIGHT] && _wrenchBiasOK[LEFT] &&
       _firstOptitrackPose[ROBOT_BASIS_RIGHT] && _firstOptitrackPose[ROBOT_BASIS_LEFT] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstOptitrackPose[P4])
    {
      _mutex.lock();

      // Check for update of passive ds controller eigen value
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_lambda1[RIGHT]);
      ros::param::getCached("/lwr2/ds_param/damping_eigval0",_lambda1[LEFT]);

      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {
        computeObjectPose();
        // Compute control command
        if(_firstObjectPose)
        {
          isObjectReachable();

          computeCommand();

        }
        // Publish data to topics
        // publishData();          

        // Log data
        logData();
      }

      _mutex.unlock();

    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k] = _q[k];
    _Fc[k].setConstant(0.0f);
    _Tc[k].setConstant(0.0f);
  }

  // publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  ros::shutdown();
}


void ObjectGrabbing::stopNode(int sig)
{
  me->_stop = true;
}


void ObjectGrabbing::computeObjectPose()
{
  if(_markersTracked.segment(NB_ROBOTS,TOTAL_NB_MARKERS-NB_ROBOTS).sum() == TOTAL_NB_MARKERS-NB_ROBOTS)
  {

    if(!_firstObjectPose)
    {
      _firstObjectPose = true;
    }
    _p1 = _markersPosition.col(P1)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p2 = _markersPosition.col(P2)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p3 = _markersPosition.col(P3)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    _p4 = _markersPosition.col(P4)-_markersPosition0.col(ROBOT_BASIS_RIGHT);

    _xoC = (_p1+_p2+_p3+_p4)/4.0f;
    _xoD = (_p3+_p4-_p1-_p2)/2.0f;
    // _xoD = 0.18f*_xoD.normalized(); 
    _xoD = 0.20f*_xoD.normalized(); 

    SGF::Vec temp(3);

    // Filter center position of object
    _xCFilter.AddData(_xoC);
    _xCFilter.GetOutput(0,temp);
    _xoC = temp;
    Eigen::Vector3f xDir = _p2-_p1;
    xDir.normalize();
    Eigen::Vector3f yDir = _p1-_p4;
    yDir.normalize();
    Eigen::Vector3f zDir = xDir.cross(yDir);
    zDir.normalize();
    _zDirFilter.AddData(xDir.cross(yDir));
    _zDirFilter.GetOutput(0,temp);
    zDir = temp;
    zDir.normalize();   
    _xoC -= 1.0f*(_objectDim(2)/2.0f)*zDir;
      
    // Filter object direction
    _xLFilter.AddData(_xoD);
    _xLFilter.GetOutput(0,temp);
    _xoD = 0.20f*temp.normalized();

    // std::cerr <<"real" << _xdD.norm() << " " <<_xdD.transpose() << std::endl;
    // std::cerr << "filter" <<  _xoD.norm() << " " <<_xoD.transpose() << std::endl;

    // Update marker object position and orientation
    _msgMarker.pose.position.x = _xoC(0);
    _msgMarker.pose.position.y = _xoC(1);
    _msgMarker.pose.position.z = _xoC(2);
    Eigen::Matrix3f R;
    R.col(0) = xDir;
    R.col(1) = yDir;
    R.col(2) = zDir;
    Eigen::Vector4f q = Utils<float>::rotationMatrixToQuaternion(R);
    _msgMarker.pose.orientation.x = q(1);
    _msgMarker.pose.orientation.y = q(2);
    _msgMarker.pose.orientation.z = q(3);
    _msgMarker.pose.orientation.w = q(0);
  }

}

void ObjectGrabbing::isObjectReachable()
{
  bool l = _workspace.isReachable(_xoC-_leftRobotOrigin);
  bool r = _workspace.isReachable(_xoC);
  if(r && l)
  {
    _objectReachable = true;
  }
  else
  {
    _objectReachable = false;
  }

  std::cerr << "Reachable: " << (int) _objectReachable << " " << (int) l << " " << (int) r << " " << _markersTracked.segment(NB_ROBOTS,TOTAL_NB_MARKERS-NB_ROBOTS).sum() << std::endl;
}


void ObjectGrabbing::computeCommand()
{
  computeOriginalDynamics();

  updateTankScalars();

  forceModulation();

  computeDesiredOrientation();
}


void ObjectGrabbing::computeOriginalDynamics()
{
  // Compute robots center + distance vector;
  _xC = (_x[LEFT]+_x[RIGHT])/2.0f;
  _xD = (_x[RIGHT]-_x[LEFT]);


  // If robots grabbed object, adapt center attractor to be the current robot's center
  // => kill the center dynamics 
  _eoD = (_xD-_xoD).dot(_xoD.normalized());
  _eoC = (_xoC-_xC).norm();
  float alpha = Utils<float>::smoothFall(_eoD,0.02f,0.1f)*Utils<float>::smoothFall(_eoC,0.1f,0.2f); 

  // Check if object is grasped
  if(_normalForce[LEFT]*alpha>_grabbingForceThreshold && _normalForce[RIGHT]*alpha>_grabbingForceThreshold)
  {
    if(_objectGrabbed == false)
    {
      ROS_INFO("Object grabbed");
    }
    _objectGrabbed = true;
  }
  else
  {
    _objectGrabbed = false;
  }

  _goHome = false;
  // Compute desired center position and distance direction
  if(_objectReachable)
  {
    if(_objectGrabbed)
    {
      _xdC = _xC;
    }
    else
    {
      _xdC = _xoC;
    }
    if(_contactDynamics == LINEAR)
    {
      _xdD << 0.0f,-1.0f,0.0f;
      _xdD *= 0.20f;
      // _xdD *= 0.41f;
    }
    else
    {
      _xdD = _xoD; 
    }
  }
  else
  {
    if(_objectGrabbed)
    {
      _xdC = _xC;
      if(_contactDynamics == LINEAR)
      {
        _xdD << 0.0f,-1.0f,0.0f;
        _xdD *= 0.20f;
        // _xdD *= 0.41f;
      }
      else
      {
        _xdD = _xoD; 
      }
    }
    else
    {
      _goHome = true;
      _xdC << -0.4f, 0.45f, 0.7f;
      _xdD << 0.0f,-1.0f,0.0f;
      _xdD *= 0.7f; 
    }
  }

  _eD = (_xD-_xdD).dot(_xdD.normalized());
  _eC = (_xdC-_xC).norm();

  std::cerr << "Reachable: " << (int) _objectReachable << " eD: " << _eD << " eC: " <<  _eC << " eoD: " << _eoD << " eoC: " <<  _eoC << std::endl;
  std::cerr << "FL: " << _normalForce[LEFT] << " FR: " <<  _normalForce[RIGHT] << std::endl;
  std::cerr << "Go home: " << (int) _goHome << std::endl;
  // std::cerr << "xD: " <<_xD.transpose() << " xdD" << _xdD.transpose() << " xoD" << _xoD.transpose() << std::endl;

  if(_eD<0.0f)
  {
    _eD = 0.0f;
  }

  _e1[LEFT] = _xdD.normalized();
  _e1[RIGHT] = -_xdD.normalized();


  if(_contactDynamics == NONE)
  {
    _vdC = 4.0f*(_xdC-_xC);
  }
  else if(_contactDynamics == LINEAR)
  {
    if(_objectGrabbed)
    {
      _vdC = (_taskAttractor+_offset-_xC);
      // _vdC = getCyclingMotionVelocity(_xC, _taskAttractor);
      // _xdD << 0.0f,-1.0f,0.0f;
      // _xdD *= 0.24f;
      // _xdD = _xD;
    }
    else
    {
      _vdC = 4.0f*(_xdC-_xC);
    }
  }
  _vdD = 2.0f*(_xdD-_xD);


  // Get robot dynamics + adjust z dynamics to make them at the same height
  _vdOrig[RIGHT] = _vdC+_vdD/2.0f;
  // _vdOrig[RIGHT](2) += -2.0f*(_x[RIGHT](2)-_x[LEFT](2));
  _vdOrig[LEFT] = _vdC-_vdD/2.0f;
  // _vdOrig[LEFT](2) += -2.0f*(_x[LEFT](2)-_x[RIGHT](2));

  for(int k = 0; k < NB_ROBOTS; k++)
  { 
    _vdR[k] = _vdOrig[k]-_vdC;
    if(_vdR[k].dot(_e1[k])<0.0f && _objectGrabbed)
    {
      _vdR[k].setConstant(0.0f);
    }
  }
}


void ObjectGrabbing::updateTankScalars()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_s[k]>_smax)
    {
      _alpha[k] = 0.0f;
    }
    else
    {
      _alpha[k] = 1.0f;
    }
    _alpha[k] = Utils<float>::smoothFall(_s[k],_smax-0.1f*_smax,_smax);

    float dz = 0.01f;
    float ds = 0.1f*_smax;

    _ut[k] = _v[k].dot(_vdR[k]);

    if(_s[k] < 0.0f && _ut[k] < 0.0f)
    {
      _beta[k] = 0.0f;
    }
    else if(_s[k] > _smax && _ut[k] > FLT_EPSILON)
    {
      _beta[k] = 0.0f;
    }
    else
    {
      _beta[k] = 1.0f;
    }
    
    _vt[k] = _v[k].dot(_e1[k]);
    
    if(_s[k] < FLT_EPSILON && _vt[k] > FLT_EPSILON)
    {
      _gamma[k] = 0.0f;
    }
    else if(_s[k] > _smax && _vt[k] < FLT_EPSILON)
    {
      _gamma[k] = 0.0f;
    }
    else
    {
      _gamma[k] = 1.0f;
    }

     // _gamma = 1.0f-smoothRise(_s,_smax-ds,_smax)*Utils<float>::smoothFall(_vt,0.0f,dz)-Utils<float>::smoothFall(_s,0.0f,ds)*smoothRise(_vt,-dz,0.0f);

    if(_vt[k]<FLT_EPSILON)
    {
      _gammap[k] = 1.0f;
    }
    else
    {
      _gammap[k] = _gamma[k];
    }

    // std::cerr << k << ": alpha: " << _alpha[k] << " beta: " << _beta[k] << " gamma: " << _gamma[k] << " gammap: " << _gammap[k] << std::endl;
  }
}


Eigen::Vector3f ObjectGrabbing::getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
{
  Eigen::Vector3f velocity;

  position = position-attractor;

  velocity(2) = -position(2);

  float R = sqrt(position(0) * position(0) + position(1) * position(1));
  float T = atan2(position(1), position(0));

  float r = 0.05f;
  float omega = M_PI;

  velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
  velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

  return velocity;
}


void ObjectGrabbing::forceModulation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _normalForce[k] = fabs((_wRb[k]*_filteredWrench[k].segment(0,3)).dot(_e1[k]));

    // std::cerr << Utils<float>::smoothFall(_eD,0.02f,0.1f) << std::endl;
    float alpha = Utils<float>::smoothFall(_eoD,0.02f,0.1f)*Utils<float>::smoothFall(_eoC,0.1f,0.2f);

    if(_lambda1[k]<1.0f)
    {
      _lambda1[k] = 1.0f;
    }

    if (_goHome)
    {
      _Fd[k] = 0.0f;
    }
    else
    {
      if(_objectGrabbed)
      {
        _Fd[k] = _targetForce;
        // _Fd[k] = _targetForce*Utils<float>::smoothFall(_eD,0.02f,0.1f);
      }
      else
      {
        // _Fd[k] = _targetForce*(1.0f-std::tanh(10.0f*_eD))*(1.0f-std::tanh(10.0f*(_xdC-_xC).norm()));    
        _Fd[k] = _targetForce*alpha;    
      }   
    }

    if(_ensurePassivity)
    {
      _Fd[k]*=_gammap[k];
    }

    float delta = std::pow(2.0f*_e1[k].dot(_vdR[k])*(_Fd[k]/_lambda1[k]),2.0f)+4.0f*std::pow(_vdR[k].norm(),4.0f); 

    float la;

    if(_goHome)
    {
      la = 1.0f;
    }
    else
    {
      if(fabs(_vdR[k].norm())<FLT_EPSILON)
      {
        la = 0.0f;
      }
      else
      {
        la = (-2.0f*_e1[k].dot(_vdR[k])*(_Fd[k]/_lambda1[k])+sqrt(delta))/(2.0f*std::pow(_vdR[k].norm(),2.0f));
      }
      
      if(_ensurePassivity && _s[k] < 0.0f && _ut[k] < 0.0f)
      {
        la = 1.0f;
      }      
    }

    // Update tank dynamics
    float ds;

    if(_firstDampingMatrix[k])
    {
      ds = _dt*(_alpha[k]*_v[k].transpose()*_D[k]*_v[k]-_beta[k]*_lambda1[k]*(la-1.0f)*_ut[k]-_gamma[k]*_Fd[k]*_vt[k]);

      if(_s[k]+ds>=_smax)
      {
        _s[k] = _smax;
      }
      else if(_s[k]+ds<=0.0f)
      {
        _s[k] = 0.0f;
      }
      else
      {
        _s[k]+=ds;
      }
    }

    _dW[k] = _lambda1[k]*(la-1.0f)*(1-_beta[k])*_ut[k]+_Fd[k]*(_gammap[k]-_gamma[k])*_vt[k]-(1-_alpha[k])*_v[k].transpose()*_D[k]*_v[k];


    if(_ensurePassivity)
    {
      _vd[k] = la*_vdR[k]+_gammap[k]*_Fd[k]*_e1[k]/_lambda1[k];
    }
    else
    {
      _vd[k] = la*_vdR[k]+_Fd[k]*_e1[k]/_lambda1[k];
    }

    _vd[k]+=_vdC;

    // std::cerr <<"Measured force: " << (-_wRb*_filteredWrench.segment(0,3)).dot(_e1) << " Fd:  " << _Fd*_lambda1 << " vdR: " << _vdR.norm() << std::endl;
    std::cerr << k << ": Fd: " << _Fd[k] << " delta: " << delta << " la: " << la << " vdr.dot(e1) " << _e1[k].dot(_vdR[k]) << std::endl;
    // std::cerr << k << ": " << _vdR[k].transpose() << " " << _vd[k].transpose() << std::endl;
    // std::cerr << k << ": " << la*_vdR[k].transpose() << " " << temp*lb*_e1[k].transpose() << std::endl;

      std::cerr << k << " Tank: " << _s[k]  <<" dW: " << _dW[k] <<std::endl;
      // std::cerr << _alpha[k]*_v[k].transpose()*_D[k]*_v[k]<< " " << -_beta[k]*_lambda1[k]*(la-1.0f)*_ut[k] << " " << -_gamma[k]*_Fd[k]*_vt[k] << std::endl;
      // std::cerr << "at: " << _alpha[k]*_v[k].transpose()*_D[k]*_v[k] << std::endl;
      // std::cerr << k << ": ut: " << _ut[k] <<  " " << -_beta[k]*_lambda1[k]*(la-1.0f)*_ut[k] << std::endl;
      // std::cerr << k << ": vt: " << _vt[k] << " " << -_gamma[k]*_Fd[k]*_vt[k] << std::endl;

    // Bound desired velocity  
    if(_vd[k].norm()>_velocityLimit)
    {
      _vd[k] *= _velocityLimit/_vd[k].norm();
    }   
  }
}



void ObjectGrabbing::computeDesiredOrientation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Vector3f ref;
    if(k == (int) RIGHT)
    {
      ref = -_xdD.normalized();
    }
    else
    {
      ref = _xdD.normalized();
    }
      
    ref.normalize();

    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f u;
    u = (_wRb[k].col(2)).cross(ref);
    float c = (_wRb[k].col(2)).transpose()*ref;  
    float s = u.norm();
    u /= s;
    
    Eigen::Matrix3f K;
    K << Utils<float>::getSkewSymmetricMatrix(u);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
    Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[k]);

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
    _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(3.0f*_eD));
    // _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,Utils<float>::smoothFall(_eD,0.3f,0.6f));
    // std::cerr << k << " " << angle << " " <<Utils<float>::smoothFall(_eD,0.3f,0.6f) << std::endl;
    // Utils<float>::smkoothFall(_eoD,0.02f,0.1f)

    if(_qd[k].dot(_qdPrev[k])<0.0f)
    {
      _qd[k] *=-1.0f;
    }

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 

    _qdPrev[k] = _qd[k];
  }
}


void ObjectGrabbing::logData()
{
  _outputFile << ros::Time::now() << " "
              << _x[LEFT].transpose() << " "
              << _v[LEFT].transpose() << " "
              << _vdR[LEFT].transpose() << " "
              << _vd[LEFT].transpose() << " "
              << _normalForce[LEFT] << " "
              << _Fd[LEFT] << " "
              << _x[RIGHT].transpose() << " "
              << _v[RIGHT].transpose() << " "
              << _vdR[RIGHT].transpose() << " "
              << _vd[RIGHT].transpose() << " "
              << _normalForce[RIGHT] << " "
              << _Fd[RIGHT] << " "
              << _e1[LEFT].transpose() << " "
              << _vdC.transpose() << " "
              << _xoC.transpose() << " "
              << _xoD.transpose() << " "
              << _xdC.transpose() << " "
              << _xdD.transpose() << " "
              << (int) _objectGrabbed << " "
              << (int) _ensurePassivity << " "
              << _s[LEFT] << " " 
              << _alpha[LEFT] << " "
              << _beta[LEFT] << " "
              << _betap[LEFT] << " "
              << _gamma[LEFT] << " "
              << _gammap[LEFT] << " "
              << _dW[LEFT] << " "
              << _s[RIGHT] << " " 
              << _alpha[RIGHT] << " "
              << _beta[RIGHT] << " "
              << _betap[RIGHT] << " "
              << _gamma[RIGHT] << " "
              << _gammap[RIGHT] << " "
              << _dW[RIGHT] << std::endl;

}


void ObjectGrabbing::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Publish desired twist (passive ds controller)
    _msgDesiredTwist.linear.x  = _vd[k](0);
    _msgDesiredTwist.linear.y  = _vd[k](1);
    _msgDesiredTwist.linear.z  = _vd[k](2);

    // Convert desired end effector frame angular velocity to world frame
    _msgDesiredTwist.angular.x = _omegad[k](0);
    _msgDesiredTwist.angular.y = _omegad[k](1);
    _msgDesiredTwist.angular.z = _omegad[k](2);

    _pubDesiredTwist[k].publish(_msgDesiredTwist);

    // Publish desired orientation
    _msgDesiredOrientation.w = _qd[k](0);
    _msgDesiredOrientation.x = _qd[k](1);
    _msgDesiredOrientation.y = _qd[k](2);
    _msgDesiredOrientation.z = _qd[k](3);

    _pubDesiredOrientation[k].publish(_msgDesiredOrientation);

    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _filteredWrench[k](0);
    _msgFilteredWrench.wrench.force.y = _filteredWrench[k](1);
    _msgFilteredWrench.wrench.force.z = _filteredWrench[k](2);
    _msgFilteredWrench.wrench.torque.x = _filteredWrench[k](3);
    _msgFilteredWrench.wrench.torque.y = _filteredWrench[k](4);
    _msgFilteredWrench.wrench.torque.z = _filteredWrench[k](5);
    _pubFilteredWrench[k].publish(_msgFilteredWrench);

    _msgDesiredWrench.force.x = _Fc[k](0);
    _msgDesiredWrench.force.y = _Fc[k](1);
    _msgDesiredWrench.force.z = _Fc[k](2);
    _msgDesiredWrench.torque.x = _Tc[k](0);
    _msgDesiredWrench.torque.y = _Tc[k](1);
    _msgDesiredWrench.torque.z = _Tc[k](2);
    _pubDesiredWrench[k].publish(_msgDesiredWrench);

    std_msgs::Float32 msg;
    msg.data = _normalForce[k];
    _pubNormalForce[k].publish(msg);
    
  }

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _pubMarker.publish(_msgMarker);
}


void ObjectGrabbing::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffset*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _x[k] += _leftRobotOrigin;
  }

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    _qd[k] = _q[k];
    _qdPrev[k] = _q[k];
    _vd[k].setConstant(0.0f);
    _qinit[k] = _q[k];
  }
}


void ObjectGrabbing::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void ObjectGrabbing::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_loadMass*_gravity;
    _wrenchBias[k].segment(0,3) -= loadForce;
    _wrenchBias[k].segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias[k] += raw; 
    _wrenchCount[k]++;
    if(_wrenchCount[k]==NB_SAMPLES)
    {
      _wrenchBias[k] /= NB_SAMPLES;
      _wrenchBiasOK[k] = true;
      std::cerr << "Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    _wrench[k] = raw-_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_loadMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }
}


void ObjectGrabbing::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void ObjectGrabbing::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(k == (int) ROBOT_BASIS_RIGHT || k == (int) ROBOT_BASIS_LEFT)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
}


uint16_t ObjectGrabbing::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void ObjectGrabbing::optitrackInitialization()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked(ROBOT_BASIS_RIGHT) && _markersTracked(ROBOT_BASIS_LEFT))
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }
    std::cerr << "Optitrack Initialization count: " << _averageCount << std::endl;
    if(_averageCount == 1)
    {
      ROS_INFO("Optitrack Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      ROS_INFO("Optitrack Initialization done !");
      _leftRobotOrigin = _markersPosition0.col(ROBOT_BASIS_LEFT)-_markersPosition0.col(ROBOT_BASIS_RIGHT);
    }
  }
  else
  {
    _optitrackOK = true;
  }
}


void ObjectGrabbing::dynamicReconfigureCallback(motion_force_control::objectGrabbing_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _grabbingForceThreshold = config.grabbingForceThreshold;
  _offset(0) = config.xOffset;
  _offset(1) = config.yOffset;
  _offset(2) = config.zOffset;
}