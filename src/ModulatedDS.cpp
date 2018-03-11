#include "ModulatedDS.h"

ModulatedDS* ModulatedDS::me = NULL;

ModulatedDS::ModulatedDS(ros::NodeHandle &n, double frequency, std::string fileName, bool useOptitrack, 
                         OriginalDynamics originalDynamics, ModulationType modulationType,
                         Formulation formulation, Constraint constraint, float targetVelocity, float targetForce):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _useOptitrack(useOptitrack),
  _originalDynamics(originalDynamics),
  _modulationType(modulationType),
  _formulation(formulation),
  _constraint(constraint),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.035f;
  _toolOffset = 0.14f;
  _loadMass = 0.1f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _vdOrig.setConstant(0.0f);
  _vdR.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _Fd = 0.0f;
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _taskAttractor << -0.6f, 0.0f, 0.19f;
  _p << 0.0f,0.0f,0.19f;
  
  _planeNormal << 0.0f, 0.0f, 1.0f;

  _Fc.setConstant(0.0f);

  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;

  _wrenchBiasOK = false;
  _stop = false;

  _sequenceID = 0;

  if(_useOptitrack)
  {
    _firstOptitrackRobotPose = false;
    _firstOptitrackP1Pose = false;
    _firstOptitrackP2Pose = false;
    _firstOptitrackP3Pose = false;
  }
  else
  {
    _firstOptitrackRobotPose = true;
    _firstOptitrackP1Pose = true;
    _firstOptitrackP2Pose = true;
    _firstOptitrackP3Pose = true;  
  }

  _lambda1 = 0.0f;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "marker_test_triangle_list";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _p(0);
  _msgMarker.pose.position.y = _p(1);
  _msgMarker.pose.position.z = _p(2);
  _msgMarker.pose.orientation.x = 0.0;
  _msgMarker.pose.orientation.y = 1.0;
  _msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 0.0;
  _msgMarker.scale.x = 1.0;
  _msgMarker.scale.y = 1.0;
  _msgMarker.scale.z = 1.0;
  _msgMarker.color.a = 1.0;

  geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  float objectWidth = 0.59f;
  float objectLength = 0.82f;
  p1.x = objectWidth/2.0f;
  p1.y = -objectLength/2.0f;
  p1.z = 0.0f;
  p2.x = -objectWidth/2.0f;
  p2.y = -objectLength/2.0f;
  p2.z = 0.0f;
  p3.x = -objectWidth/2.0f;
  p3.y = objectLength/2.0f;
  p3.z = 0.0f;
  p4.x = -objectWidth/2.0f;
  p4.y = objectLength/2.0f;
  p4.z = 0.0f;
  p5.x = objectWidth/2.0f;
  p5.y = objectLength/2.0f;
  p5.z = 0.0f;
  p6.x = objectWidth/2.0f;
  p6.y = -objectLength/2.0f;
  p6.z = 0.0f;

  Eigen::Vector3f t1,t2;
  t1 << 1.0f,0.0f,0.0f;
  t2 << 0.0f,1.0f,0.0f;
  _p1 = _p-0.3f*t1+(objectLength/2.0f)*t2;
  _p2 = _p1-objectLength*t2;
  _p3 = _p1-objectWidth*t1;

  std_msgs::ColorRGBA c;
  c.r = 0.7;
  c.g = 0.7;
  c.b = 0.7;
  c.a = 1.0;


  for(int k = 0; k < 6; k++)
  {
    _msgMarker.colors.push_back(c);
  }

  _msgMarker.points.push_back(p1);
  _msgMarker.points.push_back(p2);
  _msgMarker.points.push_back(p3);
  _msgMarker.points.push_back(p4);
  _msgMarker.points.push_back(p5);
  _msgMarker.points.push_back(p6);



}


bool ModulatedDS::init() 
{
  // Subscriber definitions
  _subRobotPose = _n.subscribe("/lwr/ee_pose", 1, &ModulatedDS::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _n.subscribe("/lwr/ee_vel", 1, &ModulatedDS::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ModulatedDS::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackRobotBasisPose = _n.subscribe("/optitrack/robot/pose", 1, &ModulatedDS::updateOptitrackRobotPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane1Pose = _n.subscribe("/optitrack/plane1/pose", 1, &ModulatedDS::updateOptitrackP1Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane2Pose = _n.subscribe("/optitrack/plane2/pose", 1, &ModulatedDS::updateOptitrackP2Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane3Pose = _n.subscribe("/optitrack/plane3/pose", 1, &ModulatedDS::updateOptitrackP3Pose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ModulatedDS/filteredWrench", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ModulatedDS/plane", 1);
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ModulatedDS/taskAttractor", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ModulatedDS::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ModulatedDS::stopNode);


  ROS_INFO("Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data/"+_fileName+"_data.txt");


  if(!_n.getParamCached("/ds_param/damping_eigval0",_lambda1))
  {
    ROS_ERROR("Cannot read first eigen value of passive ds controller");
    return false;
  }


  if(!_outputFile.is_open())
  {
    ROS_ERROR("Cannot open data file");
    return false;
  }

  ROS_INFO("Use optitrack: %d", (int) _useOptitrack);

  if(_originalDynamics == CONSTANT)
  {
    ROS_INFO("Original dynamics: CONSTANT");
  }
  else if(_originalDynamics == ARBITRARY)
  {
    ROS_INFO("Original dynamics: ARBITRARY");
  }
  else
  {
    ROS_ERROR("Original dynamics not recognized");
    return false;
  }

  if(_modulationType == ROTATION)
  {
    ROS_INFO("Modulation type: ROTATION");
  }
  else if(_modulationType == ROTATION_AND_FORCE)
  {
    ROS_INFO("Modulation type: ROTATION_AND_FORCE");
  }
  else
  {
    ROS_ERROR("Modulation type not recognized");
    return false;
  }

  if(_formulation == F1)
  {
    ROS_INFO("Formulation: F1");
  }
  else if(_formulation == F2)
  {
    ROS_INFO("Formulation: F2");
  }
  else if(_formulation == F3)
  {
    ROS_INFO("Formulation: F3");
  }
  else
  {
    ROS_ERROR("Formulation not recognized");
    return false;
  }

  if(_constraint == VELOCITY_NORM)
  {
    ROS_INFO("Constraint: VELOCITY_NORM");
  }
  else if(_constraint == APPARENT_VELOCITY_NORM)
  {
    ROS_INFO("Constraint: APPARENT_VELOCITY_NORM");
  }
  else
  {
    ROS_ERROR("Constraint not recognized");
    return false;
  }

  if(_targetVelocity>0.0f)
  {
    ROS_INFO("Target velocity: %f", _targetVelocity);
  }
  else
  {
    ROS_ERROR("Target velocity should be positive");
  }

  if(_targetForce>0.0f)
  {
    ROS_INFO("Target force: %f", _targetForce);
  }
  else
  {
    ROS_ERROR("Target force should be positive");
  }


  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void ModulatedDS::run()
{
  while (!_stop) 
  {

    // std::cerr << (int) _firstRobotPose << (int) _firstRobotTwist  << (int) _firstOptitrackRobotPose << (int) _firstOptitrackP1Pose << (int) _firstOptitrackP2Pose << (int) _firstOptitrackP3Pose << std::endl;
    if(_firstRobotPose && _firstRobotTwist && /*_wrenchBiasOK &&*/
       _firstOptitrackRobotPose && _firstOptitrackP1Pose &&
       _firstOptitrackP2Pose && _firstOptitrackP3Pose)
    {
      _mutex.lock();

      // Check for update of passive ds controller eigen value
      ros::param::getCached("/ds_param/damping_eigval0",_lambda1);

      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _Fc.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void ModulatedDS::stopNode(int sig)
{
  me->_stop = true;
}

void ModulatedDS::computeCommand()
{

  computeProjectionOnSurface();

  computeOriginalDynamics();


  switch(_modulationType)
  {
    case ROTATION:
    {

      rotatingDynamics();
      _vd = _vdR;

      break;
    }
    case ROTATION_AND_FORCE:
    {
      rotatingDynamics();

      forceModulation();
      break;
    }
    default:
    {
      ROS_ERROR("Modulation type does not exist");
      me->_stop = true;

      break;
    }
  }

  computeDesiredOrientation();
}

void ModulatedDS::computeProjectionOnSurface()
{
  if(_useOptitrack)
  {
    // Compute plane normal form markers position
    _p1 = _plane1Position-_robotBasisPosition;
    _p2 = _plane2Position-_robotBasisPosition;
    _p3 = _plane3Position-_robotBasisPosition;
    Eigen::Vector3f p13,p12;
    p13 = _p3-_p1;
    p12 = _p2-_p1;
    p13 /= p13.norm();
    p12 /= p12.norm();
    _planeNormal = p13.cross(p12);
    _planeNormal /= _planeNormal.norm();
  }

  // Compute vertical projection on surface
  _xProj = _x;

  if(_useOptitrack)
  {
    _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p3(0))-_planeNormal(1)*(_xProj(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
  }
  else
  {
    _xProj(2) = (-_planeNormal(0)*(_xProj(0)-_p(0))-_planeNormal(1)*(_xProj(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  }

  // Compute _e1 = normal vector pointing towards the surface
  _e1 = -_planeNormal;
  
  // Compute signed normal distance to the plane
  _normalDistance = (_xProj-_x).dot(_e1);

  if(_normalDistance < 0.0f)
  {
    _normalDistance = 0.0f;
  }
}


void ModulatedDS::computeOriginalDynamics()
{
  // Compute fixed attractor on plane
  if(_useOptitrack)
  {
    _xAttractor = _p1+0.7f*(_p2-_p1)+0.5f*(_p3-_p1);
    _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p1(0))-_planeNormal(1)*(_xAttractor(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  }
  else
  {
    _xAttractor = _taskAttractor;
    _xAttractor(2) = (-_planeNormal(0)*(_xAttractor(0)-_p(0))-_planeNormal(1)*(_xAttractor(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  }
  
  if(_originalDynamics == CONSTANT)
  {
    Eigen::Vector3f dir;
    dir = -_planeNormal;
    dir.normalize();
    _vdOrig = _targetVelocity*dir;
  }
  else if(_originalDynamics == ARBITRARY)
  {
    float alpha = std::tanh(50.0f*_normalDistance);
    _vdOrig = alpha*(_xProj-_x)+_convergenceRate*(1.0f-alpha)*(Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCyclingMotionVelocity(_x,_xAttractor);
  }

  if(_vdOrig.norm()>_velocityLimit)
  {
    _vdOrig *= _velocityLimit/_vdOrig.norm();
  }
}


Eigen::Vector3f ModulatedDS::getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
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


void ModulatedDS::rotatingDynamics()
{
  if(_originalDynamics == CONSTANT)
  {
    Eigen::Vector3f vdContact;
    vdContact = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCyclingMotionVelocity(_x,_xAttractor);
    vdContact.normalize();

    // Compute rotation angle
    float angle = std::acos(_vdOrig.normalized().dot(vdContact));
    float theta = (1.0f-std::tanh(10*_normalDistance))*angle;

    // Compute rotation direction
    Eigen::Vector3f u = (_vdOrig.normalized()).cross(vdContact);
    Eigen::Matrix3f K,R;

    if(u.norm() < FLT_EPSILON)
    {
      R.setIdentity();
    }
    else
    {
      u/=u.norm();
      K = getSkewSymmetricMatrix(u);
      R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-std::cos(theta))*K*K;
    }
    _vdR = R*_vdOrig;
  }
  else
  {
    _vdR = _vdOrig;
  }
}


void ModulatedDS::forceModulation()
{
  // Extract linear speed, force and torque data

  // Compute modulation matrix used to apply a force Fd when the surface is reached while keeping the norm of the velocity constant 
  // M(x) = B(x)L(x)B(x)
  // B(x) = [e1 e2 e3] with e1 = -n is an orthognal basis defining the modulation frame
  //        [la lb lb] 
  // L(x) = [0  lc 0 ] is the matrix defining the modulation gains to apply on the frame
  //        [0  0  lc]

  // Compute modulation frame B(x)
  Eigen::Vector3f xDir;
  xDir << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  _e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*xDir;
  _e2.normalize();
  _e3 = _e1.cross(_e2);
  _e3.normalize();
  B.col(0) = _e1;
  B.col(1) = _e2;
  B.col(2) = _e3;

  // Compute force profile
  if(_lambda1<1.0f)
  {
    _lambda1 = 1.0f;
  }

  _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;

  // Compute diagonal gain matrix L(x)
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  float la, lb, lc;

  float temp, delta;

  switch(_formulation)
  {
    case F1:
    {
      temp = (_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = _Fd/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdOrig.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdOrig.norm(),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdOrig.norm(),4.0f); 
      }

      if(delta < 0.0f)
      {
        delta = 0.0f;
      }

      la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));

      L(0,0) = la;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = la;
      L(2,2) = la;

      break;
    }
    
    case F2:
    {
      temp = (_e1+_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = _Fd/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdOrig.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdOrig.norm(),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdOrig.norm(),4.0f); 
      }

      if(delta < 0.0f)
      {
        delta = 0.0f;
      }
      la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));

      L(0,0) = la+lb;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = la;
      L(2,2) = la;

      break;
    }

    case F3:
    {
      temp = (_e1+_e2+_e3).dot(_vdR);
      if(fabs(temp)<FLT_EPSILON)
      {
        lb = 0.0f;
      }
      else
      {
        lb = (_Fd+_e1.dot(_vdR))/temp;
      }

      if(_constraint == VELOCITY_NORM)
      {
        delta = (_vdR.squaredNorm()-std::pow(lb*temp,2.0f))/(std::pow(_e2.dot(_vdR),2.0f)+std::pow(_e3.dot(_vdR),2.0f));
      }
      else if(_constraint == APPARENT_VELOCITY_NORM)
      {
        delta = (_vdR.squaredNorm()-2*lb*temp*_e1.dot(_vdR)+std::pow(_e1.dot(_vdR),2.0f))/(std::pow(_e2.dot(_vdR),2.0f)+std::pow(_e3.dot(_vdR),2.0f));
      }

      if(delta<0)
      {
        delta = 0.0f;
      }
      la = sqrt(delta)-1.0f; 

      L(0,0) = lb;
      L(0,1) = lb;
      L(0,2) = lb;
      L(1,1) = 1.0f+la;
      L(2,2) = 1.0f+la; 

      break;
    }

    default:
    {
      break; 
    }
  }

  // Compute modulation matrix
  Eigen::Matrix3f M;
  M = B*L*B.transpose();

  // Apply force modulation to the rotating dynamics

  _vd = M*_vdR;

  std::cerr << "delta: " << delta << " la: " << la << " lb: " << lb << " vd: " << _vd.norm() << std::endl;


  // Bound desired velocity  
  if(_vd.norm()>_velocityLimit)
  {
    _vd *= _velocityLimit/_vd.norm();
  }

  _Fc.setConstant(0.0f);

  std::cerr << "vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << std::endl;
}


void ModulatedDS::computeDesiredOrientation()
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  
  Eigen::Matrix3f K;
  K << getSkewSymmetricMatrix(k);

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
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  Eigen::Vector4f qf = quaternionProduct(qtemp,_q);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  _qd = slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp; 
}

void ModulatedDS::logData()
{
    _outputFile << ros::Time::now() << " "
                << _x.transpose() << " "
                << _v.transpose() << " "
                << _vdOrig.transpose() << " "
                << _vdR.transpose() << " "
                << _vd.transpose() << " "
                << _e1.transpose() << " "
                << _normalDistance << " "
                << _Fd << " "
                << _sequenceID << std::endl;
}


void ModulatedDS::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubTaskAttractor.publish(_msgTaskAttractor);

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  Eigen::Vector3f center, u,v,n;
  if(_useOptitrack)
  {
    center = _p1+0.5f*(_p2-_p1)+0.5f*(_p3-_p1); 
    u = _p3-_p1;
    v = _p2-_p1;  
  }
  else
  {
    center = _p;
    u << 1.0f, 0.0f, 0.0f;
    v << 0.0f, 1.0f, 0.0f;
  }
  _msgMarker.pose.position.x = center(0);
  _msgMarker.pose.position.y = center(1);
  _msgMarker.pose.position.z = center(2);
  u /= u.norm();
  v /= v.norm();
  n = u.cross(v);
  Eigen::Matrix3f R;
  R.col(0) = u;
  R.col(1) = v;
  R.col(2) = n;
  Eigen::Vector4f q = rotationMatrixToQuaternion(R);


  _msgMarker.pose.orientation.x = q(1);
  _msgMarker.pose.orientation.y = q(2);
  _msgMarker.pose.orientation.z = q(3);
  _msgMarker.pose.orientation.w = q(0);

  _pubMarker.publish(_msgMarker);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);


  _msgDesiredWrench.force.x = _Fc(0);
  _msgDesiredWrench.force.y = _Fc(1);
  _msgDesiredWrench.force.z = _Fc(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);
}


void ModulatedDS::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if((_x-temp).norm()>FLT_EPSILON)
  {
    _sequenceID++;
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void ModulatedDS::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _v(0) = msg->linear.x;
  _v(1) = msg->linear.y;
  _v(2) = msg->linear.z;
  _w(0) = msg->angular.x;
  _w(1) = msg->angular.y;
  _w(2) = msg->angular.z;

  if(!_firstRobotTwist)
  {
    _firstRobotTwist = true;
  }
}
 
void ModulatedDS::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}


void ModulatedDS::updateOptitrackRobotPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _robotBasisPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  _robotBasisPosition(2) -= 0.03f;
  if(!_firstOptitrackRobotPose)
  {
    _firstOptitrackRobotPose = true;
  }
}


void ModulatedDS::updateOptitrackP1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane1Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstOptitrackP1Pose)
  {
    _firstOptitrackP1Pose = true;
  }
}


void ModulatedDS::updateOptitrackP2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane2Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstOptitrackP2Pose)
  {
    _firstOptitrackP2Pose = true;
  }
}


void ModulatedDS::updateOptitrackP3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane3Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstOptitrackP3Pose)
  {
    _firstOptitrackP3Pose = true;
  }
}


Eigen::Vector4f ModulatedDS::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f ModulatedDS::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f ModulatedDS::rotationMatrixToQuaternion(Eigen::Matrix3f R)
{
  Eigen::Vector4f q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);


  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


Eigen::Matrix3f ModulatedDS::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void ModulatedDS::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));
}


Eigen::Vector4f ModulatedDS::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
{

  Eigen::Vector4f q;

  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
  if(q1.dot(q2)<0.0f)
  {   
    q2 = -q2;
  }

  float dotProduct = q1.dot(q2);
  if(dotProduct > 1.0f)
  {
    dotProduct = 1.0f;
  }
  else if(dotProduct < -1.0f)
  {
    dotProduct = -1.0f;
  }

  float omega = acos(dotProduct);

  if(std::fabs(omega)<FLT_EPSILON)
  {
    q = q1.transpose()+t*(q2-q1).transpose();
  }
  else
  {
    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
  }

  return q;
}


void ModulatedDS::dynamicReconfigureCallback(motion_force_control::modulatedDS_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
}