#include "UnifiedMotionForceController.h"

UnifiedMotionForceController* UnifiedMotionForceController::me = NULL;

UnifiedMotionForceController::UnifiedMotionForceController(ros::NodeHandle &n, double frequency):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _controller(1.0f/frequency)
{
    me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.046f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _taskAttractor << -0.5f, -0.1f, 0.186f;
  
  _planeNormal << 0.0f, 0.0f, 1.0f;
  _p << 0.0f,0.0f,0.186f;

  _Fc.setConstant(0.0f);
  _Fds.setConstant(0.0f);
  _Fu.setConstant(0.0f);
  _minFc = 0.0f;
  _maxFc = 0.0f;

  _firstRealPoseReceived = false;
  _firstWrenchReceived = false;
  _wrenchBiasOK = false;

  _useOptitrack = true;

  if(_useOptitrack)
  {
    _firstPlane1Pose = false;
    _firstPlane2Pose = false;
    _firstPlane3Pose = false;
    _firstRobotBasisPose = false;    
  }
  else
  {
    _firstPlane1Pose = true;
    _firstPlane2Pose = true;
    _firstPlane3Pose = true;
    _firstRobotBasisPose = true;   
  }
  _stop = false;


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


bool UnifiedMotionForceController::init() 
{
  // Subscriber definitions
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &UnifiedMotionForceController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &UnifiedMotionForceController::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &UnifiedMotionForceController::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackRobotBasisPose = _n.subscribe("/optitrack/robot/pose", 1, &UnifiedMotionForceController::updateRobotBasisPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane1Pose = _n.subscribe("/optitrack/plane1/pose", 1, &UnifiedMotionForceController::updatePlane1Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane2Pose = _n.subscribe("/optitrack/plane2/pose", 1, &UnifiedMotionForceController::updatePlane2Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane3Pose = _n.subscribe("/optitrack/plane3/pose", 1, &UnifiedMotionForceController::updatePlane3Pose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1); 
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("UnifiedMotionForceController/taskAttractor", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("UnifiedMotionForceController/plane", 1);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("UnifiedMotionForceController/forceNorm", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("UnifiedMotionForceController/filteredWrench", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&UnifiedMotionForceController::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,UnifiedMotionForceController::stopNode);

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The foot mouse controller is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void UnifiedMotionForceController::run()
{
  while (!_stop) 
  {
    // if(_firstRealPoseReceived && _wrenchBiasOK)
    if(_firstRealPoseReceived && _wrenchBiasOK &&//)// &&
       _firstRobotBasisPose && _firstPlane1Pose &&
       _firstPlane2Pose && _firstPlane3Pose)
    {

      _mutex.lock();
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _Fc.setConstant(0.0f);
  _Fds.setConstant(0.0f);
  _Fu.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void UnifiedMotionForceController::stopNode(int sig)
{
  me->_stop = true;
}

void  UnifiedMotionForceController::computeCommand()
{
  autonomousControl();
}

void  UnifiedMotionForceController::autonomousControl()
{
  
  // Extract linear speed, force and torque data
  Eigen::Vector3f v = _twist.segment(0,3);
  Eigen::Vector3f F = _filteredWrench.segment(0,3);  
  Eigen::Vector3f T = _filteredWrench.segment(3,3);

  // Compute plane normal form markers position
  if(_useOptitrack)
  {
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


  // Compute canonical basis used to decompose the desired dynamics along the plane frame
  // D = [e1 e2 e3] with e1 = -n while e2 and e3 are orthogonal to e1
  Eigen::Vector3f temp;
  temp << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B, S;
  Eigen::Vector3f e1, e2, e3;
  e1 = -_planeNormal;
  S = e1*e1.transpose();
  e2 = (Eigen::Matrix3f::Identity()-S)*temp;
  e2 = e2/(e2.norm());
  e3 = e1.cross(e2);
  B.col(0) = e1;
  B.col(1) = e2;
  B.col(2) = e3;

  // Compute normal distance to the plane
  float normalDistance = (S*(_x-_xp)).norm();

  // Compute Weighted diagonal matrix
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  // L(0,0) = _convergenceRate*(1-std::tanh(10*normalDistance));
  L(0,0) = _convergenceRate*std::tanh(50*normalDistance);
  // L(0,0) = _convergenceRate;
  L(1,1) = _convergenceRate*(1-std::tanh(50*normalDistance));
  L(2,2) = _convergenceRate*(1-std::tanh(50*normalDistance));

  // Compute vertical projection of the current position onto the plane
  _xp = _x;
  if(_useOptitrack)
  {
    _xp(2) = (-_planeNormal(0)*(_xp(0)-_p3(0))-_planeNormal(1)*(_xp(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
    
  }
  else
  {
    _xp(2) = (-_planeNormal(0)*(_xp(0)-_p(0))-_planeNormal(1)*(_xp(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  }
  // Compute fixed attractor on plane


  Eigen::Vector3f xs;
  if(_useOptitrack)
  {
    xs = _p1+0.05*(_p2-_p1)+0.5*(_p3-_p1);
    _xa = _p1+0.7*(_p2-_p1)+0.5*(_p3-_p1);
    _xa(2) = (-_planeNormal(0)*(_xa(0)-_p1(0))-_planeNormal(1)*(_xa(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  }
  else
  {
    _xa = _taskAttractor;
    _xa(2) = (-_planeNormal(0)*(_xa(0)-_p(0))-_planeNormal(1)*(_xa(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  }
  // std::cerr << "xa: " << _xa.transpose() << std::endl;

  // Check if polishing motion is activated and compute desired velocity based on motion dynamics = sum of linear dynamics coming from moving and fixed attractors
  if(_polishing && !_linear)
  {
    // _vd = (B*L*B.transpose())*((_xp-_x)+(Eigen::Matrix3f::Identity()-S)*getDesiredVelocity(_x,_xa));
    _vd = std::tanh(50*normalDistance)*(_xp-_x)+_convergenceRate*(1-std::tanh(50*normalDistance))*(Eigen::Matrix3f::Identity()-S)*getDesiredVelocity(_x,_xa);

  }
  else if(_linear && !_polishing)
  {
    _vd = (B*L*B.transpose())*((_xp-_x)+(Eigen::Matrix3f::Identity()-S)*(_xa-_x));
    // _vd = std::tanh(50*normalDistance)*(xs-_x)+_convergenceRate*(1-std::tanh(50*normalDistance))*(Eigen::Matrix3f::Identity()-S)*(_xa-_x);
  }
  else
  {
    _vd = (B*L*B.transpose())*(_xp-_x);
    // _vd = std::tanh(50*normalDistance)*(xs-_x);
  }

  // Compute projected force and speed along plane normal
  Eigen::Vector3f Fn = S*(-_wRb*F);
  Eigen::Vector3f vn = S*v;

  // Compute desired force along surface normal
  Eigen::Vector3f Fd = _targetForce*e1;

  // Check if force control activated and compute contact force following the specified dynamics
  if(_controlForce)
  {
    // Compute force error
    Eigen::Vector3f Fe = Fd-Fn;
    
    // Compute force stiffness rate gain from measured speed
    float alpha = 1-std::tanh(50*vn.norm());
    // float alpha = (1-std::tanh(50*normalDistance));

    // Integrate contact force magnitude dynamics
    _Fc += _dt*(_k1*alpha*Fe-_k2*(1-alpha)*_Fc);

    if(_Fc.dot(e1) < _minFc*alpha)
    {
      _Fc = _minFc*alpha*e1;
    }
    else if(_Fc.dot(e1) > _maxFc)
    {
      _Fc = _maxFc*e1;
    }


    // _Fc = alpha*(Fd+_k2*Fe-_k1*vn);

    // std::cerr << "contact force: " << _Fc.transpose() << " fc: " << _Fc.dot(e1) << " vn: " << vn.norm() << std::endl;
    // std::cerr << "vd: " << _vd.transpose() << " dt: " << _dt << " alpha: " << alpha << std::endl;
  }
  else
  {
    
    _Fc.setConstant(0.0f);
  }


  // Bound desired velocity  
  if(_vd.norm()>0.3f)
  {
    _vd *= 0.3f/_vd.norm();
  }

  //////////////////////////////////////
  // Test passive ds force controller //
  //////////////////////////////////////

  _controller.updateDampingGains(_lambda1,_lambda2);
  // _Fds = _controller.step(_vd,v);
  // _Fu = _Fc+_Fds;
  // float gamma;
  // if(normalDistance>0.0f)
  // {
  //   gamma = 1.0f-std::tanh(100*normalDistance);
  // }
  // else
  // {
  //   gamma = 1.0f;
  // }
  // _Fc = Fd;
  // _Fu = _controller.step(_vd,v)+_Fc;

  // _s = 0.0f;
  _Fu = _controller.step(_vd,v,_Fc);
  _s = _controller.getEnergyTank();
  // std::cerr << _Fds.transpose() << " " << _lambda1 << " " << _lambda2 <<std::endl;
  // std::cerr << "z: "<< _Fc.dot(v) << " Energy tank: " << _controller.getEnergyTank() << " Alpha: " << _controller.getAlpha() << " Beta: " << _controller.getBeta() << std::endl;
  


  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  Eigen::Matrix3f K;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Convert rotation error into axis angle representation
  float angle;
  Eigen::Vector3f omega;
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  Eigen::Vector4f qf = quaternionProduct(qtemp,_q);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  _qd = slerpQuaternion(_q,qf,1-std::tanh(5*normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q(0);
  qcurI.segment(1,3) = -_q.segment(1,3);
  wq = 5.0f*quaternionProduct(qcurI,_qd-_q);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegad = omegaTemp;
}


void UnifiedMotionForceController::publishData()
{
  // _mutex.lock();

  // Publish desired pose
  _msgDesiredPose.position.x = _xd(0);
  _msgDesiredPose.position.y = _xd(1);
  _msgDesiredPose.position.z = _xd(2);
  _msgDesiredPose.orientation.w = _qd(0);
  _msgDesiredPose.orientation.x = _qd(1);
  _msgDesiredPose.orientation.y = _qd(2);
  _msgDesiredPose.orientation.z = _qd(3);

  _pubDesiredPose.publish(_msgDesiredPose);

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
  Eigen::Vector3f center = _p1+0.5f*(_p2-_p1)+0.5f*(_p3-_p1); 
  _msgMarker.pose.position.x = center(0);
  _msgMarker.pose.position.y = center(1);
  _msgMarker.pose.position.z = center(2);
  Eigen::Vector3f u,v,n;
  u = _p3-_p1;
  v = _p2-_p1;
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

  std_msgs::Float32 msg;
  msg.data = _forceNorm;
  msg.data = _s;
  _pubForceNorm.publish(msg);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);


  _msgDesiredWrench.force.x = _Fu(0);
  _msgDesiredWrench.force.y = _Fu(1);
  _msgDesiredWrench.force.z = _Fu(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);
}


void UnifiedMotionForceController::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if(!_firstRealPoseReceived)
  {
    _firstRealPoseReceived = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}

 
void UnifiedMotionForceController::updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRealPoseReceived)
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

  if(_wrenchBiasOK && _firstRealPoseReceived)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}


void UnifiedMotionForceController::updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;
}


void UnifiedMotionForceController::updateRobotBasisPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _robotBasisPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  _robotBasisPosition(2) -= 0.025f;
  if(!_firstRobotBasisPose)
  {
    _firstRobotBasisPose = true;
  }
}


void UnifiedMotionForceController::updatePlane1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane1Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane1Pose)
  {
    _firstPlane1Pose = true;
  }
}


void UnifiedMotionForceController::updatePlane2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane2Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane2Pose)
  {
    _firstPlane2Pose = true;
  }
}


void UnifiedMotionForceController::updatePlane3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane3Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane3Pose)
  {
    _firstPlane3Pose = true;
  }
}


void UnifiedMotionForceController::dynamicReconfigureCallback(motion_force_control::unifiedMotionForceController_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _contactForceThreshold = config.contactForceThreshold;
  _targetForce = config.targetForce;
  _polishing = config.polishing;
  _linear = config.linear;
  _controlForce = config.controlForce;
  _k1 = config.k1;
  _k2 = config.k2;
  _minFc = config.minFc;
  _maxFc = config.maxFc;
  _lambda1 = config.lambda1;
  _lambda2 = config.lambda2;
  _controller.updateDampingGains(_lambda1,_lambda2);
}


Eigen::Vector4f UnifiedMotionForceController::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f UnifiedMotionForceController::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f UnifiedMotionForceController::rotationMatrixToQuaternion(Eigen::Matrix3f R)
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


Eigen::Matrix3f UnifiedMotionForceController::quaternionToRotationMatrix(Eigen::Vector4f q)
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


void UnifiedMotionForceController::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
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


Eigen::Vector4f UnifiedMotionForceController::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
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


Eigen::Vector3f UnifiedMotionForceController::getDesiredVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
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