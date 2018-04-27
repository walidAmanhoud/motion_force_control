#include "ObjectGrabbing.h"

ObjectGrabbing* ObjectGrabbing::me = NULL;

ObjectGrabbing::ObjectGrabbing(ros::NodeHandle &n, double frequency, float targetVelocity, float targetForce):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _targetVelocity(targetVelocity),
  _targetForce(targetForce)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.035f;
  _toolOffset = 0.14f;
  _loadMass = 0.0f;

  for(int k= 0; k <2; k++)
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
    _normalDistance[k] = 0.0f;
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
  }

  for(int k = 0; k < TOTAL_NB_MARKERS; k++)
  {
    _firstOptitrackPose[k] = true;
  }

  _taskAttractor << -0.6f, -0.2f, 0.186f;
  _contactAttractor << -0.6f, 0.2f, 0.186f;
  _p << 0.0f,0.0f,0.19f;
  
  _stop = false;

  _optitrackOK = false;

  _averageCount = 0;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _lambda1 = 0.0f;

  // _firstDampingMatrix = false;
  // _D.setConstant(0.0f);
  // _smax = 0.2f;
  // _s = 0.0f;

  // _msgMarker.header.frame_id = "world";
  // _msgMarker.header.stamp = ros::Time();
  // _msgMarker.ns = "marker_test_triangle_list";
  // _msgMarker.id = 0;
  // _msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  // _msgMarker.action = visualization_msgs::Marker::ADD;
  // _msgMarker.pose.position.x = _p(0);
  // _msgMarker.pose.position.y = _p(1);
  // _msgMarker.pose.position.z = _p(2);
  // _msgMarker.pose.orientation.x = 0.0;
  // _msgMarker.pose.orientation.y = 1.0;
  // _msgMarker.pose.orientation.z = 0.0;
  // _msgMarker.pose.orientation.w = 0.0;
  // _msgMarker.scale.x = 1.0;
  // _msgMarker.scale.y = 1.0;
  // _msgMarker.scale.z = 1.0;
  // _msgMarker.color.a = 1.0;

  // geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  // float objectWidth = 0.59f;
  // float objectLength = 0.82f;
  // p1.x = objectWidth/2.0f;
  // p1.y = -objectLength/2.0f;
  // p1.z = 0.0f;
  // p2.x = -objectWidth/2.0f;
  // p2.y = -objectLength/2.0f;
  // p2.z = 0.0f;
  // p3.x = -objectWidth/2.0f;
  // p3.y = objectLength/2.0f;
  // p3.z = 0.0f;
  // p4.x = -objectWidth/2.0f;
  // p4.y = objectLength/2.0f;
  // p4.z = 0.0f;
  // p5.x = objectWidth/2.0f;
  // p5.y = objectLength/2.0f;
  // p5.z = 0.0f;
  // p6.x = objectWidth/2.0f;
  // p6.y = -objectLength/2.0f;
  // p6.z = 0.0f;

  // Eigen::Vector3f t1,t2;
  // t1 << 1.0f,0.0f,0.0f;
  // t2 << 0.0f,1.0f,0.0f;
  // _p1 = _p-0.3f*t1+(objectLength/2.0f)*t2;
  // _p2 = _p1-objectLength*t2;
  // _p3 = _p1-objectWidth*t1;

  // std_msgs::ColorRGBA c;
  // c.r = 0.7;
  // c.g = 0.7;
  // c.b = 0.7;
  // c.a = 1.0;

  // _msgArrowMarker.header.frame_id = "world";
  // _msgArrowMarker.header.stamp = ros::Time();
  // _msgArrowMarker.ns = "marker_test_arrow";
  // _msgArrowMarker.id = 1;
  // _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  // _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  // p1.x = 0.0f;
  // p1.y = 0.0f;
  // p1.z = 0.0f;
  // p2.x = 0.0f+0.3f*_e1(0);
  // p2.x = 0.0f+0.3f*_e1(1);
  // p2.x = 0.0f+0.3f*_e1(2);
  // _msgArrowMarker.scale.x = 0.05;
  // _msgArrowMarker.scale.y = 0.1;
  // _msgArrowMarker.scale.z = 0.1;
  // _msgArrowMarker.color.a = 1.0;
  // _msgArrowMarker.color.r = 1.0;
  // _msgArrowMarker.color.g = 0.0;
  // _msgArrowMarker.color.b = 0.0;
  // _msgArrowMarker.points.push_back(p1);
  // _msgArrowMarker.points.push_back(p2);


  // for(int k = 0; k < 6; k++)
  // {
  //   _msgMarker.colors.push_back(c);
  // }

  // _msgMarker.points.push_back(p1);
  // _msgMarker.points.push_back(p2);
  // _msgMarker.points.push_back(p3);
  // _msgMarker.points.push_back(p4);
  // _msgMarker.points.push_back(p5);
  // _msgMarker.points.push_back(p6);

}


bool ObjectGrabbing::init() 
{
  // Subscriber definitions
  _subRobotPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&ObjectGrabbing::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _n.subscribe<geometry_msgs::Twist>("/lwr/joint_controllers/twist", 1, boost::bind(&ObjectGrabbing::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&ObjectGrabbing::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&ObjectGrabbing::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _n.subscribe<geometry_msgs::Twist>("/lwr2/joint_controllers/twist", 1, boost::bind(&ObjectGrabbing::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&ObjectGrabbing::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackPose[ROBOT_BASIS_RIGHT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_right/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,ROBOT_BASIS_RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[ROBOT_BASIS_LEFT] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/robot_left/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,ROBOT_BASIS_LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P1] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p1/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P2] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p2/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P2),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P3] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p3/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P3),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[P4] = _n.subscribe<geometry_msgs::PoseStamped>("/optitrack/p4/pose", 1, boost::bind(&ObjectGrabbing::updateOptitrackPose,this,_1,P4),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix = _n.subscribe("/lwr/joint_controllers/passive_ds_damping_matrix", 1, &ObjectGrabbing::updateDampingMatrix,this,ros::TransportHints().reliable().tcpNoDelay());

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
  // _pubMarker = _n.advertise<visualization_msgs::Marker>("ObjectGrabbing/plane", 1);
  // _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ObjectGrabbing/taskAttractor", 1);  

  // Dynamic reconfigure definition
  // _dynRecCallback = boost::bind(&ObjectGrabbing::dynamicReconfigureCallback, this, _1, _2);
  // _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ObjectGrabbing::stopNode);

  if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_lambda1))
  {
    ROS_ERROR("Cannot read first eigen value of passive ds controller");
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

  _timeInit = ros::Time::now().toSec();

  while (!_stop) 
  {


    if(_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _wrenchBiasOK[RIGHT] &&
       _firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && _wrenchBiasOK[LEFT] &&
       _firstOptitrackPose[ROBOT_BASIS_RIGHT] && _firstOptitrackPose[ROBOT_BASIS_LEFT] && _firstOptitrackPose[P1] &&
       _firstOptitrackPose[P2] && _firstOptitrackPose[P3] && _firstOptitrackPose[P4])
    {
      _mutex.lock();

      // Check for update of passive ds controller eigen value
      ros::param::getCached("/lwr/ds_param/damping_eigval0",_lambda1);

      if(!_optitrackOK)
      {
        optitrackInitialization();
      }
      else
      {
        // Compute control command
        computeCommand();

        // Publish data to topics
        publishData();

        // Log data
        logData();
      }

      _mutex.unlock();

    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd[RIGHT].setConstant(0.0f);
  _omegad[RIGHT].setConstant(0.0f);
  _qd[RIGHT] = _q[RIGHT];
  _Fc[RIGHT].setConstant(0.0f);
  _Tc[RIGHT].setConstant(0.0f);

  _vd[LEFT].setConstant(0.0f);
  _omegad[LEFT].setConstant(0.0f);
  _qd[LEFT] = _q[LEFT];
  _Fc[LEFT].setConstant(0.0f);
  _Tc[LEFT].setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void ObjectGrabbing::stopNode(int sig)
{
  me->_stop = true;
}

void ObjectGrabbing::computeCommand()
{

  computeProjectionOnSurface();

  computeOriginalDynamics();

  rotatingDynamics();

  updateTankScalars();

  forceModulation();

  computeDesiredOrientation();
}

void ObjectGrabbing::computeProjectionOnSurface()
{

}


void ObjectGrabbing::computeOriginalDynamics()
{

}


void ObjectGrabbing::rotatingDynamics()
{

}


void ObjectGrabbing::updateTankScalars()
{

}


void ObjectGrabbing::forceModulation()
{
  // Extract linear speed, force and torque data

  // Compute modulation matrix used to apply a force Fd when the surface is reached while keeping the norm of the velocity constant 
  // M(x) = B(x)L(x)B(x)
  // B(x) = [e1 e2 e3] with e1 = -n is an orthognal basis defining the modulation frame
  //        [la lb lb] 
  // L(x) = [0  lc 0 ] is the matrix defining the modulation gains to apply on the frame
  //        [0  0  lc]

  // Compute modulation frame B(x)
  // Eigen::Vector3f xDir;
  // xDir << 1.0f,0.0f,0.0f;
  // Eigen::Matrix3f B;
  // _e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*xDir;
  // _e2.normalize();
  // _e3 = _e1.cross(_e2);
  // _e3.normalize();
  // B.col(0) = _e1;
  // B.col(1) = _e2;
  // B.col(2) = _e3;

  // // Compute force profile
  // if(_lambda1<1.0f)
  // {
  //   _lambda1 = 1.0f;
  // }

  // if((-_wRb*_filteredWrench.segment(0,3)).dot(_e1)<2.0f)
  // {
  //   _Fd = 5.0f/_lambda1;
  // }
  // else
  // {
  //   _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;    
  //   // _Fd = (_targetForce*(1.0f-std::tanh(100.0f*_normalDistance))+_integratorGain*(_targetForce*(1.0f-std::tanh(100.0f*_normalDistance))+(_wRb*_filteredWrench.segment(0,3)).dot(_e1)))/_lambda1;    

  // }
  // _Fd = _targetForce*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;






  // Compute diagonal gain matrix L(x)
  // Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  // float la, lb, lc;

  // float temp, delta;


  //   temp = (_e1+_e2+_e3).dot(_vdR);
  //   if(fabs(temp)<FLT_EPSILON)
  //   {
  //     lb = 0.0f;
  //   }
  //   else
  //   {
  //     lb = _Fd/temp;
  //     // lb = _gammap*_Fd/temp;
  //   }

  //   if(_constraint == VELOCITY_NORM)
  //   {
  //     delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdR.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdR.norm(),2.0f));
  //   }
  //   else if(_constraint == APPARENT_VELOCITY_NORM)
  //   {
  //     delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdR.norm(),4.0f); 
  //   }

  //   if(delta < 0.0f)
  //   {
  //     delta = 0.0f;
  //     la = 0.0f;
  //   }
  //   else
  //   {
  //     la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdR.norm(),2.0f));
  //   }

    
    // if(_s < 0.0f && _ut < 0.0f)
    // {
    //   la = 1.0f;
    // }
    // else
    // {
    //   la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));
    // }


    // Update tank dynamics
    // float ds;

    // if(_firstDampingMatrix)
    // {
    //   ds = _dt*(_alpha*_v.transpose()*_D*_v-_beta*_lambda1*(la-1.0f)*_ut-_gamma*_Fd*_vt);
    //   // ds = _dt*(-_beta*_lambda1*(la-1.0f)*_ut-_gamma*_Fd*_vt);
    //   // ds = _dt*(_alpha*_v.transpose()*_D*_v-_beta*_lambda1*la*_ut-_gamma*_Fd*_vt);
    //   if(_s+ds>=_smax)
    //   {
    //     _s = _smax;
    //   }
    //   else if(_s+ds<=0.0f)
    //   {
    //     _s = 0.0f;
    //   }
    //   else
    //   {
    //     _s+=ds;
    //   }
    // }

    // float dW;
    // dW = _lambda1*(la-1.0f)*(1-_beta)*_ut+_Fd*(_gammap-_gamma)*_vt-(1-_alpha)*_v.transpose()*_D*_v;

    // std::cerr << "Tank: " << _s << " " <<_alpha*_v.transpose()*_D*_v<< " " << -_beta*_lambda1*(la-1.0f)*_ut << " " << -_gamma*_Fd*_vt << std::endl;
    // std::cerr << "at: " << _alpha*_v.transpose()*_D*_v << std::endl;
    // std::cerr << "ut: " << _ut <<  " " << -_beta*_lambda1*(la-1.0f)*_ut << std::endl;
    // std::cerr << "vt: " << _vt << " " << -_gamma*_Fd*_vt << std::endl;
    // std::cerr << "Tank: " << _s  <<" dW: " << dW <<std::endl;

    // L(0,0) = la+lb;
    // L(0,1) = lb;
    // L(0,2) = lb;
    // L(1,1) = la;
    // L(2,2) = la;

}


void ObjectGrabbing::computeDesiredOrientation()
{
  // // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  // Eigen::Vector3f k;
  // k = (-_wRb.col(2)).cross(_planeNormal);
  // float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  // float s = k.norm();
  // k /= s;
  
  // Eigen::Matrix3f K;
  // K << getSkewSymmetricMatrix(k);

  // Eigen::Matrix3f Re;
  // if(fabs(s)< FLT_EPSILON)
  // {
  //   Re = Eigen::Matrix3f::Identity();
  // }
  // else
  // {
  //   Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  // }
  
  // // Convert rotation error into axis angle representation
  // Eigen::Vector3f omega;
  // float angle;
  // Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  // quaternionToAxisAngle(qtemp,omega,angle);

  // // Compute final quaternion on plane
  // Eigen::Vector4f qf = quaternionProduct(qtemp,_q);

  // // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  // _qd = slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));

  // // Compute needed angular velocity to perform the desired quaternion
  // Eigen::Vector4f qcurI, wq;
  // qcurI(0) = _q(0);
  // qcurI.segment(1,3) = -_q.segment(1,3);
  // wq = 5.0f*quaternionProduct(qcurI,_qd-_q);
  // Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  // _omegad = omegaTemp; 

}

void ObjectGrabbing::logData()
{

}


void ObjectGrabbing::publishData()
{
  for(int k = 0; k < 2; k++)
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
  // _msgTaskAttractor.header.frame_id = "world";
  // _msgTaskAttractor.header.stamp = ros::Time::now();
  // _msgTaskAttractor.point.x = _taskAttractor(0);
  // _msgTaskAttractor.point.y = _taskAttractor(1);
  // _msgTaskAttractor.point.z = _taskAttractor(2);
  // _pubTaskAttractor.publish(_msgTaskAttractor);

  // _msgMarker.header.frame_id = "world";
  // _msgMarker.header.stamp = ros::Time();
  // Eigen::Vector3f center, u,v,n;
  // if(_surfaceType == PLANE_OPTITRACK)
  // {
  //   center = _p1+0.5f*(_p2-_p1)+0.2f*(_p3-_p1); 
  //   u = _p3-_p1;
  //   v = _p2-_p1;  
  // }
  // else
  // {
  //   center = _p;
  //   u << 1.0f, 0.0f, 0.0f;
  //   v << 0.0f, 1.0f, 0.0f;
  // }
  // _msgMarker.pose.position.x = center(0);
  // _msgMarker.pose.position.y = center(1);
  // _msgMarker.pose.position.z = center(2);
  // u /= u.norm();
  // v /= v.norm();
  // n = u.cross(v);
  // Eigen::Matrix3f R;
  // R.col(0) = u;
  // R.col(1) = v;
  // R.col(2) = n;
  // Eigen::Vector4f q = rotationMatrixToQuaternion(R);


  // _msgMarker.pose.orientation.x = q(1);
  // _msgMarker.pose.orientation.y = q(2);
  // _msgMarker.pose.orientation.z = q(3);
  // _msgMarker.pose.orientation.w = q(0);


  // _pubMarker.publish(_msgMarker);

  // _msgArrowMarker.points.clear();
  // geometry_msgs::Point p1, p2;
  // p1.x = _x(0);
  // p1.y = _x(1);
  // p1.z = _x(2);
  // p2.x = _x(0)+0.3f*_e1(0);
  // p2.y = _x(1)+0.3f*_e1(1);
  // p2.z = _x(2)+0.3f*_e1(2);
  // _msgArrowMarker.points.push_back(p1);
  // _msgArrowMarker.points.push_back(p2);
  // _pubMarker.publish(_msgArrowMarker);
}


void ObjectGrabbing::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffset*_wRb[k].col(2);

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    _qd[k] = _q[k];
    _vd[k].setConstant(0.0f);
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

void ObjectGrabbing::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg) 
{
  if(!_firstDampingMatrix)
  {
    _firstDampingMatrix = true;
  }

  _D << msg->data[0],msg->data[1],msg->data[2],
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
    }
  }
  else
  {
    _optitrackOK = true;
  }
}


Eigen::Vector4f ObjectGrabbing::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f ObjectGrabbing::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f ObjectGrabbing::rotationMatrixToQuaternion(Eigen::Matrix3f R)
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


Eigen::Matrix3f ObjectGrabbing::quaternionToRotationMatrix(Eigen::Vector4f q)
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


void ObjectGrabbing::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
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


Eigen::Vector4f ObjectGrabbing::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
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


// void ObjectGrabbing::dynamicReconfigureCallback(motion_force_control::modulatedDS_paramsConfig &config, uint32_t level)
// {
//   ROS_INFO("Reconfigure request. Updatig the parameters ...");

//   _convergenceRate = config.convergenceRate;
//   _filteredForceGain = config.filteredForceGain;
//   _velocityLimit = config.velocityLimit;
//   _integratorGain = config.integratorGain;
//   _duration = config.duration;
//   _offset(0) = config.xOffset;
//   _offset(1) = config.yOffset;
//   _offset(2) = config.zOffset;
//   _integrator.setConstant(0.0f);
// }


float ObjectGrabbing::smoothRise(float x, float a, float b)
{
  float y; 
  if(x<a)
  {
    y = 0.0f;
  }
  else if(x>b)
  {
    y = 1.0f;
  }
  else
  {
    y = (1.0f+sin(M_PI*(x-a)/(b-a)-M_PI/2.0f))/2.0f;
  }

  return y;
}

float ObjectGrabbing::smoothFall(float x, float a, float b)
{
  return 1.0f-smoothRise(x,a,b);
}

float ObjectGrabbing::smoothRiseFall(float x, float a, float b, float c, float d)
{
  return smoothRise(x,a,b)*smoothFall(x,c,d);
}
