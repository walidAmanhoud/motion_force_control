#include "ForceTaskSharedControl.h"

ForceTaskSharedControl* ForceTaskSharedControl::me = NULL;

ForceTaskSharedControl::ForceTaskSharedControl(ros::NodeHandle &n, double frequency):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
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
  _vh.setConstant(0.0f);

  _taskAttractor << -0.5f, 0.0f, 0.25f;
  
  _normalDirection << 0.0f, 0.0f, 1.0f;
  _p << 0.0f,0.0f,0.186f;

  _Fc.setConstant(0.0f);
  _minFc = 0.0f;
  _maxFc = 0.0f;
  _e1 = -_normalDirection;

  _linearVhLimit = 0.0f;
  _angularVhLimit = 0.0f;

  _firstRealPoseReceived = false;
  _firstWrenchReceived = false;
  _wrenchBiasOK = false;
  _stop = false;

  _alpha = 0.0f;

	_firstEventReceived = false;
	_msgFootMouse.event = foot_interfaces::FootMouseMsg::FM_NONE;
	_lastEvent = foot_interfaces::FootMouseMsg::FM_NONE;
	_buttonPressed = false;

  _msgArrowMarker.header.frame_id = "world";
  _msgArrowMarker.header.stamp = ros::Time();
  _msgArrowMarker.ns = "marker_test_arrow";
  _msgArrowMarker.id = 0;
  _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p1,p2,p3;
  p1.x = _p(0);
  p1.y = _p(1);
  p1.z = _p(2);
  p2.x = _p(0)+0.3f*_e1(0);
  p2.x = _p(1)+0.3f*_e1(1);
  p2.x = _p(2)+0.3f*_e1(2);
  _msgArrowMarker.scale.x = 0.1;
  _msgArrowMarker.scale.y = 0.3;
  _msgArrowMarker.scale.z = 0.1;
  _msgArrowMarker.color.a = 1.0;
  _msgArrowMarker.color.r = 1.0;
  _msgArrowMarker.color.g = 0.0;
  _msgArrowMarker.color.b = 0.0;
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
}


bool ForceTaskSharedControl::init() 
{
  // Subscriber definitions
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &ForceTaskSharedControl::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/joint_controllers/twist", 1, &ForceTaskSharedControl::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ForceTaskSharedControl::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subFootMouse= _n.subscribe("/foot_mouse", 1, &ForceTaskSharedControl::updateFootMouseData, this, ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1); 
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ForceTaskSharedControl/markers", 10);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("ForceTaskSharedControl/forceNorm", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ForceTaskSharedControl/filteredWrench", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ForceTaskSharedControl::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ForceTaskSharedControl::stopNode);

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The force modulated task is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void ForceTaskSharedControl::run()
{
  while (!_stop) 
  {
    if(_firstRealPoseReceived && _wrenchBiasOK)
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

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void ForceTaskSharedControl::stopNode(int sig)
{
  me->_stop = true;
}


void ForceTaskSharedControl::computeCommand()
{
  
  if(_filteredWrench.segment(0,3).norm() > _contactForceThreshold)
  {
    primaryTask(true);
    forceTask();
  }
  else
  {
    primaryTask(false);
    _Fc.setConstant(0.0f);
  }
}


void ForceTaskSharedControl::primaryTask(bool contact)
{
  if(fabs(_vh(0)) < 1e-3f)
  {
    _vd = -_convergenceRate*(_x-_xd);
    _vd.setConstant(0.0f);
  }
  else
  {
    _vd = _vh(0)*_wRb.col(2);
    _xd = _x;
  }

  if(!contact)
  {
    _omegad(0) = _vh(1)*_angularVhLimit/_linearVhLimit;
    Eigen::Vector4f q;
    q = _qd;
    Eigen:: Vector4f wq;
    wq << 0, _wRb.transpose()*_omegad;
    Eigen::Vector4f dq = quaternionProduct(q,wq);
    q += 0.5f*_dt*dq;
    q /= q.norm();
    _qd = q;
  }
}


void ForceTaskSharedControl::forceTask()
{

  float stiffnessGain = 2.0f;
  float dampingGain = 0.0f;
  bool updateQuaternion = false;

  // Extract linear speed, force and torque data
  Eigen::Vector3f v = _twist.segment(0,3);
  Eigen::Vector3f w = _twist.segment(3,3);
  Eigen::Vector3f F = _wRb*_filteredWrench.segment(0,3);  
  Eigen::Vector3f T = _wRb*_filteredWrench.segment(3,3);
  Eigen::Matrix3f P = Eigen::Matrix3f::Identity()-_wRb.col(2)*_wRb.col(2).transpose();

  Eigen::Vector3f Ft,Fn;
  Eigen::Vector3f vu = _vh(1)*_wRb.col(1);

  if(vu.norm()>1e-3f)// || (force.norm() > _targetForce && _twist.segment(0,3).norm() > 1e-3f))
  {
    Ft = P*F;
  }
  else
  {
    Ft.setConstant(0.0f);
  }
  
  Fn = F-Ft;
  _forceNorm = Fn.norm();
  _normalDirection = Fn.normalized();
  _e1 = -_normalDirection;
  Eigen::Matrix3f S = _e1*_e1.transpose();

  // Compute rotation error between current orientation and normal direction using Rodrigues' law
  Eigen::Matrix3f K;
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_normalDirection);
  float c = (-_wRb.col(2)).transpose()*_normalDirection;  
  float s = k.norm();
  k /= s;
  K << getSkewSymmetricMatrix(k);

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Convert rotation error into axis angle representation
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);

  // Convert to quaternion to axis angle representation
  float angle;
  Eigen::Vector3f omega;
  quaternionToAxisAngle(qtemp,omega,angle);

  Eigen::Vector3f damping;
  damping =-fabs(angle)*w;

  if(fabs(angle)<M_PI/3.0f)
  {
    if(updateQuaternion)
    {
      Eigen::Vector4f q, wq;
      q = _q;
      wq << 0, _wRb.transpose()*(stiffnessGain*omega + dampingGain*damping);
      Eigen::Vector4f dq = quaternionProduct(q,wq);
      q += 0.5f*_dt*dq;
      q /= q.norm();
      _qd = q;   
    }
    else
    {
      _qd = _q;
    }

    _omegad = stiffnessGain*omega + dampingGain*damping;

    // Compute projected force and speed along plane normal
    Eigen::Vector3f vn = S*v;

    // Compute desired force along surface normal
    Eigen::Vector3f Fd = _targetForce*_e1;

    // Check if force control activated and compute contact force following the specified dynamics
    if(_controlForce)
    {
      // Compute force error
      Eigen::Vector3f Fe = Fd-(-Fn);
      
      // Compute force stiffness rate gain from measured speed
      float alpha = 1-std::tanh(50*vn.norm());

      // Integrate contact force magnitude dynamics
      _Fc += _dt*(_k1*alpha*Fe-_k2*(1-alpha)*Fd);

      if(_Fc.dot(_e1) < _minFc*alpha)
      {
        _Fc = _minFc*alpha*_e1;
      }
      else if(_Fc.dot(_e1) > _maxFc)
      {
        _Fc = _maxFc*_e1;
      }

      std::cerr << "contact force: " << _Fc.transpose() << " fc: " << _Fc.norm() << " vn: " << vn.norm() << std::endl;
    }
    else
    {
      _Fc.setConstant(0.0f);
    }


    float ch = _agreementWeight*(_Fc.normalized()).dot(_vd.normalized());
    float ct = 0.0f;
    if(_forceNorm > _targetForce)
    {
        ct = 1.0f;
    }
    else
    {
      ct = (-cos(_forceNorm*M_PI/(_targetForce))+1)/2.0f;
    }

    c = ch+ct;

    _alpha = c;

    if(_alpha>_arbitrationLimit)
    {
      _alpha = _arbitrationLimit;
    }
    else if(_alpha < 0.0f)
    {
      _alpha = 0.0f;
    }

    _vd = (1-_alpha)*_vd+_vh(1)*_wRb.col(1); 
  }
  else
  {
    _Fc.setConstant(0.0f);
  }
}


Eigen::Matrix3f ForceTaskSharedControl::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f ForceTaskSharedControl::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Vector4f ForceTaskSharedControl::rotationMatrixToQuaternion(Eigen::Matrix3f R)
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


Eigen::Matrix3f ForceTaskSharedControl::quaternionToRotationMatrix(Eigen::Vector4f q)
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


void ForceTaskSharedControl::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
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


Eigen::Vector4f ForceTaskSharedControl::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
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


void ForceTaskSharedControl::processFootMouseData(void)
{
  uint8_t event;
  int buttonState, relX, relY;
  bool newEvent = false;
  float filteredRelX = 0.0f, filteredRelY = 0.0f;
  // If new event received update last event, otherwhise keep the last one

  if(_msgFootMouse.event > 0)
  {
    relX = _msgFootMouse.relX;
    relY = _msgFootMouse.relY;
    filteredRelX = _msgFootMouse.filteredRelX;
    filteredRelY = _msgFootMouse.filteredRelY;
    _lastEvent = _msgFootMouse.event;
    buttonState = _msgFootMouse.buttonState;
    newEvent = true;
  }
  else
  {
    relX = 0;
    relY = 0;
    filteredRelX = 0;
    filteredRelY = 0;
    buttonState = 0;
    newEvent = false;
  }

  event = _lastEvent;

  // Process corresponding event
  switch(event)
  {
    case foot_interfaces::FootMouseMsg::FM_BTN_A:
    {
      processABButtonEvent(buttonState,newEvent,-1.0f);
      break;
    }
    case foot_interfaces::FootMouseMsg::FM_BTN_B:
    {
      processABButtonEvent(buttonState,newEvent,1.0f);
      break;
    }
    case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
    {
      processRightClickEvent(buttonState,newEvent);
      break;
    }
    case foot_interfaces::FootMouseMsg::FM_CURSOR:
    {
      processCursorEvent(filteredRelX,filteredRelY,newEvent);
      break;
    }
    default:
    {
      break;
    }
  }
}


void ForceTaskSharedControl::processABButtonEvent(int value, bool newEvent, int direction)
{
  if(newEvent)
  {
    if(value>0) // Button pressed
    {
      _buttonPressed = true;
    }
    else
    {
      _buttonPressed = false;
    }
  }
}


void ForceTaskSharedControl::processRightClickEvent(int value, bool newEvent)
{
  if(newEvent)
  {
    if(value>0) // Button pressed
    {
      _buttonPressed = true;
    }
    else
    {
      _buttonPressed = false;
    }
  }
}

void ForceTaskSharedControl::processCursorEvent(float relX, float relY, bool newEvent)
{
  if(!newEvent) // No new event received
  {
    // Track desired position
    _vh = -_convergenceRate*(_x-_xd);
  }
  else
  {
    if(relX >-5 && relX <5)
    {
      relX = 0;
    }

    if(relY >-5 && relY <5)
    {
      relY = 0;
    }
    // Update desired x,y position
    _xd(0) = _x(0);
    _xd(1) = _x(1);

    // Compute desired x, y velocities along x, y axis of world frame
    if(relX>MAX_XY_REL)
    {
      _vh(1) = -_linearVhLimit;
    }
    else if(relX < -MAX_XY_REL)
    {
      _vh(1) = _linearVhLimit;
    }
    else
    {
      _vh(1) = -_linearVhLimit*relX/MAX_XY_REL;
    }

    if(relY>MAX_XY_REL)
    {
      _vh(0) = -_linearVhLimit;
    }
    else if(relY < -MAX_XY_REL)
    {
      _vh(0) = _linearVhLimit;
    }
    else
    {
      _vh(0) = -_linearVhLimit*relY/MAX_XY_REL;
    }

    if(!_buttonPressed)
    {
      _vh(2) = 0.0f;
    }
  } 
}



void ForceTaskSharedControl::publishData()
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

  _msgArrowMarker.points.clear();
  geometry_msgs::Point p1,p2;
  p1.x = _x(0);
  p1.y = _x(1);
  p1.z = _x(2);
  p2.x = _x(0)+0.3f*_e1(0);
  p2.y = _x(1)+0.3f*_e1(1);
  p2.z = _x(2)+0.3f*_e1(2);
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
  _pubMarker.publish(_msgArrowMarker);

  std_msgs::Float32 msg;
  msg.data = _forceNorm;
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

  _msgDesiredWrench.force.x = _alpha*_Fc(0);
  _msgDesiredWrench.force.y = _alpha*_Fc(1);
  _msgDesiredWrench.force.z = _alpha*_Fc(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);
}

void ForceTaskSharedControl::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
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

 
void ForceTaskSharedControl::updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
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


void ForceTaskSharedControl::updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;
}


void ForceTaskSharedControl::updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg)
{
	_msgFootMouse = *msg;

	if(!_firstEventReceived && _msgFootMouse.event > 0)
	{
		_firstEventReceived = true;
	}
}


void ForceTaskSharedControl::dynamicReconfigureCallback(motion_force_control::forceTaskSharedControl_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  _filteredForceGain = config.filteredForceGain;
  _contactForceThreshold = config.contactForceThreshold;
  _targetForce = config.targetForce;
  _controlForce = config.controlForce;
  _k1 = config.k1;
  _k2 = config.k2;
  _minFc = config.minFc;
  _maxFc = config.maxFc;
  _linearVhLimit = config.linearVhLimit;
  _angularVhLimit = config.angularVhLimit;
  _arbitrationLimit = config.arbitrationLimit;
  _agreementWeight = config.agreementWeight;
}
