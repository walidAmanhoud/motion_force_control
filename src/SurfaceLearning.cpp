#include "SurfaceLearning.h"

SurfaceLearning* SurfaceLearning::me = NULL;

SurfaceLearning::SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName)
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
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);


  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;

  _Fc.setConstant(0.0f);

  _wrenchBiasOK = false;
  _stop = false;

  _sequenceID = 0;
}


bool SurfaceLearning::init() 
{
  // Subscriber definitions
  _subRobotPose = _n.subscribe("/lwr/ee_pose", 1, &SurfaceLearning::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _n.subscribe("/lwr/ee_vel", 1, &SurfaceLearning::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &SurfaceLearning::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("SurfaceLearning/filteredWrench", 1);


  signal(SIGINT,SurfaceLearning::stopNode);


  ROS_INFO("Filename: %s", _fileName.c_str());

  _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data/"+_fileName+"_surface_data.txt");


  if(!_outputFile.is_open())
  {
    ROS_ERROR("Cannot open data file");
    return false;
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


void SurfaceLearning::run()
{
  while (!_stop) 
  {
    if(_firstRobotPose && _firstRobotTwist && _wrenchBiasOK)
    {
      _mutex.lock();

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


void SurfaceLearning::stopNode(int sig)
{
  me->_stop = true;
}

void SurfaceLearning::computeCommand()
{

  computeDesiredOrientation();
}



void SurfaceLearning::computeDesiredOrientation()
{
  _omegad.setConstant(0.0f); 
}


void SurfaceLearning::logData()
{
    _outputFile << ros::Time::now() << " "
                << _x.transpose() << " "
                << _v.transpose() << " "
                << _filteredWrench.segment(0,3).transpose() << " "
                << _sequenceID << std::endl;
}


void SurfaceLearning::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);
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


void SurfaceLearning::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
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
    std::cerr << _sequenceID << std::endl;
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void SurfaceLearning::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
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
 
void SurfaceLearning::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
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

Eigen::Vector4f SurfaceLearning::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f SurfaceLearning::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f SurfaceLearning::rotationMatrixToQuaternion(Eigen::Matrix3f R)
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


Eigen::Matrix3f SurfaceLearning::quaternionToRotationMatrix(Eigen::Vector4f q)
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


void SurfaceLearning::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
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


Eigen::Vector4f SurfaceLearning::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
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

