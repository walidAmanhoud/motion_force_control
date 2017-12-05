#include "MoveToDesiredPose.h"
// #include <tf/transform_datatypes.h>

MoveToDesiredPose* MoveToDesiredPose::me = NULL;

MoveToDesiredPose::MoveToDesiredPose(ros::NodeHandle &n, float frequency, float jointTolerance):
	_n(n),
  _loopRate(frequency),
	_jointTolerance(jointTolerance)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredPose::init() 
{
  me = this;
  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);
  _firstRealPoseReceived = false;
  _stop = false;


  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &MoveToDesiredPose::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

  signal(SIGINT,MoveToDesiredPose::stopNode);


	if (_n.ok())
	{ 
	  // Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void MoveToDesiredPose::run() {

  while (!_stop) 
  {
    if(_firstRealPoseReceived)
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

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void MoveToDesiredPose::stopNode(int sig)
{
  me->_stop = true;
}


void MoveToDesiredPose::setDesiredPose(Eigen::Vector3f desiredPosition) 
{
	_xd = desiredPosition;
}


void MoveToDesiredPose::computeCommand() 
{
	_vd = 3.0f*(_xd-_x);
	_omegad.setConstant(0.0f);

	std::cerr << _vd.transpose() << std::endl;
	std::cerr << _xd.transpose() << std::endl;
	std::cerr << _x.transpose() << std::endl;

  // Bound desired velocity  
  if(_vd.norm()>0.3f)
  {
    _vd *= 0.3f/_vd.norm();
  }
}


void MoveToDesiredPose::publishData()
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
}


void MoveToDesiredPose::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
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
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}

Eigen::Matrix3f MoveToDesiredPose::quaternionToRotationMatrix(Eigen::Vector4f q)
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