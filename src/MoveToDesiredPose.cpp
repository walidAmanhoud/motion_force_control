#include "MoveToDesiredPose.h"
// #include <tf/transform_datatypes.h>

MoveToDesiredPose* MoveToDesiredPose::me = NULL;

MoveToDesiredPose::MoveToDesiredPose(ros::NodeHandle &n, float frequency, bool bimanual):
	_n(n),
  _loopRate(frequency),
  _bimanual(bimanual)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredPose::init() 
{
  me = this;

  for(int k = 0; k < 2; k++)
  {  
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _xd[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _firstRealPoseReceived[k] = false;
  }

  if(!_bimanual)
  {
    _firstRealPoseReceived[LEFT] = true;
  }

  _stop = false;
  _toolOffset = 0.14f;

  _subRealPose[RIGHT] = _n.subscribe("/lwr/ee_pose", 1, &MoveToDesiredPose::updateRealPoseRight, this, ros::TransportHints().reliable().tcpNoDelay());
  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

  if(_bimanual)
  {
    _subRealPose[LEFT] = _n.subscribe("/lwr2/ee_pose", 1, &MoveToDesiredPose::updateRealPoseLeft, this, ros::TransportHints().reliable().tcpNoDelay());
    _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  }


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
    if(_firstRealPoseReceived[LEFT] && _firstRealPoseReceived[RIGHT])
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

  _vd[LEFT].setConstant(0.0f);
  _omegad[LEFT].setConstant(0.0f);
  _qd[LEFT] = _q[LEFT];

  _vd[RIGHT].setConstant(0.0f);
  _omegad[RIGHT].setConstant(0.0f);
  _qd[RIGHT] = _q[RIGHT];


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
  _xd[LEFT] = desiredPosition;
  _xd[RIGHT] = desiredPosition;  
}


void MoveToDesiredPose::computeCommand() 
{
  _vd[RIGHT] = 3.0f*(_xd[RIGHT]-_x[RIGHT]);
  _omegad[RIGHT].setConstant(0.0f);

  // std::cerr << "RIGHT: " <<_x.transpose() << std::endl;
  // std::cerr << "LEFT: " <<_x.transpose() << std::endl;

  // Bound desired velocity  
  if(_vd[RIGHT].norm()>0.2f)
  {
    _vd[RIGHT] *= 0.2f/_vd[RIGHT].norm();
  }

  _vd[LEFT] = 3.0f*(_xd[LEFT]-_x[LEFT]);
  _omegad[LEFT].setConstant(0.0f);

  // Bound desired velocity  
  if(_vd[LEFT].norm()>0.2f)
  {
    _vd[LEFT] *= 0.2f/_vd[LEFT].norm();
  }
}


void MoveToDesiredPose::publishData()
{


  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd[RIGHT](0);
  _msgDesiredTwist.linear.y  = _vd[RIGHT](1);
  _msgDesiredTwist.linear.z  = _vd[RIGHT](2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad[RIGHT](0);
  _msgDesiredTwist.angular.y = _omegad[RIGHT](1);
  _msgDesiredTwist.angular.z = _omegad[RIGHT](2);

  _pubDesiredTwist[RIGHT].publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd[RIGHT](0);
  _msgDesiredOrientation.x = _qd[RIGHT](1);
  _msgDesiredOrientation.y = _qd[RIGHT](2);
  _msgDesiredOrientation.z = _qd[RIGHT](3);

  _pubDesiredOrientation[RIGHT].publish(_msgDesiredOrientation);

  if(_bimanual)
  {
    _msgDesiredTwist.linear.x  = _vd[LEFT](0);
    _msgDesiredTwist.linear.y  = _vd[LEFT](1);
    _msgDesiredTwist.linear.z  = _vd[LEFT](2);

    // Convert desired end effector frame angular velocity to world frame
    _msgDesiredTwist.angular.x = _omegad[LEFT](0);
    _msgDesiredTwist.angular.y = _omegad[LEFT](1);
    _msgDesiredTwist.angular.z = _omegad[LEFT](2);

    _pubDesiredTwist[LEFT].publish(_msgDesiredTwist);

    // Publish desired orientation
    _msgDesiredOrientation.w = _qd[LEFT](0);
    _msgDesiredOrientation.x = _qd[LEFT](1);
    _msgDesiredOrientation.y = _qd[LEFT](2);
    _msgDesiredOrientation.z = _qd[LEFT](3);

    _pubDesiredOrientation[LEFT].publish(_msgDesiredOrientation);
  }
}


void MoveToDesiredPose::updateRealPoseRight(const geometry_msgs::Pose::ConstPtr& msg)
{

  // Update end effecotr pose (position+orientation)
  _x[RIGHT] << msg->position.x, msg->position.y, msg->position.z;
  _q[RIGHT] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[RIGHT] = quaternionToRotationMatrix(_q[RIGHT]);
  _x[RIGHT] = _x[RIGHT]+_toolOffset*_wRb[RIGHT].col(2);

  if(!_firstRealPoseReceived[RIGHT])
  {
    _firstRealPoseReceived[RIGHT] = true;
    _qd[RIGHT] = _q[RIGHT];
    _vd[RIGHT].setConstant(0.0f);
  }
}


void MoveToDesiredPose::updateRealPoseLeft(const geometry_msgs::Pose::ConstPtr& msg)
{

  // Update end effecotr pose (position+orientation)
  _x[LEFT] << msg->position.x, msg->position.y, msg->position.z;
  _q[LEFT] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[LEFT] = quaternionToRotationMatrix(_q[LEFT]);
  _x[LEFT] = _x[LEFT]+_toolOffset*_wRb[LEFT].col(2);

  if(!_firstRealPoseReceived[LEFT])
  {
    _firstRealPoseReceived[LEFT] = true;
    _qd[LEFT] = _q[LEFT];
    _vd[LEFT].setConstant(0.0f);
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