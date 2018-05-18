#include "MoveToDesiredPose.h"
#include "Utils.h"
#include <boost/bind.hpp>

MoveToDesiredPose* MoveToDesiredPose::me = NULL;

MoveToDesiredPose::MoveToDesiredPose(ros::NodeHandle &n, float frequency, Mode mode):
	_n(n),
  _loopRate(frequency),
  _mode(mode)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredPose::init() 
{
  me = this;

  for(int k = 0; k < NB_ROBOTS; k++)
  {  
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _xd[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _firstRealPoseReceived[k] = false;
  }

  if(_mode == SINGLE_LEFT)
  {
    _firstRealPoseReceived[RIGHT] = true; 
    std::cerr <<"bou2" << std::endl;
  }
  else if(_mode == SINGLE_RIGHT)
  {
    _firstRealPoseReceived[LEFT] = true;
  }

  _stop = false;
  _toolOffset = 0.15f;


  _subRealPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1,boost::bind(&MoveToDesiredPose::updateRealPose,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

  _subRealPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&MoveToDesiredPose::updateRealPose,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);


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

  if(_mode == SINGLE_RIGHT)
  {
    _vd[RIGHT] = 3.0f*(_xd[RIGHT]-_x[RIGHT]);
  }
  else if(_mode == SINGLE_LEFT)
  {
    _vd[LEFT] = 3.0f*(_xd[LEFT]-_x[LEFT]);
  }
  else if (_mode == BOTH)
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    {
      _vd[k] = 3.0*(_xd[k]-_x[k]);
    }
  }

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_vd[k].norm()>0.2f)
    {
      _vd[k] *= 0.2f/_vd[k].norm();
    }
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


  // Publish desired orientation
  _msgDesiredOrientation.w = _qd[RIGHT](0);
  _msgDesiredOrientation.x = _qd[RIGHT](1);
  _msgDesiredOrientation.y = _qd[RIGHT](2);
  _msgDesiredOrientation.z = _qd[RIGHT](3);

  if(_mode == BOTH || _mode == SINGLE_RIGHT)
  {
    _pubDesiredTwist[RIGHT].publish(_msgDesiredTwist);
    _pubDesiredOrientation[RIGHT].publish(_msgDesiredOrientation);
  }

  _msgDesiredTwist.linear.x  = _vd[LEFT](0);
  _msgDesiredTwist.linear.y  = _vd[LEFT](1);
  _msgDesiredTwist.linear.z  = _vd[LEFT](2);

  // Convert desired end effector frame angular velocity to world frame
  _msgDesiredTwist.angular.x = _omegad[LEFT](0);
  _msgDesiredTwist.angular.y = _omegad[LEFT](1);
  _msgDesiredTwist.angular.z = _omegad[LEFT](2);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd[LEFT](0);
  _msgDesiredOrientation.x = _qd[LEFT](1);
  _msgDesiredOrientation.y = _qd[LEFT](2);
  _msgDesiredOrientation.z = _qd[LEFT](3);

  if(_mode == BOTH || _mode == SINGLE_LEFT)
  {
   _pubDesiredTwist[LEFT].publish(_msgDesiredTwist);
    _pubDesiredOrientation[LEFT].publish(_msgDesiredOrientation);
  }
}


void MoveToDesiredPose::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffset*_wRb[k].col(2);

  if(!_firstRealPoseReceived[k])
  {
  std::cerr << k << std::endl;
    _firstRealPoseReceived[k] = true;
    _qd[k] = _q[k];
    _vd[k].setConstant(0.0f);
  }
}