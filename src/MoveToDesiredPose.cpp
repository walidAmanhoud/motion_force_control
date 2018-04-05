#include "MoveToDesiredPose.h"
// #include <tf/transform_datatypes.h>

MoveToDesiredPose* MoveToDesiredPose::me = NULL;

MoveToDesiredPose::MoveToDesiredPose(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _kinematicChain(NB_JOINTS,1.0f/frequency),
  _filter(0.05f),
  _passiveDSController(3,0.0f,0.0f)
{
	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredPose::init() 
{
  me = this;
  _x.setConstant(0.0f);
  _v.setConstant(0.0f);
  _omega.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _firstRobotPoseReceived = false;
  _firstRobotJointStateReceived = false;
  _firstUpdateState = false;
  _stop = false;
  _toolOffset = 0.0f;

  _jointAngles.resize(NB_JOINTS);
  _previousJointAngles.resize(NB_JOINTS);
  _jointVelocities.resize(NB_JOINTS);
  _filteredJointVelocities.resize(NB_JOINTS);
  _jointTorques.resize(NB_JOINTS);
  _torquesd.resize(NB_JOINTS);
  _nullspaceTorquesd.resize(NB_JOINTS);

  _Fdlin.setConstant(0.0f);
  _Fdang.setConstant(0.0f);
  _Fd.setConstant(0.0f);
  _torquesd.setConstant(0.0f);
  _nullspaceTorquesd.setConstant(0.0f);

  _msgRobotJointState.position.resize(NB_JOINTS);
  _msgRobotJointState.velocity.resize(NB_JOINTS);
  _msgRobotJointState.effort.resize(NB_JOINTS);
  _msgRobotCommand.position.resize(NB_JOINTS);
  _msgRobotCommand.velocity.resize(NB_JOINTS);
  _msgRobotCommand.effort.resize(NB_JOINTS);
  _msgRobotCommand.stiffness.resize(NB_JOINTS);

  initKinematics();

  _subRobotJointState = _n.subscribe("/real_r_arm_pos_controller/joint_states", 1, &MoveToDesiredPose::updateRobotJointStates, this, ros::TransportHints().reliable().tcpNoDelay());
  _pubRobotCommand = _n.advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 1);
  _pubRobotPose = _n.advertise<geometry_msgs::Pose>("/iiwa/pose", 1);
  _pubRobotTwist = _n.advertise<geometry_msgs::Twist>("/iiwa/twist", 1);


  _dynRecCallback = boost::bind(&MoveToDesiredPose::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

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
    if(_firstRobotJointStateReceived)
    {
      _mutex.lock();

      // Update robot state
      updateRobotState();

      // Compute desired velocity
      computeDesiredVelocity();

      // Compute desired torques
      computeDesiredTorques();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _torquesd.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void MoveToDesiredPose::initKinematics()
{
  _geometricJacobian.resize(6,NB_JOINTS);

  _kinematicChain.setDH(0, 0.0f, 0.34f, M_PI_2, 0.0f, 1, DEG2RAD(-170.0f), DEG2RAD(170.0f), DEG2RAD(98.0f));
  _kinematicChain.setDH(1, 0.0f, 0.0f,-M_PI_2, 0.0f, 1, DEG2RAD(-120.0f), DEG2RAD(120.0f), DEG2RAD(98.0f));
  _kinematicChain.setDH(2, 0.0f, 0.4f,-M_PI_2, 0.0f, 1, DEG2RAD(-170.0f), DEG2RAD(170.0f), DEG2RAD(100.0f));
  _kinematicChain.setDH(3, 0.0f, 0.0f, M_PI_2, 0.0f, 1, DEG2RAD(-120.0f), DEG2RAD(120.0f), DEG2RAD(120.0f));
  _kinematicChain.setDH(4, 0.0f, 0.4f, M_PI_2, 0.0f, 1, DEG2RAD(-170.0f), DEG2RAD(170.0f), DEG2RAD(140.0f));
  _kinematicChain.setDH(5, 0.0f, 0.0f,-M_PI_2, 0.0f, 1, DEG2RAD(-120.0f), DEG2RAD(120.0f), DEG2RAD(180.0f));
  _kinematicChain.setDH(6, 0.05f, 0.126f+0.14f, 0.0f, 0.0, 1,DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0f));

  _kinematicChain.readyForKinematics();

  double T0[4][4];
  for(int k=0; k<4; k++)
  {
    for(int m=0; m<4; m++)
    {
      T0[k][m] = 0.0;
    }
  }

  T0[0][0] = 1;
  T0[1][1] = 1;
  T0[2][2] = 1;
  T0[3][3] = 1;

  _kinematicChain.setT0(T0);
  _kinematicChain.setTF(T0);
  _kinematicChain.readyForKinematics();
}


void MoveToDesiredPose::updateRobotState()
{

  // std::cerr << "a" << std::endl;
  _kinematicChain.setJoints(_jointAngles.data());
  _kinematicChain.getEndPos(_x);


    // std::cerr << "b" << std::endl;

  Eigen::Vector3d axis;
  for(int k = 0; k < 3; k++)
  {
    _kinematicChain.getEndDirAxis(k,axis);
    _wRb.col(k) = axis;
  }

    // std::cerr << "c" << std::endl;

  _q = rotationMatrixToQuaternion(_wRb);

  _x = _x+_toolOffset*_wRb.col(2);

    // std::cerr << "d" << std::endl;


  _kinematicChain.getJacobian(_geometricJacobian); 

  _jointVelocities = (_jointAngles-_previousJointAngles)/_dt;
  _filteredJointVelocities = _filter.filter(_jointVelocities);


  // std::cerr << "e" << std::endl;

  Eigen::Matrix<double,6,1> twist;
  twist = _geometricJacobian*_filteredJointVelocities;
  _v = twist.segment(0,3);
  _omega = twist.segment(3,3);

  _previousJointAngles = _jointAngles;

  if(!_firstUpdateState)
  {
    _qd = _q;
    _wRbd = _wRb;
    _firstUpdateState = true;
  }

}


void MoveToDesiredPose::stopNode(int sig)
{
  me->_stop = true;
}


void MoveToDesiredPose::setDesiredPose(Eigen::Vector3d desiredPosition) 
{
	_xd = desiredPosition;
}


void MoveToDesiredPose::computeDesiredVelocity() 
{
	_vd = 3.0f*(_xd-_x);
	_omegad.setConstant(0.0f);

	std::cerr << _vd.transpose() << std::endl;
	std::cerr << _xd.transpose() << std::endl;
	std::cerr << _x.transpose() << std::endl;

  // Bound desired velocity  
  if(_vd.norm()>0.2f)
  {
    _vd *= 0.2f/_vd.norm();
  }

  _omegad.setConstant(0.0f);
}

void MoveToDesiredPose::computeDesiredTorques() 
{
  _passiveDSController.Update(_v,_vd);
  _Fdlin = _passiveDSController.control_output();

  Eigen::Matrix3d Re;
  Re = _wRbd*_wRb.inverse();

  Eigen::Vector4d qe;
  Eigen::Vector3d axis;
  double angle;
  qe = rotationMatrixToQuaternion(Re);
  quaternionToAxisAngle(qe,axis,angle);

  // std::cerr << "Re: " << Re <<std::endl;
  // std::cerr << "wRbd: " << _wRbd <<std::endl;
  // std::cerr << "wRb: " << _wRb <<std::endl;
  // std::cerr << "qe: " << qe.transpose() <<std::endl;
  // std::cerr << "r: " <<_rotationalStiffness << std::endl;
  // std::cerr << "angle: " <<angle << std::endl;
  // std::cerr << "axis: " <<axis.transpose() << std::endl;
  // std::cerr << "rd: " <<_rotationalDamping << std::endl;
  // std::cerr << "od: " <<_omegad.transpose() << std::endl;
  // std::cerr << "o: " <<_omega.transpose() << std::endl;

  _Fdang = _rotationalStiffness*angle*axis+_rotationalDamping*(_omegad-_omega);

  _Fd.segment(0,3) = _Fdlin;
  _Fd.segment(3,3) = _Fdang;

  Eigen::Matrix<double,6,NB_JOINTS> pseudoInverseJacobian;
  pseudoInverseJacobian = getPseudoInverse(_geometricJacobian.transpose());

  Eigen::Matrix<double,NB_JOINTS,NB_JOINTS> nullspace;
  nullspace = (Eigen::MatrixXd::Identity(NB_JOINTS,NB_JOINTS)-_geometricJacobian.transpose()*pseudoInverseJacobian);
  _nullspaceTorquesd = nullspace*(_jointLimitsGain*_jointAngles+_jointVelocitiesGain*_filteredJointVelocities);

  _torquesd = _geometricJacobian.transpose()*_Fd;


  std::cerr << _Fd.transpose() << std::endl;
  std::cerr << _torquesd.transpose() << std::endl;

  if(_useNullSpace)
  {
    _torquesd += _nullspaceTorquesd;
  }
}


void MoveToDesiredPose::publishData()
{
  // Publish desired twist (passive ds controller)
  // _msgDesiredTwist.linear.x  = _vd(0);
  // _msgDesiredTwist.linear.y  = _vd(1);
  // _msgDesiredTwist.linear.z  = _vd(2);

  // // Convert desired end effector frame angular velocity to world frame
  // _msgDesiredTwist.angular.x = _omegad(0);
  // _msgDesiredTwist.angular.y = _omegad(1);
  // _msgDesiredTwist.angular.z = _omegad(2);

  // _pubDesiredTwist.publish(_msgDesiredTwist);

  // // Publish desired orientation
  // _msgDesiredOrientation.w = _qd(0);
  // _msgDesiredOrientation.x = _qd(1);
  // _msgDesiredOrientation.y = _qd(2);
  // _msgDesiredOrientation.z = _qd(3);

  // _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgRobotPose.position.x = _x(0);
  _msgRobotPose.position.y = _x(1);
  _msgRobotPose.position.z = _x(2);
  _msgRobotPose.orientation.w = _q(0);
  _msgRobotPose.orientation.x = _q(1);
  _msgRobotPose.orientation.y = _q(2);
  _msgRobotPose.orientation.z = _q(3);

  _pubRobotPose.publish(_msgRobotPose);

  _msgRobotTwist.linear.x = _v(0);
  _msgRobotTwist.linear.y = _v(1);
  _msgRobotTwist.linear.z = _v(2);
  _msgRobotTwist.angular.x = _omega(0);
  _msgRobotTwist.angular.y = _omega(1);
  _msgRobotTwist.angular.z = _omega(2);

  _pubRobotTwist.publish(_msgRobotTwist);

  for(int k = 0; k < NB_JOINTS; k++)
  {
    _msgRobotCommand.effort[k] = _torquesd(k);
    _msgRobotCommand.position[k] = _jointAngles(k);
    _msgRobotCommand.velocity[k] = _filteredJointVelocities(k);
    _msgRobotCommand.stiffness[k] = 0.0f;
  }

  _pubRobotCommand.publish(_msgRobotCommand);
}


void MoveToDesiredPose::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRobotPose = *msg;

  // Update end effecotr pose (position+orientation)
  _x << _msgRobotPose.position.x, _msgRobotPose.position.y, _msgRobotPose.position.z;
  _q << _msgRobotPose.orientation.w, _msgRobotPose.orientation.x, _msgRobotPose.orientation.y, _msgRobotPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if(!_firstRobotPoseReceived)
  {
    _firstRobotPoseReceived = true;
    _qd = _q;
    _wRbd = _wRb;
  }
}


void MoveToDesiredPose::updateRobotJointStates(const sensor_msgs::JointState::ConstPtr& msg)
{

  _msgRobotJointState = *msg;

  // Update end effecotr pose (position+orientation)
  for(int k = 0; k < NB_JOINTS; k++)
  {
    _jointAngles(k) = _msgRobotJointState.position[k];
    _jointTorques(k) = _msgRobotJointState.effort[k];
  }


  if(!_firstRobotJointStateReceived)
  {
    _previousJointAngles = _jointAngles;
    _jointVelocities.setConstant(0.0f);
    _filteredJointVelocities = _jointVelocities;
    _firstRobotJointStateReceived = true;
  }
}


Eigen::Matrix3d MoveToDesiredPose::quaternionToRotationMatrix(Eigen::Vector4d q)
{
  Eigen::Matrix3d R;

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


Eigen::Vector4d MoveToDesiredPose::rotationMatrixToQuaternion(Eigen::Matrix3d R)
{
  Eigen::Vector4d q;

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


void MoveToDesiredPose::quaternionToAxisAngle(Eigen::Vector4d q, Eigen::Vector3d &axis, double &angle)
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


Eigen::MatrixXd MoveToDesiredPose::getPseudoInverse(Eigen::MatrixXd M, bool damped)
{ 
  double lambda = damped?0.2f:0.0f;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues = svd.singularValues();
  
  Eigen::MatrixXd S = M;
  S.setZero();

  for (int k = 0; k < singularValues.size(); k++)
  {  
    S(k,k) = (singularValues(k))/(singularValues(k)*singularValues(k) + lambda*lambda);
  }

  return svd.matrixV()*S.transpose()*svd.matrixU().transpose();
}


void MoveToDesiredPose::dynamicReconfigureCallback(motion_force_control::moveToDesiredPose_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _lambda1 = config.lambda1;
  _lambda2 = config.lambda2;
  _rotationalStiffness = config.rotationalStiffness;
  _rotationalDamping = config.rotationalDamping;
  _useNullSpace = config.useNullSpace;
  _jointLimitsGain = config.jointLimitsGain;
  _desiredJointsGain = config.desiredJointsGain;
  _jointVelocitiesGain = config.jointVelocitiesGain;

  _passiveDSController.set_damping_eigval(_lambda1,_lambda2);

}