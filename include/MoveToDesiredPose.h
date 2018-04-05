#ifndef __MOVE_TO_DESIRED_POSE_H__
#define __MOVE_TO_DESIRED_POSE_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "Eigen/LU"
#include "Eigen/SVD"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "sKinematics.h"
#include "exponentialsmoother.h"
#include "passive_ds_controller.h"
#include <dynamic_reconfigure/server.h>
#include "motion_force_control/moveToDesiredPose_paramsConfig.h"


#define NB_JOINTS 7

class MoveToDesiredPose 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subRobotJointState;	// Subscribe to robot joint pose
		ros::Publisher _pubRobotCommand;		// Publish robot command
		ros::Publisher _pubRobotPose;		    // Publish robot pose
		ros::Publisher _pubRobotTwist;		    // Publish robot twist
		ros::Publisher _pubDesiredTwist;		// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation


		geometry_msgs::Pose _msgRobotPose;
		geometry_msgs::Twist _msgRobotTwist;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		sensor_msgs::JointState   _msgRobotJointState;
		kuka_fri_bridge::JointStateImpedance _msgRobotCommand;

		// Node variables
		Eigen::Vector3d _x;
		Eigen::Vector3d _v;
		Eigen::Vector4d _q;
		Eigen::Matrix3d _wRb;
		Eigen::Matrix3d _wRbd;
		Eigen::Vector3d _omega;
		Eigen::Vector3d _xd;
		Eigen::Vector4d _qd;
		Eigen::Vector3d _omegad;
		Eigen::Vector3d _vd;
		Eigen::VectorXd _jointAngles;
		Eigen::VectorXd _previousJointAngles;
		Eigen::VectorXd _jointVelocities;
		Eigen::VectorXd _filteredJointVelocities;
		Eigen::VectorXd _jointTorques;
		Eigen::MatrixXd _geometricJacobian;
		Eigen::Vector3d _Fdlin;
		Eigen::Vector3d _Fdang;
		Eigen::Matrix<double,6,1> _Fd;
		Eigen::VectorXd _torquesd;
		Eigen::VectorXd _nullspaceTorquesd;


		float _jointTolerance;
		bool _firstRobotPoseReceived;
		bool _firstRobotJointStateReceived;
		bool _firstUpdateState;
		float _toolOffset;
		float _dt = 0.0f;

		sKinematics _kinematicChain;
		ExponentialSmoother<Eigen::Matrix<double,NB_JOINTS,1>> _filter;
		DSController _passiveDSController;

		double _lambda1;
		double _lambda2;
    double _rotationalStiffness;
    double _rotationalDamping;
    bool _useNullSpace;
    double _jointLimitsGain;
    double _desiredJointsGain;
    double _jointVelocitiesGain;

		// Class variables
		std::mutex _mutex;

		bool _stop = false;
		static MoveToDesiredPose* me;

		dynamic_reconfigure::Server<motion_force_control::moveToDesiredPose_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<motion_force_control::moveToDesiredPose_paramsConfig>::CallbackType _dynRecCallback;


	public:
		MoveToDesiredPose(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired joint angles
		void setDesiredPose(Eigen::Vector3d desiredPosition);

	private:

		static void stopNode(int sig);

		void initKinematics();

		void updateRobotState();

  	void computeDesiredVelocity();

  	void computeDesiredTorques();
  
 	 	void publishData();

  	void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

  	void updateRobotJointStates(const sensor_msgs::JointState::ConstPtr& msg);

  	Eigen::Matrix3d quaternionToRotationMatrix(Eigen::Vector4d q);

  	Eigen::Vector4d rotationMatrixToQuaternion(Eigen::Matrix3d R);

		void quaternionToAxisAngle(Eigen::Vector4d q, Eigen::Vector3d &axis, double &angle);

		Eigen::MatrixXd getPseudoInverse(Eigen::MatrixXd M, bool damped = true);

    void dynamicReconfigureCallback(motion_force_control::moveToDesiredPose_paramsConfig &config, uint32_t level);
};


#endif
