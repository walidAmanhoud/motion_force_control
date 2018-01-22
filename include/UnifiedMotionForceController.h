#ifndef __UNIFIED_MOTION_FORCE_CONTROLLER_H__
#define __UNIFIED_MOTION_FORCE_CONTROLLER_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include <dynamic_reconfigure/server.h>

#include "motion_force_control/unifiedMotionForceController_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"

#include "PassiveDsForceController.h"

#define NB_SAMPLES 50
#define MAX_XY_REL 300
#define MAX_FRAME 200

class UnifiedMotionForceController 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRealPose;						// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor;				// Subscribe to robot current pose
		ros::Subscriber _subRealTwist;						// Subscribe to robot current twist
		ros::Publisher _pubMarker;
		ros::Publisher _pubTaskAttractor;
		ros::Publisher _pubForceNorm;
		ros::Publisher _pubFilteredWrench;
		ros::Publisher _pubDesiredPose;         // Publish desired pose
		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredWrench;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation

		ros::Subscriber _subOptitrackRobotBasisPose;
		ros::Subscriber _subOptitrackPlane1Pose;
		ros::Subscriber _subOptitrackPlane2Pose;
		ros::Subscriber _subOptitrackPlane3Pose;

		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		visualization_msgs::Marker _msgMarker;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		

		// Tool variables
		float _loadMass = 0.132f;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;
		float _toolOffset = 0.114f;
		// float _toolOffset = 0.0f;

		// End effector state variables
		Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
		Eigen::Vector3f _x;				// Current position [m] (3x1)
		Eigen::Vector4f _q;				// Current end effector quaternion (4x1)
		Eigen::Matrix<float,6,1> _wrench;
		Eigen::Matrix<float,6,1> _wrenchBias;
		Eigen::Matrix<float,6,1> _filteredWrench;
		Eigen::Matrix<float,6,1> _twist;
		float _filteredForceGain;
		int _wrenchCount = 0;

		// End effector desired variables
		Eigen::Vector4f _qd;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector3f _vd;				// Desired velocity [m/s] (3x1)
		float _targetForce;

		// Task variables
		Eigen::Vector3f _attractorPosition;				// Current position [m] (3x1)
		Eigen::Vector3f _taskAttractor;
    Eigen::Vector3f _planeNormal;
    Eigen::Vector3f _p;
    Eigen::Vector3f _xp;		
    Eigen::Vector3f _xa;		

		// Control variables
		float _forceNorm;
		float _contactForceThreshold;
    float _convergenceRate;       // Convergence rate of the DS
    float _k1;
    float _k2;
		Eigen::Vector3f _Fc;
		float _minFc;
		float _maxFc;
		float _lambda1;
		float _lambda2;
		
    // Booleans
		bool _firstRealPoseReceived;	// Monitor the first robot pose update
		bool _firstWrenchReceived;
		bool _wrenchBiasOK;
  	bool _stop = false;
		bool _linear;
    bool _polishing;
		bool _controlForce;

    // Optitrack variables
		Eigen::Vector3f _robotBasisPosition;
		Eigen::Vector3f _plane1Position;
		Eigen::Vector3f _plane2Position;
		Eigen::Vector3f _plane3Position;
		bool _firstRobotBasisPose;
		bool _firstPlane1Pose;
		bool _firstPlane2Pose;
		bool _firstPlane3Pose;

		Eigen::Vector3f _p1;
		Eigen::Vector3f _p2;
		Eigen::Vector3f _p3;

		// Other variables
		PassiveDsForceController _controller;
		static UnifiedMotionForceController* me;
		std::mutex _mutex;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<motion_force_control::unifiedMotionForceController_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<motion_force_control::unifiedMotionForceController_paramsConfig>::CallbackType _dynRecCallback;


	public:

		// Class constructor
		UnifiedMotionForceController(ros::NodeHandle &n, double frequency);

		bool init();

		void run();

	private:
		
	static void stopNode(int sig);
		
    void computeCommand();
    
    void autonomousControl();

    void publishData();

    void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void dynamicReconfigureCallback(motion_force_control::unifiedMotionForceController_paramsConfig &config, uint32_t level);

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

  	Eigen::Vector4f slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t);

		void updateRobotBasisPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updatePlane1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updatePlane2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void updatePlane3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

		Eigen::Vector3f getDesiredVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);
};


#endif
