#ifndef __SURFACE_LEARNING_H__
#define __SURFACE_LEARNING_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include <dynamic_reconfigure/server.h>

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "armadillo"
#include "svm_grad.h"

#define NB_SAMPLES 50

class SurfaceLearning 
{
	public:

		enum Mode {LOGGING = 0, TESTING = 1};

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRobotPose;						// Subscribe to robot current pose
		ros::Subscriber _subRobotTwist;						// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor;				// Subscribe to robot current pose

		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
		ros::Publisher _pubDesiredWrench;				// Publish desired twist
		ros::Publisher _pubFilteredWrench;
		ros::Publisher _pubMarker;
		
		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		visualization_msgs::Marker _msgArrowMarker;

		// Tool variables
		float _loadMass;
		float _toolOffset;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;

		// End effector state variables
		Eigen::Vector3f _x;				// Current position [m] (3x1)
		Eigen::Vector4f _q;				// Current end effector quaternion (4x1)
		Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
		Eigen::Vector3f _v;
		Eigen::Vector3f _w;
		Eigen::Matrix<float,6,1> _wrench;
		Eigen::Matrix<float,6,1> _wrenchBias;
		Eigen::Matrix<float,6,1> _filteredWrench;
		float _filteredForceGain;
		int _wrenchCount = 0;
		float _normalDistance;

		// End effector desired variables
		Eigen::Vector4f _qd;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _omegad;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _xd;				// Desired position [m] (3x1)
		Eigen::Vector3f _vd;				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _e1;
		Eigen::Vector3f _e2;
		Eigen::Vector3f _e3;
		Eigen::Vector3f _xAttractor;
		float _lambda1;
		float _Fd;


		// Control variables
    float _convergenceRate;       // Convergence rate of the DS
		Eigen::Vector3f _Fc;
		
    // Booleans
		bool _firstRobotPose;	// Monitor the first robot pose update
		bool _firstRobotTwist;	// Monitor the first robot pose update
		bool _firstWrenchReceived;
		bool _wrenchBiasOK;
  	bool _stop;

    uint32_t _sequenceID;

		std::string _fileName;


		static SurfaceLearning* me;
		std::mutex _mutex;


		std::ofstream _outputFile;
		std::ifstream _inputFile;
		SVMGrad _svm;
		Mode _mode;

		Eigen::Vector3f _vdOrig;

		Eigen::Vector3f _vdR;



	public:

		// Class constructor
		SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, Mode mode);

		bool init();

		void run();

	private:
		
	static void stopNode(int sig);
		
    void computeCommand();

		void computeOriginalDynamics();

		Eigen::Vector3f getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);

		void rotatingDynamics();

		void forceModulation();

		void computeDesiredOrientation();
    
    void logData();

    void publishData();

    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

  	Eigen::Vector4f slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t);
};


#endif
