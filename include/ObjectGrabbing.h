#ifndef __OBJECT_GRABBING_H__
#define __OBJECT_GRABBING_H__

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

#include "motion_force_control/modulatedDS_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"

#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define TOTAL_NB_MARKERS 6

class ObjectGrabbing 
{

	public:

		enum ROBOT {LEFT = 0, RIGHT = 1};
  	enum MarkersID {ROBOT_BASIS_LEFT = 0, ROBOT_BASIS_RIGHT = 1, P1 = 2, P2 = 3, P3 = 4, P4 = 5};

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRobotPose[2];						// Subscribe to robot current pose
		ros::Subscriber _subRobotTwist[2];						// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor[2];				// Subscribe to robot current pose
		ros::Subscriber _subOptitrackPose[TOTAL_NB_MARKERS];
		ros::Subscriber _subDampingMatrix;

		ros::Publisher _pubDesiredTwist[2];				// Publish desired twist
		ros::Publisher _pubDesiredOrientation[2];  // Publish desired orientation
		ros::Publisher _pubDesiredWrench[2];				// Publish desired twist
		ros::Publisher _pubFilteredWrench[2];
		ros::Publisher _pubNormalForce[2];
		ros::Publisher _pubMarker;
		ros::Publisher _pubTaskAttractor;
		
		// Subsciber and publisher messages declaration
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		visualization_msgs::Marker _msgMarker;
		visualization_msgs::Marker _msgArrowMarker;

		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		// Tool variables
		float _loadMass;
		float _toolOffset;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;

		// End effector state variables
		Eigen::Vector3f _x[2];				// Current position [m] (3x1)
		Eigen::Vector4f _q[2];				// Current end effector quaternion (4x1)
		Eigen::Matrix3f _wRb[2];				// Current rotation matrix [m] (3x1)
		Eigen::Vector3f _v[2];
		Eigen::Vector3f _w[2];
		Eigen::Matrix<float,6,1> _wrench[2];
		Eigen::Matrix<float,6,1> _wrenchBias[2];
		Eigen::Matrix<float,6,1> _filteredWrench[2];
		int _wrenchCount[2];
		float _filteredForceGain;

		// End effector desired variables
		Eigen::Vector4f _qd[2];				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _omegad[2];		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector3f _xd[2];				// Desired position [m] (3x1)
		Eigen::Vector3f _vd[2];				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _vdOrig[2];
		Eigen::Vector3f _vdR[2];
		float _Fd[2];
		float _targetForce;
		float _targetVelocity;
		float _velocityLimit;

		// Task variables
    Eigen::Vector3f _taskAttractor;
    Eigen::Vector3f _contactAttractor;
    Eigen::Vector3f _e1[2];
    Eigen::Vector3f _e2[2];
    Eigen::Vector3f _e3[2];
    Eigen::Vector3f _p;
    Eigen::Vector3f _xProj;		
    Eigen::Vector3f _xAttractor;  
    float _vInit; 
    float _normalDistance[2];
    float _normalForce[2];
    double _duration;
    double _timeInit;
    Eigen::Vector3f _offset;

		// Control variables
    float _convergenceRate;       // Convergence rate of the DS
		Eigen::Vector3f _Fc[2];
		Eigen::Vector3f _Tc[2];
		
    // Booleans
		bool _firstRobotPose[2];	// Monitor the first robot pose update
		bool _firstRobotTwist[2];	// Monitor the first robot pose update
		bool _firstWrenchReceived[2];
		bool _firstOptitrackPose[TOTAL_NB_MARKERS];
		bool _firstDampingMatrix;
		bool _optitrackOK;
		bool _wrenchBiasOK[2];
		bool _stop;

    // Optitrack 
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;
		Eigen::Vector3f _p1;
		Eigen::Vector3f _p2;
		Eigen::Vector3f _p3;
		Eigen::Vector3f _p4;

		// Eigen value of passive ds controller
		float _lambda1;

		// Other variables
		uint32_t _sequenceID;
		uint32_t _averageCount = 0;

		static ObjectGrabbing* me;
		std::mutex _mutex;

		// Dynamic reconfigure (server+callback)
		// dynamic_reconfigure::Server<motion_force_control::modulatedDS_paramsConfig> _dynRecServer;
		// dynamic_reconfigure::Server<motion_force_control::modulatedDS_paramsConfig>::CallbackType _dynRecCallback;

		std::ifstream _inputFile;
		std::ofstream _outputFile;

		// Tank parameters
		Eigen::Matrix3f _D;
		// float _s;
		// float _smax;
		// float _alpha;
		// float _beta;
		// float _betap;
		// float _gamma;
		// float _gammap;
		// float _ut;
		// float _vt;
	public:

		// Class constructor
		ObjectGrabbing(ros::NodeHandle &n, double frequency, float targetVelocity, float targetForce);

		bool init();

		void run();

	private:
		
		static void stopNode(int sig);
		
    void computeCommand();

		void computeProjectionOnSurface();

		void computeOriginalDynamics();

		void rotatingDynamics();

		void updateTankScalars();

		void forceModulation();

		void computeDesiredOrientation();
    
    void logData();

    void publishData();
    
		void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

		void updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k);
		    
		uint16_t checkTrackedMarker(float a, float b);
		
    void optitrackInitialization();

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

  	Eigen::Vector4f slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t);

    void dynamicReconfigureCallback(motion_force_control::modulatedDS_paramsConfig &config, uint32_t level);

    float smoothRise(float x, float a, float b);

		float smoothFall(float x, float a, float b);

		float smoothRiseFall(float x, float a, float b, float c, float d);

};


#endif
