#ifndef __MOVE_TO_DESIRED_POSE_H__
#define __MOVE_TO_DESIRED_POSE_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#define NB_ROBOTS 2


class MoveToDesiredPose 
{

	public:
	
		enum Mode {SINGLE_LEFT = 0, SINGLE_RIGHT = 1, BOTH = 2};

	private:

		enum ROBOT {LEFT = 0, RIGHT = 1};

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subRealPose[NB_ROBOTS];			// Subscribe to robot current pose
		ros::Publisher _pubDesiredTwist[NB_ROBOTS];		// Publish desired twist
		ros::Publisher _pubDesiredOrientation[NB_ROBOTS];  // Publish desired orientation

		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;

		// Node variables
		Eigen::Vector3f _x[NB_ROBOTS];
		Eigen::Vector3f _xd[NB_ROBOTS];
		Eigen::Vector4f _qd[NB_ROBOTS];
		Eigen::Vector4f _q[NB_ROBOTS];
		Eigen::Matrix3f _wRb[NB_ROBOTS];
		Eigen::Vector3f _omegad[NB_ROBOTS];
		Eigen::Vector3f _vd[NB_ROBOTS];
		bool _firstRealPoseReceived[NB_ROBOTS];
		float _toolOffset;

		Mode _mode;

		// Class variables
		std::mutex _mutex;

		bool _stop = false;
		static MoveToDesiredPose* me;

	public:
		MoveToDesiredPose(ros::NodeHandle &n, float frequency, Mode mode);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired joint angles
		void setDesiredPose(Eigen::Vector3f desiredPosition);

	private:

		static void stopNode(int sig);

    	void computeCommand();
    
   	 	void publishData();

    	void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg, int k);
};


#endif
