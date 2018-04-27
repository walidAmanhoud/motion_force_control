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

class MoveToDesiredPose 
{
	private:

		enum ROBOT {LEFT = 0, RIGHT = 1};

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subRealPose[2];			// Subscribe to robot current pose
		ros::Publisher _pubDesiredTwist[2];		// Publish desired twist
		ros::Publisher _pubDesiredOrientation[2];  // Publish desired orientation

		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;

		// Node variables
		Eigen::Vector3f _x[2];
		Eigen::Vector3f _xd[2];
		Eigen::Vector4f _qd[2];
		Eigen::Vector4f _q[2];
		Eigen::Matrix3f _wRb[2];
		Eigen::Vector3f _omegad[2];
		Eigen::Vector3f _vd[2];
		float _jointTolerance;
		bool _firstRealPoseReceived[2];
		bool _bimanual;
		float _toolOffset;


		// Class variables
		std::mutex _mutex;

		bool _stop = false;
		static MoveToDesiredPose* me;

	public:
		MoveToDesiredPose(ros::NodeHandle &n, float frequency, bool bimanual);

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

    	void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg, int robotID);

    	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

};


#endif
