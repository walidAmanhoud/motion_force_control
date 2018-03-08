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

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subRealPose;			// Subscribe to robot current pose
		ros::Publisher _pubDesiredTwist;		// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation

		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;

		// Node variables
		Eigen::Vector3f _x;
		Eigen::Vector3f _xd;
		Eigen::Vector4f _qd;
		Eigen::Vector4f _q;
		Eigen::Matrix3f _wRb;
		Eigen::Vector3f _omegad;
		Eigen::Vector3f _vd;
		float _jointTolerance;
		bool _firstRealPoseReceived;
		float _toolOffset;


		// Class variables
		std::mutex _mutex;

		bool _stop = false;
		static MoveToDesiredPose* me;

	public:
		MoveToDesiredPose(ros::NodeHandle &n, float frequency, float jointTolerance = 1.0e-3f);

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

    	void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

};


#endif
