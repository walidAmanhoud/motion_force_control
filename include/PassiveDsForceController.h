#ifndef __PASSIVE_DS_FORCE_CONTROLLER_H__
#define __PASSIVE_DS_FORCE_CONTROLLER_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
// #include "motion_force_control/passiveDsForceController_paramsConfig.h"

#include "Eigen/Eigen"

#define MIN_VELOCITY_NORM 1.0e-6f

class PassiveDsForceController 
{

	private:

		Eigen::Matrix3f _D;
		Eigen::Matrix3f _B;
		Eigen::Matrix3f _L;
		Eigen::Vector3f _F;
		float _lambda1;
		float _lambda2;


		// // Dynamic reconfigure (server+callback)
		// dynamic_reconfigure::Server<motion_force_control::passiveDsForceController_paramsConfig> _dynRecServer;
		// dynamic_reconfigure::Server<motion_force_control::passiveDsForceController_paramsConfig>::CallbackType _dynRecCallback;


	public:
		// Class constructor
		PassiveDsForceController();

		// Update damping gains
		void updateDampingGains(float lambda1, float lambda2);

		// Do one control step
		Eigen::Vector3f step(Eigen::Vector3f vd, Eigen::Vector3f v);

	private:
		// Update damping matrix
		void updateDampingMatrix(Eigen::Vector3f vd);

		// Orthonormalize basis
		void orthonormalizeBasis();


		// Dynamic reconfigure callback
		// void dynamicReconfigureCallback(motion_force_control::passiveDsForceController_paramsConfig &config, uint32_t level);

};


#endif
