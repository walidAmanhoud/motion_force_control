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

		Eigen::Matrix3f _D;		// Damping matrix (3x3)
		Eigen::Matrix3f _B;		// Canonical basis of the damping matrix (3x3)
		Eigen::Matrix3f _L;		// Diagonal matrix gathering damping values for each basis direction (3x3)
		Eigen::Vector3f _F;		// Control force [N]
		float _lambda1;			// Damping value along the direction parrallel to the desired velocity
		float _lambda2;			// Damping value along the directions orthogonal to the desired veloicty

		float _s;				// Energy tank [J]
		float _smax;		 	// Max tank level [J]
		float _ds;				// Smoothness parameter 1
		float _dz;				// Smoothness parameter 2
		float _alpha;			// Variable controlling the addition of energy to the tank from the damping
		float _beta;			// Variable controlling the addition/soustraction of energy to/from the tank based on
								// on the tank state 
		float _betaF;
		float _dt;

		// // Dynamic reconfigure (server+callback)
		// dynamic_reconfigure::Server<motion_force_control::passiveDsForceController_paramsConfig> _dynRecServer;
		// dynamic_reconfigure::Server<motion_force_control::passiveDsForceController_paramsConfig>::CallbackType _dynRecCallback;


	public:
		// Class constructor
		PassiveDsForceController(float dt);

		// Update damping gains
		void updateDampingGains(float lambda1, float lambda2);

		// Control step without contact force
		Eigen::Vector3f step(Eigen::Vector3f vd, Eigen::Vector3f v);

		// Control step with contact force
		Eigen::Vector3f step(Eigen::Vector3f vd, Eigen::Vector3f v, Eigen::Vector3f Fc);

		// Return damping matrix
		Eigen::Matrix3f getDampingMatrix();

		// Return energy tank value
		float getEnergyTank();

		// Return alpha value
		float getAlpha();

		// Return beta value
		float getBeta();
	private:
		// Update damping matrix
		void updateDampingMatrix(Eigen::Vector3f vd);

		// Update energy tank state
		Eigen::Vector3f updateEnergyTank(Eigen::Vector3f v, Eigen::Vector3f Fc);
		
		// Orthonormalize basis
		void orthonormalizeBasis();

		// Smooth rise function between 0 and 1
		float smoothRise(float x, float a, float b);

		// Smooth fall function between 1 and 0
		float smoothFall(float x, float a, float b);

		// Smooth rise fall function between 0 and 1
		float smoothRiseFall(float x, float a, float b, float c, float d);

		// Dynamic reconfigure callback
		// void dynamicReconfigureCallback(motion_force_control::passiveDsForceController_paramsConfig &config, uint32_t level);

};


#endif
