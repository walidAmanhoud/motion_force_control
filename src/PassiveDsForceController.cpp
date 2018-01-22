#include "PassiveDsForceController.h"


PassiveDsForceController::PassiveDsForceController()
{
  _D.setIdentity();
  _B.setIdentity();
  _L.setIdentity();
  _lambda1 = 0.0f;
  _lambda2 = 0.0f;
  _smax = 10.0f;
  _s = _smax;
  _ds = 0.1f*_smax;
  _dz = 0.01;

  //   // Dynamic reconfigure definition
  // _dynRecCallback = boost::bind(&PassiveDsForceController::dynamicReconfigureCallback, this, _1, _2);
  // _dynRecServer.setCallback(_dynRecCallback);
}


Eigen::Vector3f PassiveDsForceController::step(Eigen::Vector3f vd,Eigen::Vector3f v)
{
  updateDampingMatrix(vd);

  _F = _D*(vd-v);

  // Eigen::Vector3f error;
  // error = v-vd;

  // Eigen::Vector3f orthogonalError;
  // orthogonalError = error;


  // Eigen::Vector3f parallelError;
  // parallelError.setConstant(0.0f);


  // if(vd.norm()> MIN_VELOCITY_NORM)
  // {
  //     parallelError = (error.dot(vd)/pow(vd.norm(),2.0f))*vd;
  // }

  // orthogonalError -= parallelError;

  // _F = -_lambda1*parallelError-_lambda2*orthogonalError;

  return _F;
}


Eigen::Vector3f PassiveDsForceController::step(Eigen::Vector3f vd, Eigen::Vector3f v, Eigen::Vector3f Fc)
{
  updateDampingMatrix(vd);

  updateEnergyTank(Fc,v);

  _F = _D*(vd-v)+_beta*Fc;

  return _F;
}


Eigen::Matrix3f PassiveDsForceController::getDampingMatrix()
{
  return _D;
}


float PassiveDsForceController::getEnergyTank()
{
  return _s;
}


float PassiveDsForceController::getAlpha()
{
  return _alpha;
}


float PassiveDsForceController::getBeta()
{
  return _beta;
}


void PassiveDsForceController::updateDampingMatrix(Eigen::Vector3f vd)
{
  if(vd.norm()>MIN_VELOCITY_NORM)
  {
    _B.col(0) = vd.normalized();
    orthonormalizeBasis();
    _D = _B*_L*_B.transpose();
  }
}


Eigen::Vector3f PassiveDsForceController::updateEnergyTank(Eigen::Vector3f v, Eigen::Vector3f Fc)
{
  float z = v.transpose()*Fc;
  // _alpha = smoothRiseFall(s,0.0f,_ds,_smax-_ds,_smax);
  _alpha = smoothFall(_s,_smax-_ds,_smax);
  _beta = 1.0f-smoothRise(_s,_smax-_ds,_smax)*smoothFall(z,0.0f,_dz)-smoothFall(_s,0.0f,_ds)*smoothRise(z,-_dz,0.0f);

  _s += _alpha*v.transpose()*_D*v-_beta*z;
}


void PassiveDsForceController::orthonormalizeBasis()
{
  // Use Gram-Schmidt process to orthonormalize a matrix
  Eigen::Vector3f u1,u2,u3;
  u1 = _B.col(0);
  u2 = _B.col(1)-(u1.dot(_B.col(1))/(u1.squaredNorm()))*u1;
  u3 = _B.col(2)-(u1.dot(_B.col(2))/(u1.squaredNorm()))*u1-(u2.dot(_B.col(2))/(u2.squaredNorm()))*u2;

  _B.col(0) = u1.normalized();
  _B.col(1) = u2.normalized();
  _B.col(2) = u3.normalized();
}


void PassiveDsForceController::updateDampingGains(float lambda1, float lambda2)
{
  _lambda1 = lambda1;
  _lambda2 = lambda2;
  _L(0,0) = _lambda1;
  _L(1,1) = _lambda2;
  _L(2,2) = _lambda2;
}


float PassiveDsForceController::smoothRise(float x, float a, float b)
{
  float y; 
  if(x<a)
  {
    y = 0.0f;
  }
  else if(x>b)
  {
    y = 1.0f;
  }
  else
  {
    y = (1.0f+sin(M_PI*(x-a)/(b-a)-M_PI/2.0f))/2.0f;
  }

  return y;
}

float PassiveDsForceController::smoothFall(float x, float a, float b)
{
  return 1.0f-smoothRise(x,a,b);
}

float PassiveDsForceController::smoothRiseFall(float x, float a, float b, float c, float d)
{
  return smoothRise(x,a,b)*smoothFall(x,c,d);
}


// void PassiveDsForceController::dynamicReconfigureCallback(motion_force_control::passiveDsForceController_paramsConfig &config, uint32_t level)
// {
//   ROS_INFO("Reconfigure request. Updatig the parameters ...");

//   _lambda1 = config.lambda1;
//   _lambda2 = config.lambda2;

//   // Update the damping values 
//   _L(0,0) = _lambda1;
//   _L(1,1) = _lambda2;
//   _L(2,2) = _lambda2;

// }

