#include "PassiveDsForceController.h"


PassiveDsForceController::PassiveDsForceController(float dt): _dt(dt)
{
  _D.setIdentity();
  _B.setRandom();
  orthonormalizeBasis();
  _L.setIdentity();
  _lambda1 = 0.0f;
  _lambda2 = 0.0f;
  _smax = 0.2f;
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

  // std::cerr << _D << std::endl;

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

  updateEnergyTank(v,Fc);

  _F = _D*(vd-v)+_betaF*Fc;

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
  return _betaF;
}


void PassiveDsForceController::updateDampingMatrix(Eigen::Vector3f vd)
{
  if(vd.norm()>MIN_VELOCITY_NORM)
  {
    _B.col(0) = vd.normalized();
    orthonormalizeBasis();
    // std::cerr << _B << std::endl;
    // std::cerr << _L << std::endl;
    _D = _B*_L*_B.transpose();
  }
}


Eigen::Vector3f PassiveDsForceController::updateEnergyTank(Eigen::Vector3f v, Eigen::Vector3f Fc)
{
  float z = v.transpose()*Fc;

  if(fabs(z)<0.01f)
  {
    z = 0.0f;
  }
  // _alpha = smoothRiseFall(s,0.0f,_ds,_smax-_ds,_smax);
  _alpha = smoothFall(_s,_smax-_ds,_smax);


  if(z<0.0f && _s>_smax)
  {
    _beta = 0.0f;
  }
  else if (z>0.0f && _s < 0.0f)
  {
    _beta = 0.0f;
  }
  else
  {
    _beta = 1.0f;
  }
  // _beta = 1.0f-smoothRise(_s,_smax-_ds,_smax)*smoothFall(z,0.0f,_dz)-smoothFall(_s,0.0f,_ds)*smoothRise(z,-_dz,0.0f);


  if(z<0.0f)
  {
    _betaF = 1.0f;
  }
  else
  {
    _betaF = _beta;
  }

  _s += (_alpha*v.transpose()*_D*v-_beta*z)*1.0f*_dt;



  // if(_s>_smax)
  // {
  //   _s = _smax;
  // }
  // else if(_s<0.0f)
  // {
  //   _s = 0.0f;
  // }
}


void PassiveDsForceController::orthonormalizeBasis()
{
  // Use Gram-Schmidt process to orthonormalize a matrix
  Eigen::Vector3f u1,u2,u3;
  u1 = _B.col(0);
  // std::cerr << _B << std::endl;
  // std::cerr << u1 << std::endl;
  u2 = _B.col(1)-(u1.dot(_B.col(1))/(u1.squaredNorm()))*u1;
  // std::cerr << u2 << std::endl;
  u3 = _B.col(2)-(u1.dot(_B.col(2))/(u1.squaredNorm()))*u1-(u2.dot(_B.col(2))/(u2.squaredNorm()))*u2;

  // std::cerr << u3 << std::endl;
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

