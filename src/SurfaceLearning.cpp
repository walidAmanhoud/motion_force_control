#include "SurfaceLearning.h"

SurfaceLearning* SurfaceLearning::me = NULL;

SurfaceLearning::SurfaceLearning(ros::NodeHandle &n, double frequency, std::string fileName, Mode mode, float C, float sigma, float epsilonTube, bool processRawData, bool useFullData):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName),
  _mode(mode),
  _C(C),
  _sigma(sigma),
  _epsilonTube(epsilonTube),
  _processRawData(processRawData),
  _useFullData(useFullData)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.035f;
  _toolOffset = 0.14f;
  _loadMass = 0.1f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);
  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _wrenchBias.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);

  _xd.setConstant(0.0f);
  _vd.setConstant(0.0f);
  _vdOrig.setConstant(0.0f);
  _vdR.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);
  _e1 << 0.0f, 0.0f, 1.0f;
  _xAttractor << -0.55f,0.05f,0.2f;

  _Fd = 0.0f;



  _firstRobotPose = false;
  _firstRobotTwist = false;
  _firstWrenchReceived = false;

  _Fc.setConstant(0.0f);

  _wrenchBiasOK = false;
  _stop = false;

  _sequenceID = 0;
  _normalDistance = 0.0f;


  _msgArrowMarker.header.frame_id = "world";
  _msgArrowMarker.header.stamp = ros::Time();
  _msgArrowMarker.ns = "marker_test_arrow";
  _msgArrowMarker.id = 0;
  _msgArrowMarker.type = visualization_msgs::Marker::ARROW;
  _msgArrowMarker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p1, p2;
  p1.x = 0.0f;
  p1.y = 0.0f;
  p1.z = 0.0f;
  p2.x = 0.0f+0.3f*_e1(0);
  p2.x = 0.0f+0.3f*_e1(1);
  p2.x = 0.0f+0.3f*_e1(2);
  _msgArrowMarker.scale.x = 0.05;
  _msgArrowMarker.scale.y = 0.1;
  _msgArrowMarker.scale.z = 0.1;
  _msgArrowMarker.color.a = 1.0;
  _msgArrowMarker.color.r = 1.0;
  _msgArrowMarker.color.g = 0.0;
  _msgArrowMarker.color.b = 0.0;
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);

}


bool SurfaceLearning::init() 
{
  // Subscriber definitions
  _subRobotPose = _n.subscribe("/lwr/ee_pose", 1, &SurfaceLearning::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist = _n.subscribe("/lwr/ee_vel", 1, &SurfaceLearning::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &SurfaceLearning::updateRobotWrench, this, ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("SurfaceLearning/filteredWrench", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("SurfaceLearning/markers", 10);

  signal(SIGINT,SurfaceLearning::stopNode);

  // if(!_n.getParamCached("/lwr/ds_param/damping_eigval0",_lambda1))
  // {
  //   ROS_ERROR("Cannot read first eigen value of passive ds controller");
  //   return false;
  // }

  ROS_INFO("Filename: %s", _fileName.c_str());

  if(_mode == COLLECTING_DATA)
  {
    ROS_INFO("Mode: COLLECTING_DATA");
  }
  else if(_mode == LEARNING)
  {
    ROS_INFO("Mode: LEARNING");
    ROS_INFO("C: %f", _C);
    ROS_INFO("Sigma: %f", _sigma);
    ROS_INFO("Epsilon tube: %f", _epsilonTube);
    ROS_INFO("Process raw data: %d", _processRawData);
    ROS_INFO("Use full data: %d", _useFullData);
  }
  else if(_mode == TESTING)
  {
    ROS_INFO("Mode: TESTING");
  }
  else
  {
    ROS_ERROR("Mode not recognized");
    return false;
  }


  if(_mode == COLLECTING_DATA)
  {
    _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data/"+_fileName+"_raw_data.txt");
    if(!_outputFile.is_open())
    {
      ROS_ERROR("Cannot open data file");
      return false;
    } 
  }
  else if(_mode == LEARNING)
  {
    _inputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_libsvm_data.txt");
    if(!_inputFile.is_open())
    {
      ROS_ERROR("Cannot open libsvm data file");
      return false;
    }
    else
    {
      _inputFile.close();
    }  
  }
  else if(_mode == TESTING)
  {

    std::string modelPath = ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_svmgrad_model.txt";
    _inputFile.open(modelPath);
    if(!_inputFile.is_open())
    {
      ROS_ERROR("Cannot svmgrad model model file");
      return false;
    }
    else
    {
      _inputFile.close();
      _svm.loadModel(modelPath);
    }    
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The modulated ds node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void SurfaceLearning::run()
{
  while (!_stop) 
  {
    if(_firstRobotPose && _firstRobotTwist /*&& _wrenchBiasOK*/)
    {
      _mutex.lock();

      switch(_mode)
      {
        case COLLECTING_DATA:
        {
          computeCommand();
          
          // Log data
          logData();        
          break;
        }
        case TESTING:
        {
          computeCommand();
          break;
        }
        default:
        {
          break;
        }
      }

     // Publish data to topics
      publishData();

      _mutex.unlock();
    }

    if(_mode == LEARNING)
    {
      me->_stop = true;
    }
    
    ros::spinOnce();
    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;
  _Fc.setConstant(0.0f);

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  if(_mode == COLLECTING_DATA)
  {
    _outputFile.close();
  }
  
  if(_mode == LEARNING)
  {
    if(_processRawData)
    {
      processRawData();
    }

    learnSurfaceModel();
  }

  ros::shutdown();
}


void SurfaceLearning::stopNode(int sig)
{
  me->_stop = true;
}

void SurfaceLearning::computeCommand()
{

  if(_mode == TESTING)
  {
    _svm.preComputeKernel(true);
    _normalDistance = _svm.calculateGamma(_x.cast<double>());
    _e1 = -_svm.calculateGammaDerivative(_x.cast<double>()).cast<float>();
    _e1.normalize();
    std::cerr << _normalDistance << " " << _e1.transpose() << std::endl;    
    if(_normalDistance<0.0f)
    {
      _normalDistance = 0.0f;
    }

    // Check for update of passive ds controller eigen value
    // ros::param::getCached("/lwr/ds_param/damping_eigval0",_lambda1);

    // computeOriginalDynamics();
    // rotatingDynamics();
    // forceModulation();
    // _vd = _vdR;
  }

  computeDesiredOrientation();
}



void SurfaceLearning::computeDesiredOrientation()
{
  if(_mode == TESTING)
  {
    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f k;
    k = (-_wRb.col(2)).cross(-_e1);
    float c = (-_wRb.col(2)).transpose()*(-_e1);  
    float s = k.norm();
    k /= s;
    
    Eigen::Matrix3f K;
    K << getSkewSymmetricMatrix(k);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
    quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = quaternionProduct(qtemp,_q);

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
    _qd = slerpQuaternion(_q,qf,1.0f-std::tanh(5.0f*_normalDistance));
    // _qd = slerpQuaternion(_q,qf,1.0f);

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q(0);
    qcurI.segment(1,3) = -_q.segment(1,3);
    wq = 5.0f*quaternionProduct(qcurI,_qd-_q);
    Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
    _omegad = omegaTemp;    
  }
  else if(_mode == COLLECTING_DATA)
  {
    _qd = _q;
    _omegad.setConstant(0.0f); 
  }

}

void SurfaceLearning::computeOriginalDynamics()
{
  Eigen::Vector3f dir;
  dir = _e1;
  dir.normalize();
  _vdOrig = 0.25f*dir;
}


Eigen::Vector3f SurfaceLearning::getCyclingMotionVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
{
  Eigen::Vector3f velocity;

  position = position-attractor;

  velocity(2) = -position(2);

  float R = sqrt(position(0) * position(0) + position(1) * position(1));
  float T = atan2(position(1), position(0));

  float r = 0.05f;
  float omega = M_PI;

  velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
  velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

  return velocity;
}


void SurfaceLearning::rotatingDynamics()
{
    Eigen::Vector3f vdContact;
    vdContact = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*getCyclingMotionVelocity(_x,_xAttractor);
    vdContact.normalize();

    // Compute rotation angle
    float angle = std::acos(_vdOrig.normalized().dot(vdContact));
    float theta = (1.0f-std::tanh(10*_normalDistance))*angle;

    // Compute rotation direction
    Eigen::Vector3f u = (_vdOrig.normalized()).cross(vdContact);
    Eigen::Matrix3f K,R;

    if(u.norm() < FLT_EPSILON)
    {
      R.setIdentity();
    }
    else
    {
      u/=u.norm();
      K = getSkewSymmetricMatrix(u);
      R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-std::cos(theta))*K*K;
    }
    _vdR = R*_vdOrig;
}

void SurfaceLearning::forceModulation()
{
  // Extract linear speed, force and torque data

  // Compute modulation matrix used to apply a force Fd when the surface is reached while keeping the norm of the velocity constant 
  // M(x) = B(x)L(x)B(x)
  // B(x) = [e1 e2 e3] with e1 = -n is an orthognal basis defining the modulation frame
  //        [la lb lb] 
  // L(x) = [0  lc 0 ] is the matrix defining the modulation gains to apply on the frame
  //        [0  0  lc]

  // Compute modulation frame B(x)
  Eigen::Vector3f xDir;
  xDir << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  _e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*xDir;
  _e2.normalize();
  _e3 = _e1.cross(_e2);
  _e3.normalize();
  B.col(0) = _e1;
  B.col(1) = _e2;
  B.col(2) = _e3;

  // Compute force profile
  if(_lambda1<1.0f)
  {
    _lambda1 = 1.0f;
  }

  _Fd = 10.0f*(1.0f-std::tanh(100.0f*_normalDistance))/_lambda1;

  // Compute diagonal gain matrix L(x)
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  float la, lb, lc;

  float temp, delta;

  temp = (_e1+_e2+_e3).dot(_vdR);
  if(fabs(temp)<FLT_EPSILON)
  {
    lb = 0.0f;
  }
  else
  {
    lb = _Fd/temp;
  }

  // if(_constraint == VELOCITY_NORM)
  // {
    // delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)-4.0f*std::pow(_vdOrig.norm(),2.0f)*(std::pow(lb*temp,2.0f)-std::pow(_vdOrig.norm(),2.0f));
  // }
  // else if(_constraint == APPARENT_VELOCITY_NORM)
  // {
    delta = std::pow(2.0f*_e1.dot(_vdR)*lb*temp,2.0f)+4.0f*std::pow(_vdOrig.norm(),4.0f); 
  // }

  if(delta < 0.0f)
  {
    delta = 0.0f;
  }
  la = (-2.0f*_e1.dot(_vdR)*lb*temp+sqrt(delta))/(2.0f*std::pow(_vdOrig.norm(),2.0f));

  L(0,0) = la+lb;
  L(0,1) = lb;
  L(0,2) = lb;
  L(1,1) = la;
  L(2,2) = la;

  // Compute modulation matrix
  Eigen::Matrix3f M;
  M = B*L*B.transpose();

  // Apply force modulation to the rotating dynamics

  _vd = M*_vdR;

  std::cerr << "delta: " << delta << " la: " << la << " lb: " << lb << " vd: " << _vd.norm() << std::endl;


  // Bound desired velocity  
  if(_vd.norm()>0.4f)
  {
    _vd *= 0.4f/_vd.norm();
  }

  _Fc.setConstant(0.0f);

  std::cerr << "vd after scaling: " << _vd.norm() << " distance: " << _normalDistance << std::endl;
}


void SurfaceLearning::learnSurfaceModel()
{
  ROS_INFO("Learning surface model ...");
  std::string command;
  std::string dataFile = ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_data.txt";
  std::string modelFile = ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_libsvm_model.txt";
  float gamma = 1.0f/(2.0f*std::pow(_sigma,2.0f));
  std::string svmCommand = "svm-train -s 3 -t 2 -c "+std::to_string(_C)+" -g "+std::to_string(gamma)+" -h 0 -p "+std::to_string(_epsilonTube);
  command = svmCommand+" "+dataFile+" "+modelFile;
  
  int val = std::system(command.c_str());
  ROS_INFO("libsvm command result: %d", val);

  generateSVMGradModelFile();
}

void SurfaceLearning::generateSVMGradModelFile()
{
  ROS_INFO("Generate svm grad model file ...");
  std::string command;
  _inputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_libsvm_model.txt");
  if(!_inputFile.is_open())
  {
    ROS_ERROR("Cannot open libsvm model file");
  }

  std::string modelType, kernelType;
  float gamma, rho;
  int nbClasses, nbSVs;

  std::string line;
  std::getline(_inputFile,line);
  modelType = line.substr(1,' ');
  std::getline(_inputFile,line);
  kernelType = line.substr(1,' ');
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"gamma %f", &gamma);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"nr_class %d", &nbClasses);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"total_sv %d", &nbSVs);
  std::getline(_inputFile,line);
  sscanf(line.c_str(),"rho %f",&rho);
  std::getline(_inputFile,line);


  Eigen::Matrix<float,Eigen::Dynamic,3> sv;
  Eigen::VectorXf alpha;

  if(line =="SV")
  {
    sv.resize(nbSVs,3);
    alpha.resize(nbSVs);
    
    for(int k = 0; k < nbSVs; k++)
    {
      std::getline(_inputFile,line);
      sscanf(line.c_str(),"%f 1:%f 2:%f 3:%f", &alpha(k),&sv(k,0),&sv(k,1),&sv(k,2));
    }
  }
  ROS_INFO("svm_type %s", modelType.c_str());
  ROS_INFO("kernel_type %s", kernelType.c_str());
  ROS_INFO("gamma %f", gamma);
  ROS_INFO("nr_class %d", nbClasses);
  ROS_INFO("total_sv %d", nbSVs);
  ROS_INFO("rho %f",rho);


  _inputFile.close();


  _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_svmgrad_model.txt");
  if(!_outputFile.is_open())
  {
    ROS_ERROR("Cannot open svnm grad model file");
  }
  else
  {
    _outputFile << sv.cols() << std::endl
                << nbSVs << std::endl
                << -rho << std::endl
                << 1.0f/std::sqrt(2*gamma) << std::endl << std::endl
                << alpha.transpose() << std::endl << std::endl
                << sv.col(0).transpose() << std::endl
                << sv.col(1).transpose() << std::endl
                << sv.col(2).transpose() << std::endl;

    _outputFile.close();
  }


}


void SurfaceLearning::processRawData()
{
    _inputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_raw_data.txt");
    
    if(!_inputFile.is_open())
    {
      ROS_ERROR("Cannot open raw data file");
    }

    std::string line;
    double timeTemp;
    Eigen::Vector3f position,velocity,force;
    uint32_t sequenceID;

    std::vector<Eigen::Vector3f> surfaceData;
    while(std::getline(_inputFile,line))  
    {
      std::stringstream ss;
      ss << line;
      ss >> timeTemp >> position(0) >> position(1) >> position(2) >> 
                        velocity(0) >> velocity(1) >> velocity(2) >>
                        force(0) >> force(1) >> force(2) >> sequenceID;

      if(force.norm()>3.0f)
      {
        surfaceData.push_back(position);
      }
    }

    std::cerr << surfaceData.size() << std::endl;

    Eigen::Matrix<float,Eigen::Dynamic,3, Eigen::RowMajor> Xs;
    Xs.resize(surfaceData.size(),3);
    memcpy(Xs.data(),surfaceData.data(),surfaceData.size()*sizeof(Eigen::Vector3f));

    Eigen::Vector3f XsMin, XsMax;

    for(int k = 0; k< 3; k++)
    {
      XsMin(k) = Xs.col(k).array().minCoeff();
      XsMax(k) = Xs.col(k).array().maxCoeff();
    }

    srand(time(NULL));
    Eigen::Matrix<float,Eigen::Dynamic,3> R;
    R.resize(20000,3);
    R.setRandom();
    R.array()+= 1.0f;
    R.array()/=2.0f;

    Eigen::MatrixXf X;
    X.resize(R.rows(),R.cols());

    float offset = 0.3f;

    for(int k = 0; k< 3; k++)
    {
      if(k == 2)
      {
        X.col(k).array() = XsMin(k)+(XsMax(k)+offset-XsMin(k))*R.col(k).array();
      }
      else
      {
        X.col(k).array() = XsMin(k)+(XsMax(k)-XsMin(k))*R.col(k).array();
      }
    }

    Eigen::Matrix<float,Eigen::Dynamic,1> temp, XLabel, XLabelIndex;
    XLabel.resize(X.rows(),1);
    XLabelIndex.resize(X.rows());

    Eigen::VectorXf::Index index;
    for(uint32_t k = 0; k < X.rows(); k++)
    {
      XLabel(k) = (X.row(k).replicate(Xs.rows(),1)-Xs).rowwise().norm().array().minCoeff(&index);
      XLabelIndex(k) = index; 
      if(X(k,2)-Xs(index,2)<0.0f)
      {
        XLabel(k) = -XLabel(k);
      }
    }

    _inputFile.close();

    Eigen::MatrixXf XFull;
    XFull.resize(X.rows()+Xs.rows(),X.cols());
    XFull << X,Xs;

    Eigen::VectorXf XFullLabel;
    XFullLabel.resize(X.rows()+Xs.rows());
    XFullLabel.setConstant(0.0f);
    XFullLabel.segment(0,X.rows()) = XLabel;

    _outputFile.open(ros::package::getPath(std::string("motion_force_control"))+"/data_surface/"+_fileName+"_libsvm_data.txt");


    if(!_outputFile.is_open())
    {
      ROS_ERROR("Cannot open libsvm data file");
    }

    if(_useFullData)
    {
      for(uint32_t k = 0; k < XFull.rows(); k++)
      {
        _outputFile << XFullLabel(k) << " 1:" << XFull(k,0) << " 2:" << XFull(k,1) << " 3:" << XFull(k,2) << std::endl;
      }
    }
    else
    {
      for(uint32_t k = 0; k < X.rows(); k++)
      {
        _outputFile << XLabel(k) << " 1:" << X(k,0) << " 2:" << X(k,1) << " 3:" << X(k,2) << std::endl;
      }
    }


    _outputFile.close();

}

void SurfaceLearning::logData()
{
    _outputFile << ros::Time::now() << " "
                << _x.transpose() << " "
                << _v.transpose() << " "
                << _filteredWrench.segment(0,3).transpose() << " "
                << _sequenceID << std::endl;
}

void SurfaceLearning::publishData()
{
  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);
  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);
  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);

  _msgDesiredWrench.force.x = _Fc(0);
  _msgDesiredWrench.force.y = _Fc(1);
  _msgDesiredWrench.force.z = _Fc(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);

  _msgArrowMarker.points.clear();
  geometry_msgs::Point p1, p2;
  p1.x = _x(0);
  p1.y = _x(1);
  p1.z = _x(2);
  p2.x = _x(0)+0.3f*_e1(0);
  p2.y = _x(1)+0.3f*_e1(1);
  p2.z = _x(2)+0.3f*_e1(2);
  _msgArrowMarker.points.push_back(p1);
  _msgArrowMarker.points.push_back(p2);
  _pubMarker.publish(_msgArrowMarker);
}


void SurfaceLearning::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  Eigen::Vector3f temp = _x;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if((_x-temp).norm()>FLT_EPSILON)
  {
    _sequenceID++;
    if(_mode == COLLECTING_DATA)
    {
      std::cerr << _sequenceID << std::endl;
    }
  }

  if(!_firstRobotPose)
  {
    _firstRobotPose = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void SurfaceLearning::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _v(0) = msg->linear.x;
  _v(1) = msg->linear.y;
  _v(2) = msg->linear.z;
  _w(0) = msg->angular.x;
  _w(1) = msg->angular.y;
  _w(2) = msg->angular.z;

  if(!_firstRobotTwist)
  {
    _firstRobotTwist = true;
  }
}
 
void SurfaceLearning::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRobotPose)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
    }
  }

  if(_wrenchBiasOK && _firstRobotPose)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
  }

}

Eigen::Vector4f SurfaceLearning::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f SurfaceLearning::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


Eigen::Vector4f SurfaceLearning::rotationMatrixToQuaternion(Eigen::Matrix3f R)
{
  Eigen::Vector4f q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);


  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


Eigen::Matrix3f SurfaceLearning::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void SurfaceLearning::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));
}


Eigen::Vector4f SurfaceLearning::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
{

  Eigen::Vector4f q;

  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
  if(q1.dot(q2)<0.0f)
  {   
    q2 = -q2;
  }

  float dotProduct = q1.dot(q2);
  if(dotProduct > 1.0f)
  {
    dotProduct = 1.0f;
  }
  else if(dotProduct < -1.0f)
  {
    dotProduct = -1.0f;
  }

  float omega = acos(dotProduct);

  if(std::fabs(omega)<FLT_EPSILON)
  {
    q = q1.transpose()+t*(q2-q1).transpose();
  }
  else
  {
    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
  }

  return q;
}

