#include <iostream>
#include <cmath>
#include <gsl/gsl_randist.h>

#include "../include/smctc.h"
#include "IMU-GPS-Fusion.h"

using namespace std;

  cv_state::cv_state() {

	stateSpace.set_size(15);
 	stateSpace.fill(0.0); 
    
  }

  GPS_obs::GPS_obs(double time) {
	CovarianceMatrixR.set_size(6, 6);
	measurementGPS.set_size(6);
	sensorH.set_size(6, 15);
   CovarianceMatrixR.fill(0.0);
   measurementGPS.fill(0.0);
    sensorH.fill(0.0);
    sensorH.at(0,6) = 1;
    sensorH.at(1,7) = 1;
    sensorH.at(2,8) = 1;
    sensorH.at(3,9) = 1;
    sensorH.at(4,10) = 1;
    sensorH.at(5,11) = 1;
    lastTime = 0;
    currentTime = time;
    deltaT = 0;
  }

  void GPS_obs::SetMeasurement (arma::vec Pos, arma::vec Velocity, double Xerror, double Yerror, double Zerror, double Verror, double Cerror, double currentT) {
    CovarianceMatrixR.at(0,0) = Xerror*Xerror;
    CovarianceMatrixR.at(1,1) = Yerror*Yerror;
    CovarianceMatrixR.at(2,2) = Zerror*Zerror;
    CovarianceMatrixR.at(3,3) = Verror*Verror;
    CovarianceMatrixR.at(4,4) = Verror*Verror;
    CovarianceMatrixR.at(5,5) = Cerror*Cerror;
    measurementGPS.at(0) = Pos.at(0);
 	measurementGPS.at(1) = Pos.at(1);
 	measurementGPS.at(2) = Pos.at(2);
 	measurementGPS.at(3) = Velocity.at(0);
 	measurementGPS.at(4) = Velocity.at(1);
 	measurementGPS.at(5) = Velocity.at(2);
	lastTime = currentTime;
    currentTime = currentT;
    deltaT = currentTime-lastTime;
  std::cout << deltaT << std::endl;
    
      
      
  }


   IMU_obs::IMU_obs() {

CovarianceMatrixR.set_size(9, 9);
measurementIMU.set_size(9);
sensorH.set_size(9, 15);
     
    measurementIMU.fill(0.0);
    CovarianceMatrixR.fill(0.0); 
    sensorH.fill(0.0);
    sensorH.at(0,0) = 1;
    sensorH.at(1,1) = 1;
    sensorH.at(2,2) = 1;
    sensorH.at(3,0) = 1;
    sensorH.at(4,1) = 1;
    sensorH.at(5,2) = 1;
    sensorH.at(12,0) = 1;
    sensorH.at(13,1) = 1;
    sensorH.at(14,2) = 1;
    lastTime = 0;
    currentTime = 0;
    deltaT = 0;
     
   }
   IMU_obs::IMU_obs(double poseError, double gyroError, double accError, double time) {

	CovarianceMatrixR.set_size(9, 9);
	measurementIMU.set_size(9);
	sensorH.set_size(9, 15);
    measurementIMU.fill(0.0);
    CovarianceMatrixR.fill(0.0); 
    sensorH.fill(0.0);
    sensorH.at(0,0) = 1;  //first one is y, second is x
    sensorH.at(1,1) = 1;
    sensorH.at(2,2) = 1;
    sensorH.at(3,3) = 1;
    sensorH.at(4,4) = 1;
    sensorH.at(5,5) = 1;
    sensorH.at(6,12) = 1;
    sensorH.at(7,13) = 1;
    sensorH.at(8,14) = 1;
    CovarianceMatrixR.at(0,0) = poseError*poseError;
    CovarianceMatrixR.at(1,1) = poseError*poseError;
    CovarianceMatrixR.at(2,2) = poseError*poseError;
    CovarianceMatrixR.at(3,3) = gyroError*gyroError;
    CovarianceMatrixR.at(4,4) = gyroError*gyroError;
    CovarianceMatrixR.at(5,5) = gyroError*gyroError;
    CovarianceMatrixR.at(6,6) = accError*accError;
    CovarianceMatrixR.at(7,7) = accError*accError;
    CovarianceMatrixR.at(8,8) = accError*accError;
    lastTime = 0;
    currentTime = time;
    deltaT = 0;
     
   }

   void IMU_obs::SetMeasurement (arma::vec Theta,  arma::vec Omega, arma::vec Acc, double currentT) {
     
         
    measurementIMU.at(0) = Theta.at(0);
	measurementIMU.at(1) = Theta.at(1);
	measurementIMU.at(2) = Theta.at(2);
	measurementIMU.at(3) = Omega.at(0);
	measurementIMU.at(4) = Omega.at(1);
	measurementIMU.at(5) = Omega.at(2);
	measurementIMU.at(6) = Acc.at(0);
	measurementIMU.at(7) = Acc.at(1);
	measurementIMU.at(8) = Acc.at(2);
    lastTime = currentTime;
    currentTime = currentT;
//std::cout << currentT << std::endl;
    deltaT = currentTime-lastTime;
   std::cout << deltaT<< std::endl;


   }
  


/**
 * @brief 计算IMU测量的对数似然函数
 * @details 计算当前粒子状态对IMU观测的对数似然值,用于粒子权重更新
 * 
 * @param X 当前粒子状态
 * @param y_imui IMU观测值对象
 * @return double 对数似然值
 *
 * 公式: log(L) = -1/2 * (z - Hx)' * R^(-1) * (z - Hx)
 * 其中:
 * z: IMU测量值(9维向量,包含姿态、角速度、加速度)
 * H: 观测矩阵(9x15)
 * x: 状态向量(15维)
 * R: 测量噪声协方差矩阵(9x9)
 */
double logLikelihoodIMU(const cv_state & X, const IMU_obs & y_imui)
{
  // 计算测量残差: 实际测量值 - 预测测量值
  arma::mat temp = y_imui.measurementIMU-(y_imui.sensorH*X.stateSpace);

  // 计算测量协方差矩阵的逆
  arma::mat covi = arma::inv(y_imui.CovarianceMatrixR);

  // 计算马氏距离: (z-Hx)'*R^(-1)*(z-Hx)
  arma::mat weight = arma::trans(temp)*arma::trans(covi)*(temp);

  // 返回对数似然值
  // 注:负号是因为要将代价函数转化为似然函数
  return ((-0.5*weight(0,0)));
}

double logLikelihoodGPS(const cv_state & X, const GPS_obs & y_gpsi)
{

//std::cout << ((X.stateSpace)) << std::endl;
  arma::mat temp = y_gpsi.measurementGPS-y_gpsi.sensorH*X.stateSpace;
  
  arma::mat covi = inv(y_gpsi.CovarianceMatrixR);

//std::cout << ( y_gpsi.measurementGPS) << std::endl;
//std::cout << ( temp) << std::endl;
//std::cout << (covi) << std::endl;


  arma::mat weight = arma::trans(temp)*arma::trans(covi)*(temp);

 // std::cout <<"GPS Particle Weight: " << (-0.5*weight(0,0)) << std::endl;

//std::cout << (weight(0,0)*weight(0,1)*weight(0,2)) << std::endl;
  
  return ((-0.5*weight(0,0)));
}


///A function to initialise particles

/// \param pRng A pointer to the random number generator which is to be used
/**
 * @brief 初始化粒子滤波器的状态粒子
 * @details 该函数初始化一个15维的状态向量粒子，包含:
 *          - 姿态(0-2): x,y,z欧拉角,从IMU测量初始化
 *          - 角速度(3-5): x,y,z角速度,从IMU测量初始化
 *          - 位置(6-8): x,y,z位置,初始化为带噪声的0
 *          - 速度(9-11): x,y,z速度,初始化为带噪声的0
 *          - 加速度(12-14): x,y,z加速度,初始化为带噪声的0
 * 
 * @param pRng 随机数生成器指针,用于生成高斯噪声
 * @return smc::particle<cv_state> 返回初始化的粒子,权重设为1/N
 *
 * @note - 状态噪声标准差统一设为sqrt(0.1)
 *       - N为粒子数量,在其他地方定义
 *       - y_imu为IMU测量数据指针,在其他地方定义
 *       - cv_state为自定义的状态类型
 */
smc::particle<cv_state> fInitialise(smc::rng *pRng)
{
  cv_state k;
  
  // 从IMU测量初始化姿态角,加入高斯噪声
  k.stateSpace(0) = pRng->Normal(y_imu->measurementIMU.at(0),sqrt(.1)); //x姿态角
  k.stateSpace(1) = pRng->Normal(y_imu->measurementIMU.at(1),sqrt(.1)); //y姿态角  
  k.stateSpace(2) = pRng->Normal(y_imu->measurementIMU.at(2),sqrt(.1)); //z姿态角

  // 从IMU测量初始化角速度,加入高斯噪声
  k.stateSpace(3) = pRng->Normal(y_imu->measurementIMU.at(3),sqrt(.1)); //x角速度
  k.stateSpace(4) = pRng->Normal(y_imu->measurementIMU.at(4),sqrt(.1)); //y角速度
  k.stateSpace(5) = pRng->Normal(y_imu->measurementIMU.at(5),sqrt(.1)); //z角速度

  // 初始化位置为带噪声的0
  k.stateSpace(6) = pRng->Normal(0,sqrt(.1)); //x位置
  k.stateSpace(7) = pRng->Normal(0,sqrt(.1)); //y位置
  k.stateSpace(8) = pRng->Normal(0,sqrt(.1)); //z位置

  // 初始化速度为带噪声的0
  k.stateSpace(9) = pRng->Normal(0,sqrt(.1));  //x速度
  k.stateSpace(10) = pRng->Normal(0,sqrt(.1)); //y速度
  k.stateSpace(11) = pRng->Normal(0,sqrt(.1)); //z速度

  // 初始化加速度为带噪声的0
  k.stateSpace(12) = pRng->Normal(0,sqrt(.1)); //x加速度
  k.stateSpace(13) = pRng->Normal(0,sqrt(.1)); //y加速度
  k.stateSpace(14) = pRng->Normal(0,sqrt(.1)); //z加速度

  // 返回初始化的粒子,权重设为1/N
  return smc::particle<cv_state>(k, (1/N));
}
smc::particle<cv_state> fInitialise(smc::rng *pRng)
{
  cv_state k;
  
  k.stateSpace(0) = pRng->Normal(y_imu->measurementIMU.at(0),sqrt(.1)); //x pose
  k.stateSpace(1) = pRng->Normal(y_imu->measurementIMU.at(1),sqrt(.1));//y pose
  k.stateSpace(2) = pRng->Normal( y_imu->measurementIMU.at(2),sqrt(.1));//z pose
  k.stateSpace(3) = pRng->Normal( y_imu->measurementIMU.at(3),sqrt(.1)); //x angular velocity
  k.stateSpace(4) = pRng->Normal( y_imu->measurementIMU.at(4),sqrt(.1)); //y angular velocity
  k.stateSpace(5) = pRng->Normal( y_imu->measurementIMU.at(5),sqrt(.1)); //z angular velocity
  k.stateSpace(6) = pRng->Normal(0,sqrt(.1)); //x position
  k.stateSpace(7) = pRng->Normal(0,sqrt(.1)); //y position
  k.stateSpace(8) = pRng->Normal(0,sqrt(.1)); //z position
  k.stateSpace(9) = pRng->Normal(0,sqrt(.1)); //x velocity
  k.stateSpace(10) = pRng->Normal(0,sqrt(.1)); //x velocity
  k.stateSpace(11) = pRng->Normal(0,sqrt(.1)); //x velocity
  k.stateSpace(12) = pRng->Normal(0,sqrt(.1)); //x accerleration
  k.stateSpace(13) = pRng->Normal(0,sqrt(.1)); //y accerleration
  k.stateSpace(14) = pRng->Normal(0,sqrt(.1)); //z accerleration

// std::cout <<  k.stateSpace << std::endl;
//std::cout <<  exp(1/N) << std::endl;

  return smc::particle<cv_state>(k, (1/N));
}

void GPSKernel(long lTime, smc::particle<cv_state> & pFrom, smc::rng *pRng) {//movements for GPS measurements
  cv_state * k = pFrom.GetValuePointer();

//std::cout << "In GPS Kernel" << std::endl;
  
  k->stateSpace(0) = k->stateSpace(0)+y_gps->deltaT*k->stateSpace(3)+ pRng->Normal(0,sqrt(.001)); //x pose
  k->stateSpace(1) = k->stateSpace(1)+y_gps->deltaT*k->stateSpace(4)+ pRng->Normal(0,sqrt(.001));//y pose
  k->stateSpace(2) = k->stateSpace(2)+y_gps->deltaT*k->stateSpace(5)+ pRng->Normal(0,sqrt(.001));//z pose
  k->stateSpace(3) = k->stateSpace(3)+pRng->Normal(0,sqrt(.001)); //x angular velocity
  k->stateSpace(4) = k->stateSpace(4)+pRng->Normal(0,sqrt(.001)); //y angular velocity
  k->stateSpace(5) = k->stateSpace(5)+pRng->Normal(0,sqrt(.001)); //z angular velocity
  k->stateSpace(6) = k->stateSpace(6)+y_gps->deltaT*k->stateSpace(9)+y_gps->deltaT*y_gps->deltaT*0.5*k->stateSpace(12)+pRng->Normal(0,sqrt(25)); //x position
  k->stateSpace(7) = k->stateSpace(7)+y_gps->deltaT*k->stateSpace(10)+y_gps->deltaT*y_gps->deltaT*0.5*k->stateSpace(13)+pRng->Normal(0,sqrt(25)); //y position
  k->stateSpace(8) = k->stateSpace(8)+y_gps->deltaT*k->stateSpace(11)+y_gps->deltaT*y_gps->deltaT*0.5*k->stateSpace(14)+pRng->Normal(0,sqrt(25)); //z position
  k->stateSpace(9) = k->stateSpace(9)+y_gps->deltaT*k->stateSpace(12)+pRng->Normal(0,sqrt(4)); //x velocity
  k->stateSpace(10) = k->stateSpace(10)+y_gps->deltaT*k->stateSpace(13)+pRng->Normal(0,sqrt(4)); //x velocity
  k->stateSpace(11) = k->stateSpace(11)+y_gps->deltaT*k->stateSpace(14)+pRng->Normal(0,sqrt(4)); //x velocity
  k->stateSpace(12) = k->stateSpace(12)+pRng->Normal(0,sqrt(.001)); //x accerleration
  k->stateSpace(13) = k->stateSpace(13)+pRng->Normal(0,sqrt(.001)); //y accerleration
  k->stateSpace(14) = k->stateSpace(14)+pRng->Normal(0,sqrt(.001)); //z accerleration

 //std::cout <<  k->stateSpace << std::endl;
 //std::cout <<   pFrom.GetWeight() << std::endl;
  
  pFrom.AddToLogWeight(logLikelihoodGPS(*k, *y_gps));
  
}
/**
 * @brief IMU粒子滤波器的预测和更新步骤
 * @details 这是粒子滤波中的采样/预测步骤,根据IMU测量更新粒子状态和权重
 * @param lTime 当前时间步
 * @param pFrom 待更新的粒子
 * @param pRng 随机数生成器指针
 * 
 * 状态向量包含:
 * 0-2: 姿态角(roll/pitch/yaw)
 * 3-5: 角速度
 * 6-8: 位置
 * 9-11: 速度 
 * 12-14: 加速度
 */
void IMUKernel(long lTime, smc::particle<cv_state> & pFrom, smc::rng *pRng) {
  cv_state * k = pFrom.GetValuePointer();
  
  // 运动学模型更新 - 姿态部分
  k->stateSpace(0) = k->stateSpace(0)+y_imu->deltaT*k->stateSpace(3)+ pRng->Normal(0,sqrt(9)); // 姿态角X + 角速度*时间 + 噪声
  k->stateSpace(1) = k->stateSpace(1)+y_imu->deltaT*k->stateSpace(4)+ pRng->Normal(0,sqrt(9)); // 姿态角Y
  k->stateSpace(2) = k->stateSpace(2)+y_imu->deltaT*k->stateSpace(5)+ pRng->Normal(0,sqrt(9)); // 姿态角Z
  
  // 角速度更新
  k->stateSpace(3) = k->stateSpace(3)+pRng->Normal(0,sqrt(.5)); // 角速度X + 噪声
  k->stateSpace(4) = k->stateSpace(4)+pRng->Normal(0,sqrt(.5)); // 角速度Y
  k->stateSpace(5) = k->stateSpace(5)+pRng->Normal(0,sqrt(.5)); // 角速度Z
  
  // 位置更新 - 匀加速运动模型
  k->stateSpace(6) = k->stateSpace(6)+y_imu->deltaT*k->stateSpace(9)+y_imu->deltaT*y_imu->deltaT*0.5*k->stateSpace(12)+pRng->Normal(0,sqrt(.0001)); // 位置X = X + v*t + 1/2*a*t^2
  k->stateSpace(7) = k->stateSpace(7)+y_imu->deltaT*k->stateSpace(10)+y_imu->deltaT*y_imu->deltaT*0.5*k->stateSpace(13)+pRng->Normal(0,sqrt(.0001)); // 位置Y
  k->stateSpace(8) = k->stateSpace(8)+y_imu->deltaT*k->stateSpace(11)+y_imu->deltaT*y_imu->deltaT*0.5*k->stateSpace(14)+pRng->Normal(0,sqrt(.0001)); // 位置Z
  
  // 速度更新 - 匀加速
  k->stateSpace(9) = k->stateSpace(9)+y_imu->deltaT*k->stateSpace(12)+pRng->Normal(0,sqrt(.005));  // 速度X = v + a*t
  k->stateSpace(10) = k->stateSpace(10)+y_imu->deltaT*k->stateSpace(13)+pRng->Normal(0,sqrt(.005)); // 速度Y 
  k->stateSpace(11) = k->stateSpace(11)+y_imu->deltaT*k->stateSpace(14)+pRng->Normal(0,sqrt(.005)); // 速度Z
  
  // 加速度更新 - 随机游走
  k->stateSpace(12) = k->stateSpace(12)+pRng->Normal(0,sqrt(.005)); // 加速度X
  k->stateSpace(13) = k->stateSpace(13)+pRng->Normal(0,sqrt(.005)); // 加速度Y 
  k->stateSpace(14) = k->stateSpace(14)+pRng->Normal(0,sqrt(.005)); // 加速度Z

  // 根据似然计算更新粒子权重
  pFrom.AddToLogWeight(logLikelihoodIMU(*k, *y_imu));
}
//AddToLogWeight
//MultiplyWeightBy

long fSelect(long lTime, const smc::particle<cv_state> & p, smc::rng *pRng) {

  
  if  (y_gps->newData == 1) {



return 1;

}
  else if  (y_imu->newData == 1){

 return 0;
}
else
{
std::cout << "No New Data" << std::endl;
return -1;
}

}
