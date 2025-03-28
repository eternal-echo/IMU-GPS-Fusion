#include <iostream>
#include <cmath>
#include <gsl/gsl_randist.h>

#include "../include/smctc.h"
#include "IMU-GPS-Fusion.h"

using namespace std;

  /**
   * @brief cv_state类的默认构造函数
   * @details 初始化状态空间向量，设置其维度为15并将所有元素初始化为0
   *          状态向量包含:
   *          - 位置 (3维)
   *          - 速度 (3维)
   *          - 姿态四元数 (4维)
   *          - 陀螺仪零偏 (3维)
   *          - 加速度计零偏 (2维)
   * @note 此构造函数用于创建一个新的cv_state对象实例，为后续的状态估计做准备
   */
  cv_state::cv_state() {

	stateSpace.set_size(15);
 	stateSpace.fill(0.0); 
    
  }

  /**
   * @brief GPS观测器类的构造函数，用于初始化GPS相关的矩阵和时间参数
   * @details 初始化包括：
   *          - 观测噪声协方差矩阵 R (6x6)
   *          - GPS测量值向量 (6x1)
   *          - 观测矩阵 H (6x15)
   * @param time 当前时间戳
   * 
   * @note 观测矩阵H的结构:
   * - H(0,6) = 1  对应位置x
   * - H(1,7) = 1  对应位置y
   * - H(2,8) = 1  对应位置z
   * - H(3,9) = 1  对应速度vx
   * - H(4,10) = 1 对应速度vy
   * - H(5,11) = 1 对应速度vz
   * 
   * @warning 使用前需确保time参数的单位统一性
   * 
   * @see 状态向量定义：
   * - [0-2]: 姿态角
   * - [3-5]: 姿态角速度
   * - [6-8]: 位置
   * - [9-11]: 速度
   * - [12-14]: 加速度偏差
   */
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

  /**
   * @brief 设置GPS观测数据和相关参数
   * @details 该函数用于更新GPS观测数据，包括位置、速度和对应的误差协方差
   * 
   * @param Pos [输入] 3维位置向量 (x,y,z)
   * @param Velocity [输入] 3维速度向量 (vx,vy,vz)
   * @param Xerror [输入] X方向位置测量误差
   * @param Yerror [输入] Y方向位置测量误差
   * @param Zerror [输入] Z方向位置测量误差
   * @param Verror [输入] 速度测量误差（适用于所有方向）
   * @param Cerror [输入] 时钟误差
   * @param currentT [输入] 当前时间戳
   * 
   * @note 状态变量说明：
   * - CovarianceMatrixR: 观测噪声协方差矩阵(6x6)
   * - measurementGPS: GPS测量值向量(6x1)，包含位置和速度
   * - lastTime: 上一次测量时间
   * - currentTime: 当前测量时间
   * - deltaT: 时间间隔
   * 
   * @warning 确保输入向量Pos和Velocity维度为3，否则可能导致越界访问
   */
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


  /**
   * @brief IMU观测类的构造函数
   * @details 初始化IMU观测相关的矩阵和时间参数
   *          - 初始化9x9的测量噪声协方差矩阵R
   *          - 初始化9维的IMU测量向量
   *          - 初始化9x15的观测矩阵H
   *          - 设置时间相关参数
   * 
   * @note 观测矩阵H的结构:
   *       - 前3行对应位置观测 (0-2,0-2)
   *       - 中间3行对应角速度观测 (3-5,0-2)
   *       - 最后3行对应加速度观测 (12-14,0-2)
   * 
   * @param 无输入参数
   * 
   * @member CovarianceMatrixR - 测量噪声协方差矩阵 (9x9)
   * @member measurementIMU    - IMU测量向量 (9x1)
   * @member sensorH          - 观测矩阵 (9x15)
   * @member lastTime         - 上一次测量时间戳
   * @member currentTime      - 当前测量时间戳
   * @member deltaT          - 时间增量
   */
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
  /**
   * @brief IMU观测类的构造函数，初始化IMU相关的观测矩阵和协方差矩阵
   * 
   * @param poseError 姿态测量误差标准差
   * @param gyroError 陀螺仪测量误差标准差
   * @param accError  加速度计测量误差标准差
   * @param time      初始时间戳
   * 
   * @details 该构造函数主要完成以下工作:
   * 1. 初始化9x9的观测噪声协方差矩阵R
   * 2. 初始化9维的IMU测量向量
   * 3. 初始化9x15的观测矩阵H
   * 4. 设置观测矩阵H的映射关系:
   *    - 0-2行对应位置状态
   *    - 3-5行对应角速度状态
   *    - 6-8行对应加速度状态
   * 5. 设置协方差矩阵R的对角元素
   * 6. 初始化时间相关变量
   * 
   * @note 观测向量的9个分量分别为:
   * - 0-2: 位置 [x, y, z]
   * - 3-5: 角速度 [wx, wy, wz]
   * - 6-8: 加速度 [ax, ay, az]
   */


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

  /**
   * @brief 设置IMU测量数据并更新时间信息
   * 
   * @param Theta 姿态角度向量，欧拉角 (roll角、pitch角、yaw角) [rad]
   * @param Omega 角速度向量 (x轴、y轴、z轴) [rad/s]
   * @param Acc 加速度向量 (x轴、y轴、z轴) [m/s^2]
   * @param currentT 当前时间戳 [s]
   * 
   * @details 该函数用于:
   * 1. 更新IMU测量数据到measurementIMU向量中
   *    - measurementIMU[0-2]: 姿态角度
   *    - measurementIMU[3-5]: 角速度
   *    - measurementIMU[6-8]: 加速度
   * 2. 更新时间信息
   *    - 保存上一时刻时间
   *    - 更新当前时间
   *    - 计算时间增量deltaT
   * 
   * @note measurementIMU是一个9维向量，按顺序存储了IMU的所有测量量
   * @note deltaT用于后续状态预测和卡尔曼滤波更新
   */
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

/**
 * @brief GPS测量更新步骤的核函数，用于粒子滤波中的状态预测
 * @details 该函数实现了基于GPS测量的状态转移模型，包括位姿、速度和加速度的更新
 * 
 * @param lTime 当前时间戳
 * @param pFrom 需要更新的粒子，包含15维状态向量
 * @param pRng 随机数生成器指针，用于添加过程噪声
 * 
 * @note 状态向量包含：
 * - 0-2: 姿态角(roll, pitch, yaw)
 * - 3-5: 角速度(wx, wy, wz)
 * - 6-8: 位置(x, y, z)
 * - 9-11: 速度(vx, vy, vz)
 * - 12-14: 加速度(ax, ay, az)
 * 
 * @note 关键参数：
 * - deltaT: GPS采样时间间隔
 * - 姿态噪声方差: 0.001
 * - 角速度噪声方差: 0.001
 * - 位置噪声方差: 25
 * - 速度噪声方差: 4
 * - 加速度噪声方差: 0.001
 * 
 * @note 更新步骤：
 * 1. 基于运动学模型更新姿态
 * 2. 更新角速度（添加随机扰动）
 * 3. 使用二阶运动学模型更新位置（考虑速度和加速度）
 * 4. 使用一阶运动学模型更新速度
 * 5. 更新加速度（添加随机扰动）
 * 6. 计算并更新粒子权重
 */
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
