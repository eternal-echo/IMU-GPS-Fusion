#include "../include/smctc.h"
#include "./IMU-GPS-Fusion.h"
#include <cstdio> 
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <libgpsmm.h>
#include <math.h>
#include <time.h> 
#include <sys/timeb.h>  

using namespace std;


/**
 * @brief GPS观测数据类指针
 * @details 用于存储和管理GPS观测数据，包含位置、速度和误差信息
 */
GPS_obs * y_gps; 

/**
 * @brief IMU观测数据类指针
 * @details 用于存储和管理IMU观测数据，包含姿态角、角速度和加速度信息
 */
IMU_obs * y_imu; 

/**
 * @brief 计算状态x方向均值的积分函数
 * @param s 系统状态变量
 * @param void* 未使用的指针参数
 * @return x方向状态估计的均值
 */
double integrand_mean_x(const cv_state&, void*);

/**
 * @brief 计算状态y方向均值的积分函数
 * @param s 系统状态变量
 * @param void* 未使用的指针参数
 * @return y方向状态估计的均值
 */
double integrand_mean_y(const cv_state&, void*);

/**
 * @brief 计算状态x方向方差的积分函数
 * @param s 系统状态变量
 * @param vmx x方向均值的指针
 * @return x方向状态估计的方差
 */
double integrand_var_x(const cv_state&, void*);

/**
 * @brief 计算状态y方向方差的积分函数
 * @param s 系统状态变量
 * @param vmy y方向均值的指针
 * @return y方向状态估计的方差
 */
double integrand_var_y(const cv_state&, void*);

int main(int argc, char** argv)
{

/**
 * @brief 初始化时间戳和传感器对象
 * @details 创建并配置GPS和IMU观测对象，设置相关参数和初始时间戳
 */

// 定义时间戳相关变量
struct timeb timer_msec;  // 用于获取系统时间的结构体
long long int timestamp_msec; // 存储毫秒级时间戳

// 获取系统当前时间戳
if (!ftime(&timer_msec)) {
  // 将秒和毫秒转换为统一的毫秒时间戳
  timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
            (long long int) timer_msec.millitm;
}
else {
  timestamp_msec = -1;  // 获取时间失败时设置为-1
}
printf("%lld milliseconds since epoch\n", timestamp_msec);

// 初始化传感器观测对象
// GPS观测对象：输入参数为初始时间(秒)
y_gps = new GPS_obs(timestamp_msec/1000);

// IMU观测对象：输入参数为姿态角方差(3)、角速度方差(5)、加速度方差(7)和初始时间(秒)
// 这些方差参数用于传感器融合时的权重计算
y_imu = new IMU_obs(3,5,7, (timestamp_msec/1000.0)); 



/**
 * @brief 初始化传感器和坐标转换系统
 * @details 创建坐标转换对象并配置BNO055 IMU传感器
 */

// 创建坐标转换工具对象
CoordinateTransform transform;
// 定义存储地理坐标的结构体(经度、纬度、高度)
FrameCoordinates coordinates;

// 初始化传感器数据标志位
y_gps->newData = 0;  // GPS数据更新标志
y_imu->newData = 0;  // IMU数据更新标志

// 初始化BNO055 IMU传感器
// 参数说明: -1(默认I2C), BNO055_ADDRESS_A(I2C地址), 1(RST引脚)
BNO055 bno = BNO055(-1, BNO055_ADDRESS_A, 1);

// 配置BNO055工作模式为9轴融合模式(禁用快速磁力计校准)
bno.begin(bno.OPERATION_MODE_NDOF_FMC_OFF);

// 等待传感器初始化完成(1秒)
usleep(1000000);

// 读取传感器温度进行自检
int temp = bno.getTemp();
std::cout << "Current Temperature: " << temp << " C" << std::endl;

// 启用外部晶振以提高精度
bno.setExtCrystalUse(true);

// 等待配置生效(1秒)
usleep(1000000);

//
/**
 * @brief IMU数据获取和坐标转换步骤
 * @details 从BNO055 IMU传感器读取四元数、加速度、角速度和欧拉角数据
 *          并将其从体坐标系转换到NED坐标系
 * @input BNO055传感器原始数据
 * @output 转换后的姿态角(Theta)、角速度(Omega)和加速度(Acc)
 */

// 获取IMU四元数数据
imu::Quaternion quat = bno.getQuat();

// 将四元数数据转换为自定义四元数结构体
Quaternions q;
q.W = quat.w();  // 四元数实部
q.X = quat.x();  // 四元数虚部x分量
q.Y = quat.y();  // 四元数虚部y分量
q.Z = quat.z();  // 四元数虚部z分量

// 获取IMU加速度计数据(体坐标系)
imu::Vector<3> acc_temp = bno.getVector(BNO055::VECTOR_ACCELEROMETER);

// 将加速度数据转换为坐标框架结构体(体坐标系)
FrameCoordinates acc_bframe;
acc_bframe.first = acc_temp.x();   // x方向加速度
acc_bframe.second = acc_temp.y();  // y方向加速度
acc_bframe.third = acc_temp.z();   // z方向加速度

// 将体坐标系加速度转换到NED坐标系
FrameCoordinates acc_NEDframe = transform.Bframe_to_NED(q, acc_bframe);

// 获取IMU陀螺仪数据(体坐标系)
imu::Vector<3> ang_temp = bno.getVector(BNO055::VECTOR_GYROSCOPE);

// 将角速度数据转换为坐标框架结构体(体坐标系)
FrameCoordinates gyro_bframe;
gyro_bframe.first = ang_temp.x();   // x轴角速度
gyro_bframe.second = ang_temp.y();  // y轴角速度
gyro_bframe.third = ang_temp.z();   // z轴角速度

// 将体坐标系角速度转换到NED坐标系
FrameCoordinates gyro_NEDframe = transform.Bframe_to_NED(q, gyro_bframe);

// 获取IMU欧拉角数据
imu::Vector<3> pose_temp = bno.getVector(BNO055::VECTOR_EULER);

// 将欧拉角数据转换为坐标框架结构体
FrameCoordinates pose;
pose.first = pose_temp.x();   // 横滚角(roll)
pose.second = pose_temp.y();  // 俯仰角(pitch)
pose.third = pose_temp.z();   // 偏航角(yaw)

// 设置IMU数据更新标志
y_imu->newData = 1;

// 将数据转换为Armadillo向量格式
arma::vec Theta = { pose.first, pose.second, pose.third };        // 姿态角向量
arma::vec Omega = { gyro_NEDframe.first, gyro_NEDframe.second, gyro_NEDframe.third };  // NED系角速度向量
arma::vec Acc = { acc_NEDframe.first, acc_NEDframe.second, acc_NEDframe.third-9.81 };  // NED系加速度向量(减去重力加速度)

// 更新IMU测量数据
// 输入参数：姿态角、角速度、加速度和时间戳(秒)
y_imu->SetMeasurement (Theta, Omega, Acc, (timestamp_msec/1000.0));

//



//

  /**
   * @brief GPS数据读取与处理模块
   * @details 从GPSD服务器读取GPS数据并进行处理，包括位置、速度和误差信息的获取
   * @input GPSD服务器数据
   * @output 地理坐标(coordinates)、运动参数(heading,speed,climb)和误差参数
   * @param coordinates 经纬度和高度的地理坐标结构体
   * @param heading 航向角
   * @param speed 速度
   * @param climb 垂直速度
   * @param xerror,yerror,zerror 位置误差
   * @param serror 速度误差
   * @param cerror 垂直速度误差
   */

  // 创建GPSD客户端连接对象
  gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

  // 检查GPSD服务是否运行并开启数据流
  if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
    std::cerr << "No GPSD running.\n";
    return 1;
  }

  // 定义GPS数据结构指针
  struct gps_data_t *gpsd_data;

  // 等待GPS数据(超时时间1秒)
  if (!gps_rec.waiting(1000000)) {
    std::cerr << "GPSD timeout error.\n";
    return 2;
  }

  // 读取GPS数据
  if ((gpsd_data = gps_rec.read()) == NULL) {
    std::cerr << "GPSD read error.\n";
    return 1;
  } else {
    // 等待获得有效的GPS定位(2D或3D)
    while (((gpsd_data = gps_rec.read()) == NULL) ||
        (gpsd_data->fix.mode < MODE_2D)) {
      std::cout << "waiting for GPS fix" << std::endl;
      usleep(1000000);
    }

    // 获取GPS时间戳
    timestamp_t ts = gpsd_data->fix.time;

    // 获取地理坐标信息
    coordinates.second = gpsd_data->fix.latitude;   // 纬度
    coordinates.first = gpsd_data->fix.longitude;   // 经度
    coordinates.third = gpsd_data->fix.altitude;    // 高度

    // 获取运动参数
    double heading = gpsd_data->fix.track;  // 航向角
    double speed = gpsd_data->fix.speed;    // 速度
    double climb = gpsd_data->fix.climb;    // 垂直速度

    // 获取误差参数
    double xerror = gpsd_data->fix.epx;  // 经度误差
    double yerror = gpsd_data->fix.epy;  // 纬度误差
    double zerror = gpsd_data->fix.epv;  // 高度误差
    double serror = gpsd_data->fix.eps;  // 速度误差
    double cerror = gpsd_data->fix.epc;  // 垂直速度误差

    // 处理无效数据，设置默认值
    if (isnan(xerror)) xerror = 1000;
    if (isnan(yerror)) yerror = 1000;
    if (isnan(zerror)) zerror = 1000;
    if (isnan(serror)) serror = 1000;
    if (isnan(cerror)) cerror = 1000;
    if (isnan(speed)) speed = 0;
    if (isnan(climb)) climb = 0;
if (isnan(heading))
		heading = 45;



transform.set_initialGeodetic(coordinates);

/**
 * @brief GPS数据坐标系转换与观测值设置
 * @details 将GPS经纬度坐标转换为NED坐标系，并设置GPS观测数据
 * 
 * @param input 
 *   - coordinates: 地理坐标(经度、纬度、高度)
 *   - heading: GPS航向角
 *   - speed: GPS速度
 *   - climb: 垂直速度
 *   - xerror,yerror,zerror: 位置误差
 *   - serror,cerror: 速度误差
 *   - timestamp_msec: 时间戳(毫秒)
 * 
 * @param output
 *   - y_gps: 更新后的GPS观测数据对象
 */

// 将地理坐标转换为NED坐标系
FrameCoordinates NED = transform.Geodetic_to_NED(coordinates);

// 处理NED坐标中的无效值
if (isnan(NED.first))
  NED.first = 0;
if (isnan(NED.second))
  NED.second = 0;
if (isnan(NED.third))
  NED.third = 0;

// 设置GPS数据更新标志
y_gps->newData = 1;

// 构建位置向量(NED坐标系)
arma::vec Pos = { NED.first, NED.second, NED.third };

// 构建速度向量(NED坐标系)
// 使用航向角和速度计算水平分量，climb为垂直分量
arma::vec Velocity = { cos(heading*2*M_PI)*speed, sin(heading*2*M_PI)*speed, climb };

// 更新GPS观测数据
// 参数：位置、速度、位置误差(x,y,z)、速度误差、垂直速度误差、时间戳(秒)
y_gps->SetMeasurement(Pos, Velocity, xerror, yerror, zerror, serror, cerror, (timestamp_msec/1000));

// 打印调试信息
std::cout << "New data" << std::endl;
std::cout << Pos << std::endl;      // 输出NED坐标系下的位置
std::cout << Velocity << std::endl;  // 输出NED坐标系下的速度

  }
  

//

  
  try {

/**
 * @brief 粒子滤波器初始化与迭代计算
 * @details 设置并运行粒子滤波器，融合IMU和GPS数据进行状态估计
 * 
 * @param 输入:
 *   - y_gps: GPS观测数据(位置、速度)
 *   - y_imu: IMU观测数据(姿态、角速度、加速度)
 * 
 * @param 状态变量:
 *   - cv_state: 包含位置、速度、姿态等的系统状态向量
 *   - Sampler: 粒子滤波采样器对象
 * 
 * @param 输出:
 *   - xm,ym: 位置估计的均值(x,y方向)
 *   - xv,yv: 位置估计的方差(x,y方向)
 * 
 * @param 参数:
 *   - N: 粒子数量(在其他模块中定义)
 *   - 重采样阈值: 0.8
 */

// 定义状态转移核函数数组(IMU核函数和GPS核函数)
void (*pfMoves[])(long, smc::particle<cv_state> &,smc::rng*) = {IMUKernel, GPSKernel};

// 创建移动集合对象，包含初始化、选择和状态转移函数
smc::moveset<cv_state> Moveset(fInitialise, fSelect,sizeof(pfMoves) /sizeof(pfMoves[0]),pfMoves, NULL);

// 初始化粒子滤波采样器(N个粒子，不保存历史)
smc::sampler<cv_state> Sampler(N, SMC_HISTORY_NONE);  

// 设置重采样参数(使用残差重采样方法，阈值0.8)
Sampler.SetResampleParams(SMC_RESAMPLE_RESIDUAL, .8);

// 设置移动集合并初始化采样器
Sampler.SetMoveSet(Moveset);
Sampler.Initialise();

// 执行100次滤波迭代
for(int n=0 ; n <100 ; ++n) {
  
  // 执行一次滤波迭代(包含预测和更新步骤)
  Sampler.Iterate();

  // 重置传感器数据更新标志
  y_gps->newData = 0;
  y_imu->newData = 0;
    
  // 计算状态估计的统计量
  double xm,xv,ym,yv;
  xm = Sampler.Integrate(integrand_mean_x,NULL);    // x方向位置均值
  xv = Sampler.Integrate(integrand_var_x, (void*)&xm); // x方向位置方差
  ym = Sampler.Integrate(integrand_mean_y,NULL);    // y方向位置均值
  yv = Sampler.Integrate(integrand_var_y, (void*)&ym); // y方向位置方差
    
  // 输出状态估计结果：x位置,y位置,x标准差,y标准差
  cout << xm << "," << ym << "," << sqrt(xv) << "," << sqrt(yv) << endl;

/**
 * @brief 时间戳更新模块
 * @details 获取最新的系统时间并更新时间戳
 * 
 * @param 输入:
 *   - timer_msec: 系统时间结构体(包含秒和毫秒)
 * 
 * @param 状态变量:
 *   - timestamp_msec: 毫秒级时间戳
 * 
 * @param 输出:
 *   - timestamp_msec: 更新后的时间戳
 * 
 * @param 返回值:
 *   成功返回正常时间戳，失败返回-1
 */

//usleep(100000);  // 可选的延时函数，用于控制采样频率

// 获取系统当前时间
if (!ftime(&timer_msec)) {
  // 将秒和毫秒合并为统一的毫秒时间戳
  // time字段表示秒，millitm字段表示毫秒
  timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
            (long long int) timer_msec.millitm;
}
else {
  // 获取时间失败时设置为错误标志
  timestamp_msec = -1;
}
 // printf("%lld milliseconds since epoch\n", timestamp_msec);



//
/**
 * @brief IMU传感器数据获取与坐标转换函数
 * @details 从BNO055 IMU读取姿态、加速度、角速度数据并进行坐标转换
 * 
 * @param 输入:
 *   - bno: BNO055 IMU传感器对象
 * 
 * @param 状态变量:
 *   - q: 四元数结构体，用于姿态表示
 *   - acc_bframe/gyro_bframe: 体坐标系下的加速度/角速度
 *   - pose: 欧拉角姿态数据
 * 
 * @param 输出:
 *   - Theta: 欧拉角向量(roll,pitch,yaw)
 *   - Omega: NED坐标系下的角速度向量
 *   - Acc: NED坐标系下的线性加速度向量(已去除重力)
 * 
 * @param 参数:
 *   - 重力加速度: 9.81m/s^2
 */

// 获取IMU四元数数据
imu::Quaternion quat = bno.getQuat();

// 将IMU四元数转换为自定义四元数结构体
Quaternions q;
q.W = quat.w();  // 实部
q.X = quat.x();  // x分量
q.Y = quat.y();  // y分量
q.Z = quat.z();  // z分量

// 获取体坐标系下的加速度数据
imu::Vector<3> acc_temp = bno.getVector(BNO055::VECTOR_ACCELEROMETER);

// 将加速度数据转换为坐标框架格式
FrameCoordinates acc_bframe;
acc_bframe.first = acc_temp.x();   // x轴加速度
acc_bframe.second = acc_temp.y();  // y轴加速度
acc_bframe.third = acc_temp.z();   // z轴加速度

// 将加速度从体坐标系转换到NED坐标系
FrameCoordinates acc_NEDframe = transform.Bframe_to_NED(q, acc_bframe);

// 获取体坐标系下的角速度数据
imu::Vector<3> ang_temp = bno.getVector(BNO055::VECTOR_GYROSCOPE);

// 将角速度数据转换为坐标框架格式
FrameCoordinates gyro_bframe;
gyro_bframe.first = ang_temp.x();   // 绕x轴角速度
gyro_bframe.second = ang_temp.y();  // 绕y轴角速度
gyro_bframe.third = ang_temp.z();   // 绕z轴角速度

// 将角速度从体坐标系转换到NED坐标系
FrameCoordinates gyro_NEDframe = transform.Bframe_to_NED(q, gyro_bframe);

// 获取欧拉角数据
imu::Vector<3> pose_temp = bno.getVector(BNO055::VECTOR_EULER);

// 将欧拉角数据转换为坐标框架格式
FrameCoordinates pose;
pose.first = pose_temp.x();   // 横滚角(roll)
pose.second = pose_temp.y();  // 俯仰角(pitch)
pose.third = pose_temp.z();   // 偏航角(yaw)

// 设置IMU数据更新标志
y_imu->newData = 1;

// 构建状态向量(用于滤波器)
arma::vec Theta = { pose.first, pose.second, pose.third };         // 姿态角向量
arma::vec Omega = { gyro_NEDframe.first, gyro_NEDframe.second, gyro_NEDframe.third };  // NED系角速度
arma::vec Acc = { acc_NEDframe.first, acc_NEDframe.second, acc_NEDframe.third-9.81 };  // NED系加速度(减去重力)

if (!ftime(&timer_msec)) {
    timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
                        (long long int) timer_msec.millitm;
  }
  else {
    timestamp_msec = -1;
  }
 // printf("%lld milliseconds since epoch\n", timestamp_msec);


y_imu->SetMeasurement (Theta,  Omega, Acc, timestamp_msec/1000.0);

if ((gpsd_data = gps_rec.read()) != NULL)
{
y_gps->newData = 1;
arma::vec Pos = { 0, 0, 0 };
arma::vec Velocity = { 0, 0, 0 };

if (!ftime(&timer_msec)) {
    timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
                        (long long int) timer_msec.millitm;
  }
  else {
    timestamp_msec = -1;
  }
 // printf("%lld milliseconds since epoch\n", timestamp_msec);


y_gps->SetMeasurement(Pos, Velocity, 5, 5, 5, 5, 5, (timestamp_msec/1000));
std::cout << "New data" << std::endl;
}


}
    
  }

  catch(smc::exception  e)
    {
      cerr << e;
      exit(e.lCode);
    }
}



double integrand_mean_x(const cv_state& s, void *)
{
//std::cout << s.stateSpace << std::endl;
  return  s.stateSpace(6);
}

double integrand_var_x(const cv_state& s, void* vmx)
{
  double* dmx = (double*)vmx;
  double d = (s.stateSpace(6) - (*dmx));
  return d*d;
}

double integrand_mean_y(const cv_state& s, void *)
{
  return s.stateSpace(7);
}

double integrand_var_y(const cv_state& s, void* vmy)
{
  double* dmy = (double*)vmy;
  double d = (s.stateSpace(7) - (*dmy));
  return d*d;
}



