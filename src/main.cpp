#include "../include/smctc.h"
#include "./IMU-GPS-Fusion.h"
#include <cstdio> 
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
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
    y_imu = new IMU_obs(3, 5, 7, (timestamp_msec/1000.0)); 

    /**
     * @brief 初始化坐标转换系统
     * @details 创建坐标转换对象并设置初始参数
     */
    // 创建坐标转换工具对象
    CoordinateTransform transform;
    // 定义存储地理坐标的结构体(经度、纬度、高度)
    FrameCoordinates coordinates;

    // 初始化传感器数据标志位
    y_gps->newData = 0;  // GPS数据更新标志
    y_imu->newData = 0;  // IMU数据更新标志

    // 设置模拟的初始地理坐标（示例值，可根据需要调整）
    coordinates.first = 121.5;    // 经度
    coordinates.second = 31.2;    // 纬度
    coordinates.third = 10.0;     // 高度
    transform.set_initialGeodetic(coordinates);

    /**
     * @brief 设置模拟的IMU数据
     * @details 使用预设的固定值替代实际传感器读取的数据
     */
    y_imu->newData = 1;  // 设置数据更新标志

    // 设置预设的姿态角数据（横滚、俯仰、偏航，单位：度）
    arma::vec Theta = { 0.0, 0.0, 45.0 };        

    // 设置预设的角速度数据（绕x、y、z轴，单位：度/秒）
    arma::vec Omega = { 0.1, 0.2, 0.3 };  

    // 设置预设的加速度数据（x、y、z方向，单位：m/s^2，已减去重力加速度）
    arma::vec Acc = { 0.05, 0.1, 0.0 };  

    // 更新IMU测量数据
    y_imu->SetMeasurement(Theta, Omega, Acc, (timestamp_msec/1000.0));

    /**
     * @brief 设置模拟的GPS数据
     * @details 使用预设的固定值替代实际GPS接收机数据
     */
    y_gps->newData = 1;  // 设置数据更新标志

    // 将模拟的地理坐标转换为NED坐标系
    FrameCoordinates NED = transform.Geodetic_to_NED(coordinates);

    // 设置预设的位置数据（北、东、下，单位：米）
    arma::vec Pos = { NED.first, NED.second, NED.third };

    // 设置预设的速度数据（北、东、下方向，单位：m/s）
    double heading = 45.0;  // 航向角（度）
    double speed = 1.2;     // 水平速度（m/s）
    double climb = 0.0;     // 垂直速度（m/s）
    arma::vec Velocity = { cos(heading*M_PI/180.0)*speed, sin(heading*M_PI/180.0)*speed, climb };

    // 设置预设的误差参数
    double xerror = 5.0;  // 经度误差
    double yerror = 5.0;  // 纬度误差
    double zerror = 10.0; // 高度误差
    double serror = 1.0;  // 速度误差
    double cerror = 1.0;  // 垂直速度误差

    // 更新GPS观测数据
    y_gps->SetMeasurement(Pos, Velocity, xerror, yerror, zerror, serror, cerror, (timestamp_msec/1000));

    // 打印调试信息
    std::cout << "Initial GPS data:" << std::endl;
    std::cout << Pos << std::endl;      // 输出NED坐标系下的位置
    std::cout << Velocity << std::endl;  // 输出NED坐标系下的速度

    /**
     * @brief 粒子滤波器初始化与迭代计算
     * @details 设置并运行粒子滤波器，融合IMU和GPS数据进行状态估计
     */
    try {
        // 定义状态转移核函数数组(IMU核函数和GPS核函数)
        void (*pfMoves[])(long, smc::particle<cv_state> &,smc::rng*) = {IMUKernel, GPSKernel};

        // 创建移动集合对象，包含初始化、选择和状态转移函数
        smc::moveset<cv_state> Moveset(fInitialise, fSelect, sizeof(pfMoves)/sizeof(pfMoves[0]), pfMoves, NULL);

        // 初始化粒子滤波采样器(N个粒子，不保存历史)
        smc::sampler<cv_state> Sampler(N, SMC_HISTORY_NONE);  

        // 设置重采样参数(使用残差重采样方法，阈值0.8)
        Sampler.SetResampleParams(SMC_RESAMPLE_RESIDUAL, 0.8);

        // 设置移动集合并初始化采样器
        Sampler.SetMoveSet(Moveset);
        Sampler.Initialise();

        // 执行100次滤波迭代
        for(int n=0; n<100; ++n) {
            // 更新模拟的IMU和GPS数据（每次用略微不同的值，模拟运动）
            if (n % 10 == 0) {  // 每10次迭代更新一次GPS数据
                y_gps->newData = 1;
                
                // 位置随时间微小变化
                Pos(0) += 0.1 * cos(heading*M_PI/180.0);
                Pos(1) += 0.1 * sin(heading*M_PI/180.0);
                
                if (!ftime(&timer_msec)) {
                    timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
                                    (long long int) timer_msec.millitm;
                } else {
                    timestamp_msec = -1;
                }
                
                y_gps->SetMeasurement(Pos, Velocity, xerror, yerror, zerror, serror, cerror, (timestamp_msec/1000));
                std::cout << "Updated GPS data at iteration " << n << std::endl;
            }
            
            // 每次迭代都更新IMU数据
            y_imu->newData = 1;
            
            // 姿态角略微变化
            Theta(2) += 0.1;  // 偏航角缓慢变化
            
            if (!ftime(&timer_msec)) {
                timestamp_msec = ((long long int) timer_msec.time) * 1000ll + 
                                (long long int) timer_msec.millitm;
            } else {
                timestamp_msec = -1;
            }
            
            y_imu->SetMeasurement(Theta, Omega, Acc, timestamp_msec/1000.0);
            
            // 执行一次滤波迭代(包含预测和更新步骤)
            Sampler.Iterate();

            // 重置传感器数据更新标志
            y_gps->newData = 0;
            y_imu->newData = 0;
            
            // 计算状态估计的统计量
            double xm, xv, ym, yv;
            xm = Sampler.Integrate(integrand_mean_x, NULL);    // x方向位置均值
            xv = Sampler.Integrate(integrand_var_x, (void*)&xm); // x方向位置方差
            ym = Sampler.Integrate(integrand_mean_y, NULL);    // y方向位置均值
            yv = Sampler.Integrate(integrand_var_y, (void*)&ym); // y方向位置方差
                
            // 输出状态估计结果：x位置,y位置,x标准差,y标准差
            cout << "Iteration " << n << ": " << xm << "," << ym << "," << sqrt(xv) << "," << sqrt(yv) << endl;
            
            // 短暂延时，模拟实际系统的循环时间
            usleep(10000);  // 10毫秒
        }
    }
    catch(smc::exception e) {
        cerr << e;
        exit(e.lCode);
    }
    
    // 清理分配的资源
    delete y_gps;
    delete y_imu;
    
    return 0;
}

double integrand_mean_x(const cv_state& s, void *)
{
    return s.stateSpace(6);
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