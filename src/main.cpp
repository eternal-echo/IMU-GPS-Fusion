#include "../include/smctc.h"
#include "./IMU-GPS-Fusion.h"
#include <cstdio> 
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <sstream>
#include <math.h>
#include <time.h> 
#include <sys/timeb.h>
#include "../data/kitti/KITTI_dataset.h" 

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

int main(int argc, char** argv){
    // 创建结果目录
    system("mkdir -p ../results");
    // 创建输出文件
    std::ofstream trajectory_file("../results/trajectory.dat");
    std::ofstream error_file("../results/error.dat");
    trajectory_file << "# X_Mean Y_Mean GPS_X GPS_Y\n";
    error_file << "# Iteration X_StdDev Y_StdDev\n";
    // 初始化数据读取器
    KittiDataReader reader;  // 不再需要路径参数
    std::cout << "Total frames: " << reader.getTotalFrames() << std::endl;
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

    // 创建坐标转换工具对象
    //CoordinateTransform transform;
    // 定义存储地理坐标的结构体(经度、纬度、高度)
    //FrameCoordinates coordinates;

    // 初始化传感器数据标志位
    y_gps->newData = 0;  // GPS数据更新标志
    y_imu->newData = 0;  // IMU数据更新标志

    // 设置模拟的初始地理坐标（示例值，可根据需要调整）
    //coordinates.first = 48.997575979523;    // 经度
    //coordinates.second = 8.4772921616664;    // 纬度
    //coordinates.third = 123.67921447754;     // 高度
    //transform.set_initialGeodetic(coordinates);

    try{
        double timestamp;
        arma::vec imu_data, gps_data;
        int frame_count = 0;

        // 初始化数据读取器
        KittiDataReader reader;
        std::cout << "Total frames: " << reader.getTotalFrames() << std::endl;
    /**

    // 打印调试信息
    std::cout << "Initial GPS data:" << std::endl;
    std::cout << Pos << std::endl;      // 输出NED坐标系下的位置
    std::cout << Velocity << std::endl;  // 输出NED坐标系下的速度

    /**
     * @brief 粒子滤波器初始化与迭代计算
     * @details 设置并运行粒子滤波器，融合IMU和GPS数据进行状态估计
     */
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

        while(reader.readNextFrame(timestamp, imu_data, gps_data)) {
            std::cout << "Processing frame " << ++frame_count 
                     << " of " << reader.getTotalFrames() << "\r" << std::flush;
            
            // 处理数据
            y_imu->newData = 1;
            arma::vec Theta = {imu_data(0), imu_data(1), imu_data(2)}; 
            arma::vec Omega = {imu_data(3), imu_data(4), imu_data(5)}; 
            arma::vec Acc = {imu_data(6), imu_data(7), imu_data(8)};
            y_imu->SetMeasurement(Theta, Omega, Acc, timestamp);

            y_gps->newData = 1;
            arma::vec Pos = {gps_data(0), gps_data(1), gps_data(2)};
            arma::vec Velocity = {gps_data(3), gps_data(4), gps_data(5)};
            double xerror = 5.0;  // 经度误差
            double yerror = 5.0;  // 纬度误差
            double zerror = 10.0; // 高度误差
            double serror = 1.0;  // 速度误差
            double cerror = 1.0;  // 垂直速度误差
            y_gps->SetMeasurement(Pos, Velocity, xerror, yerror, zerror, serror, cerror, timestamp);
            
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
                
            
            // 保存轨迹数据
            trajectory_file << std::setprecision(10) << xm << " " << ym << " " 
            << Pos(0) << " " << Pos(1) << "\n";

            // 保存误差数据
            error_file << std::setprecision(6) << n << " " << sqrt(xv) << " " << sqrt(yv) << " " << timestamp << "\n";
            std::cout << "\nProcessing complete!" << std::endl;

            // 短暂延时，模拟实际系统的循环时间
            usleep(10000);  // 10毫秒
        }
        trajectory_file.close();
        error_file.close(); 

        std::ofstream plot_script("../results/plot_commands.gp");
        plot_script << "set terminal png size 1600,1200 enhanced font 'Arial,12'\n"
                   << "set output '../results/particle_filter_results.png'\n"
                   << "set multiplot layout 2,1 spacing 0.15\n"
                   << "set size 1,0.6\n"
                   << "set origin 0,0.4\n"
                   << "set autoscale\n"
                   << "set title 'Particle Filter Trajectory (KITTI Dataset)' font 'Arial,14'\n"
                   << "set xlabel 'Longitude (degrees)' font 'Arial,12'\n"
                   << "set ylabel 'Latitude (degrees)' font 'Arial,12'\n"
                   << "set grid\n"
                   << "set key outside right\n"
                   << "set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5\n"
                   << "set style line 2 lc rgb '#dd181f' lt 1 lw 1 pt 5 ps 1.0\n"
                   << "# Switch x and y columns since longitude is x and latitude is y\n"
                   << "plot '../results/trajectory.dat' using 2:1 with linespoints ls 1 title 'Estimated Path',\\\n"
                   << "     '../results/trajectory.dat' using 4:3 with points ls 2 title 'GPS Measurements'\n"
                   << "set size 1,0.4\n"
                   << "set origin 0,0\n"
                   << "set title 'Estimation Uncertainty vs Time' font 'Arial,14'\n"
                   << "set xlabel 'Frame Number' font 'Arial,12'\n"
                   << "set ylabel 'Standard Deviation (degrees)' font 'Arial,12'\n"
                   << "set style line 3 lc rgb '#008000' lt 1 lw 2\n"
                   << "set style line 4 lc rgb '#800080' lt 1 lw 2\n"
                   << "plot '../results/error.dat' using 1:2 with lines ls 3 title 'Latitude StdDev',\\\n"
                   << "     '../results/error.dat' using 1:3 with lines ls 4 title 'Longitude StdDev'\n"
                   << "unset multiplot\n";
        plot_script.close();
    
        // 执行 gnuplot 绘图命令
        system("cd ../results && gnuplot plot_commands.gp");       
    }

} catch(const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
}catch(smc::exception e) {
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