#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <armadillo>

class KittiDataReader {
public:
    KittiDataReader(const std::string& base_path) {
         // 直接打开测试数据文件
         data_file_.open("/mnt/c/Users/Lenovo/Documents/GitHub/2011_09_26/2011_09_26_drive_0084_sync/oxts/10datatest.txt");
         // KITTI的时间戳文件
        timestamp_file_.open("/mnt/c/Users/Lenovo/Documents/GitHub/2011_09_26/2011_09_26_drive_0084_sync/oxts/timestamps.txt");
        if (!data_file_.is_open()) {
            throw std::runtime_error("Failed to open data file");
        }
        if (!timestamp_file_.is_open()) {
            throw std::runtime_error("Failed to open timestamp file");
        }
        frame_count_ = 0;
    }
 
    ~KittiDataReader() {
        if (data_file_.is_open()) {
            data_file_.close();
        }
        if (timestamp_file_.is_open()) {
            timestamp_file_.close();
        }
    }
        //imu_file_.open(mnt/c/Users/Lenovo/Documents/GitHub/2011_09_26/2011_09_26_drive_0084_sync/oxts/data/0000000000.txt);//KITTI的IMU数据文件
        

    bool readNextFrame(double& timestamp, arma::vec& imu_data, arma::vec& gps_data) {
        std::string data_line, timestamp_line;
        
        // 读取时间戳和数据
        if (!std::getline(timestamp_file_, timestamp_line) || 
            !std::getline(data_file_, data_line)) {
            return false;
        }

        // 解析时间戳（KITTI格式：2011-09-26 13:02:44.000000000）
        struct tm tm = {};
        char ns[10];
        sscanf(timestamp_line.c_str(), "%d-%d-%d %d:%d:%d.%s", 
               &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
               &tm.tm_hour, &tm.tm_min, &tm.tm_sec, ns);
        
        timestamp = mktime(&tm);

        // 解析一行数据
        std::stringstream ss(line);

        std::vector<double> values;
        double val;
        while (ss >> val) {
            values.push_back(val);
        }

        if (values.size() < 20) {  // 确保数据完整性
            return false;
        }

    //bool readNextFrame(double& timestamp, arma::vec& imu_data, arma::vec& gps_data) {
        //std::string line, timestamp_line;
        
        //if(!std::getline(timestamp_file_, timestamp_line) || 
        //   !std::getline(imu_file_, line)) {
        //    return false;
        //}

        // 解析时间戳（KITTI格式：2011-09-26 13:02:44.000000000）
        //struct tm tm = {};
        //char ns[10];
        //sscanf(timestamp_line.c_str(), "%d-%d-%d %d:%d:%d.%s", 
        //       &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
        //       &tm.tm_hour, &tm.tm_min, &tm.tm_sec, ns);
        
        //timestamp = mktime(&tm);

        // 解析IMU和GPS数据
        // KITTI oxts格式：lat lon alt roll pitch yaw vn ve vf ...
        //std::stringstream ss(line);
        //std::vector<double> values;
        //double val;
        //while(ss >> val) {
        //    values.push_back(val);
        //}

        // 提取IMU数据（姿态角、角速度、加速度）
        imu_data = arma::vec(9);
        imu_data(0) = values[3];  // roll
        imu_data(1) = values[4];  // pitch
        imu_data(2) = values[5];  // yaw
        imu_data(3) = values[17]; // wx
        imu_data(4) = values[18]; // wy
        imu_data(5) = values[19]; // wz
        imu_data(6) = values[11]; // ax
        imu_data(7) = values[12]; // ay
        imu_data(8) = values[13]; // az

        // 提取GPS数据（位置和速度）
        gps_data = arma::vec(6);
        gps_data(0) = values[0];  // lat
        gps_data(1) = values[1];  // lon
        gps_data(2) = values[2];  // alt
        gps_data(3) = values[6];  // vn
        gps_data(4) = values[7];  // ve
        gps_data(5) = values[8]; // vf

        // 使用帧计数作为时间戳
        timestamp = frame_count_++;

        return true;
    }

    int getFrameCount() const { return frame_count_; }

private:
    std::ifstream data_file_;
    std::ifstream timestamp_file_;
    int frame_count_;
    //std::ifstream imu_file_;
    //std::ifstream timestamp_file_;
};