#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <armadillo>

class KittiDataReader {
    public:
        KittiDataReader(const std::string& sequence_path) {
            // 构建完整的文件路径
            std::string data_path = sequence_path + "/oxts/data";
            std::string timestamp_path = sequence_path + "/oxts/timestamps.txt";
            
            // 获取所有数据文件
            getDataFiles(data_path);
            
            // 打开时间戳文件
            timestamp_file_.open(timestamp_path);
            if (!timestamp_file_.is_open()) {
                throw std::runtime_error("Failed to open timestamp file: " + timestamp_path);
            }
            
            current_frame_ = 0;
            total_frames_ = data_files_.size();
        }
    
        bool readNextFrame(double& timestamp, arma::vec& imu_data, arma::vec& gps_data) {
            if (current_frame_ >= total_frames_) {
                return false;
            }
    
            // 读取时间戳
            std::string timestamp_line;
            if (!std::getline(timestamp_file_, timestamp_line)) {
                return false;
            }
    
            // 打开当前帧的数据文件
            std::ifstream data_file(data_files_[current_frame_]);
            if (!data_file.is_open()) {
                return false;
            }
    
            // 读取数据
            std::string data_line;
            std::getline(data_file, data_line);
            
            // 解析时间戳
            struct tm tm = {};
            char ns[10];
            sscanf(timestamp_line.c_str(), "%d-%d-%d %d:%d:%d.%s", 
                   &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
                   &tm.tm_hour, &tm.tm_min, &tm.tm_sec, ns);
            
            timestamp = mktime(&tm);
    
            // 解析数据
            std::stringstream ss(data_line);
            std::vector<double> values;
            double val;
            while (ss >> val) {
                values.push_back(val);
            }
    
            if (values.size() < 20) {
                return false;
            }
    
            // 提取IMU数据
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
    
            // 提取GPS数据
            gps_data = arma::vec(6);
            gps_data(0) = values[0];  // lat
            gps_data(1) = values[1];  // lon
            gps_data(2) = values[2];  // alt
            gps_data(3) = values[6];  // vn
            gps_data(4) = values[7];  // ve
            gps_data(5) = values[8];  // vf
    
            current_frame_++;
            return true;
        }
    
        int getTotalFrames() const { return total_frames_; }
        int getCurrentFrame() const { return current_frame_; }
    
    private:
        void getDataFiles(const std::string& data_path) {
            // 使用系统命令获取所有数据文件
            std::string cmd = "ls " + data_path + "/*.txt | sort";
            FILE* pipe = popen(cmd.c_str(), "r");
            if (!pipe) {
                throw std::runtime_error("Failed to list data files");
            }
            
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
                std::string file_path(buffer);
                if (!file_path.empty() && file_path[file_path.length()-1] == '\n') {
                    file_path.erase(file_path.length()-1);
                }
                data_files_.push_back(file_path);
            }
            pclose(pipe);
            
            if (data_files_.empty()) {
                throw std::runtime_error("No data files found in: " + data_path);
            }
        }
    
        std::vector<std::string> data_files_;
        std::ifstream timestamp_file_;
        int current_frame_;
        int total_frames_;
    };