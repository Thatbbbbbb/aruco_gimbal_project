#pragma once
#include <string>
#include <vector>
#include <termios.h>

namespace hitcrt {
namespace serial {

// 电机角度结构体
std::vector<double> target_angles_vec={0.0, 0.0, 0.0};
// 串口通信类（波特率固化115200）
class SerialPort {
private:
    int fd;                 // 串口文件描述符
    std::string port;       // 串口名
    int timeout;            // 超时时间（ms）
    struct termios old_tio; // 保存旧的串口配置

public:
    // 构造函数（移除波特率参数）
    SerialPort(const std::string& port, int timeout);
    
    // 析构函数（恢复串口配置）
    ~SerialPort();

    // 打开串口
    bool open_port();

    // 关闭串口
    void close_port();

    // 接收当前电机角度
    bool receive_motor_angles(std::vector<double>& angles);

    // 发送目标电机角度
    bool send_motor_angles(const std::vector<double>& angles);
};

} // namespace serial
} // namespace hitcrt