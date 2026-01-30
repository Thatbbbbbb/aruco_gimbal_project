#pragma once
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <cstdint>

// 云台数据结构体：接收的角度和发送的角度
struct GimbalData {
    float yaw = 0.0f;    // 偏航角，单位度
    float pitch = 0.0f;  // 俯仰角，单位度
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    // 初始化串口：端口名、波特率
    bool init(const std::string& port_name, uint32_t baudrate = 115200);

    // 关闭串口
    void close();

    // 接收云台当前角度：返回是否成功
    bool receiveGimbalData(GimbalData& data);

    // 发送目标角度给云台：返回是否成功
    bool sendGimbalData(const GimbalData& data);

private:
    int serial_fd_ = -1; // 串口文件描述符
};

#endif