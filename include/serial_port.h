// serial_port.h
#pragma once
#include <string>
#include <cstdint>
#include <iostream>

// 1. 定义协议宏（解决 FRAME_HEAD 未定义问题）
#define FRAME_HEAD {0xAA, 0x55}
#define FRAME_TAIL {0x55, 0xAA}
#define FRAME_TOTAL_LEN 15  // 对应你代码中的帧长度（15字节）

// 2. 定义串口配置结构体（SerialConfig）
struct SerialConfig {
    uint32_t baudrate = 115200;    // 默认波特率
    uint32_t timeout_ms = 1000;    // 默认超时1秒
    int retry_times = 3;           // 默认重试3次
    bool crc_check = true;         // 默认开启CRC校验
};

// 3. 定义云台数据结构体（GimbalData）
struct GimbalData {
    float yaw = 0.0f;
    float pitch = 0.0f;
};

// 4. 声明全局CRC8函数（你的代码中用到了，需提前声明）
uint8_t crc8(const uint8_t* data, uint32_t len);

// 5. 声明SerialPort类（所有成员函数仅声明，无实现）
class SerialPort {
private:
    int serial_fd_ = -1;           // 串口文件描述符
    SerialConfig serial_config_;   // 串口配置

    // 私有成员函数声明（仅内部使用）
    uint16_t calculateCRC16(const uint8_t* data, uint16_t len);
    bool syncFrameHeader();
    void packFrame(const GimbalData& data, uint8_t* frame, uint16_t& frame_len);
    bool unpackFrame(const uint8_t* frame, uint16_t frame_len, GimbalData& data);

public:
    // 构造/析构声明
    ~SerialPort();

    // 公有成员函数声明（两个重载的init，对应你的两种实现）
    bool init(const std::string& port_name, uint32_t baudrate);
    bool init(const std::string& port_name, const SerialConfig& config);

    // 公有成员函数声明
    void close();
    bool receiveGimbalData(GimbalData& data);  // 简易版（无超时反馈）
    bool receiveGimbalData(GimbalData& data, bool& is_timeout);  // 完整版（带超时反馈）
    bool sendGimbalData(const GimbalData& data);
};
