#pragma once
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <cstdint>
#include <chrono>

// 云台数据结构体：接收的角度和发送的角度
struct GimbalData {
    float yaw = 0.0f;    // 偏航角，单位度
    float pitch = 0.0f;  // 俯仰角，单位度
};

// 串口配置（新增：容错相关参数）
struct SerialConfig {
    uint32_t baudrate = 115200;
    int retry_times = 3;          // 重传次数
    int timeout_ms = 1000;       // 单次通信超时（ms）
    bool crc_check = true;       // 是否开启CRC校验
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    // 初始化串口：端口名 + 配置（新增SerialConfig）
    bool init(const std::string& port_name, const SerialConfig& config);

    // 关闭串口
    void close();

    // 接收云台当前角度（新增：帧同步+校验+重传）
    // return：是否成功；is_timeout：是否超时（输出参数）
    bool receiveGimbalData(GimbalData& data, bool& is_timeout);

    // 发送目标角度给云台（新增：校验+重传）
    bool sendGimbalData(const GimbalData& data);

private:
    // 私有辅助函数
    // 1. CRC16校验计算（Modbus标准）
    uint16_t calculateCRC16(const uint8_t* data, uint16_t len);
    // 2. 帧同步：清空缓冲区并查找有效帧头
    bool syncFrameHeader();
    // 3. 打包帧数据（按新协议格式）
    void packFrame(const GimbalData& data, uint8_t* frame, uint16_t& frame_len);
    // 4. 解包帧数据（校验+解析）
    bool unpackFrame(const uint8_t* frame, uint16_t frame_len, GimbalData& data);

    int serial_fd_ = -1;                // 串口文件描述符
    SerialConfig serial_config_;        // 串口配置（容错参数）
    const uint8_t FRAME_HEAD[2] = {0xAA, 0x55};  // 帧头
    const uint8_t FRAME_TAIL[2] = {0x0D, 0x0A};  // 帧尾
    const uint16_t FRAME_TOTAL_LEN = 17;         // 总帧长
};

#endif