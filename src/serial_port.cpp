// serial_port.cpp
#include "serial_port.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <algorithm>

// 1. 实现全局CRC8函数（你的原始代码，保留不变）
uint8_t crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// 2. 实现SerialPort析构函数（仅保留1份，删除重复）
SerialPort::~SerialPort() {
    close();  // 调用类的close()释放资源
}

// 3. 实现私有成员函数：CRC16-MODBUS（保留不变）
uint16_t SerialPort::calculateCRC16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 4. 实现私有成员函数：帧同步（保留不变，解决宏未定义问题）
bool SerialPort::syncFrameHeader() {
    if (serial_fd_ < 0) return false;

    uint8_t buf[128];
    ssize_t read_len;
    // 循环读取直到找到帧头 0xAA 0x55
    while (true) {
        // 读取1字节，超时则退出
        read_len = read(serial_fd_, buf, 1);
        if (read_len <= 0) return false;

        if (buf[0] == 0xAA) {  // 直接使用帧头值，避免数组访问问题
            // 读下一字节验证是否是 0x55
            read_len = read(serial_fd_, buf+1, 1);
            if (read_len <= 0) return false;
            if (buf[1] == 0x55) {
                // 找到帧头，回退2字节（留给后续完整读取）
                lseek(serial_fd_, -2, SEEK_CUR);
                return true;
            }
        }
    }
}

// 5. 实现私有成员函数：打包帧数据（保留不变）
void SerialPort::packFrame(const GimbalData& data, uint8_t* frame, uint16_t& frame_len) {
    frame_len = FRAME_TOTAL_LEN;
    uint16_t crc = 0;

    // 1. 帧头
    frame[0] = 0xAA;
    frame[1] = 0x55;
    // 2. 数据长度
    frame[2] = 0x08;
    // 3. 有效数据（yaw + pitch，小端序）
    memcpy(frame+3, &data.yaw, 4);
    memcpy(frame+7, &data.pitch, 4);
    // 4. CRC16校验（帧头+长度+有效数据，共11字节）
    crc = calculateCRC16(frame, 11);  // 0-10字节（共11字节）
    frame[11] = crc & 0xFF;          // 低字节
    frame[12] = (crc >> 8) & 0xFF;   // 高字节
    // 5. 帧尾
    frame[13] = 0x55;
    frame[14] = 0xAA;
}

// 6. 实现私有成员函数：解包帧数据（保留不变）
bool SerialPort::unpackFrame(const uint8_t* frame, uint16_t frame_len, GimbalData& data) {
    // 1. 校验总长度
    if (frame_len != FRAME_TOTAL_LEN) return false;
    // 2. 校验帧头/帧尾
    if (frame[0] != 0xAA || frame[1] != 0x55 ||
        frame[13] != 0x55 || frame[14] != 0xAA) {
        return false;
    }
    // 3. 校验数据长度
    if (frame[2] != 0x08) return false;
    // 4. CRC校验（开启时）
    if (serial_config_.crc_check) {
        uint16_t crc_calc = calculateCRC16(frame, 11);
        uint16_t crc_recv = (frame[12] << 8) | frame[11];
        if (crc_calc != crc_recv) return false;
    }
    // 5. 解析有效数据
    memcpy(&data.yaw, frame+3, 4);
    memcpy(&data.pitch, frame+7, 4);
    return true;
}

// 7. 实现公有成员函数：简易版init（仅波特率，适配你的原始代码）
bool SerialPort::init(const std::string& port_name, uint32_t baudrate) {
    // 封装默认配置，调用完整版init
    SerialConfig config;
    config.baudrate = baudrate;
    return this->init(port_name, config);
}

// 8. 实现公有成员函数：完整版init（带SerialConfig，保留不变）
bool SerialPort::init(const std::string& port_name, const SerialConfig& config) {
    serial_config_ = config;
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open serial port: " << port_name << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // 设置波特率
    speed_t baud = B115200;
    switch (config.baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200;
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1模式：8位数据位，无校验，1位停止位
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器状态
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    options.c_oflag &= ~OPOST;

    // 超时配置（毫秒转0.1秒单位，避免除以0）
    options.c_cc[VTIME] = (config.timeout_ms / 100) > 0 ? (config.timeout_ms / 100) : 1;
    options.c_cc[VMIN] = 0;  // 取消最小字节数限制（由帧同步处理）

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIFLUSH); // 清空接收缓冲区（帧同步前置）
    return true;
}

// 9. 实现公有成员函数：close（仅保留1份，删除重复，注意::close()是系统函数）
void SerialPort::close() {
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);  // 加::表示调用全局系统函数，避免与类成员函数重名
        serial_fd_ = -1;
    }
}

// 10. 实现公有成员函数：简易版receiveGimbalData（无超时反馈，适配原始代码）
bool SerialPort::receiveGimbalData(GimbalData& data) {
    bool is_timeout = false;
    return this->receiveGimbalData(data, is_timeout);
}

// 11. 实现公有成员函数：完整版receiveGimbalData（带超时反馈，保留不变）
bool SerialPort::receiveGimbalData(GimbalData& data, bool& is_timeout) {
    is_timeout = false;
    uint8_t frame[FRAME_TOTAL_LEN];
    ssize_t read_len = 0;

    // 1. 帧同步：找到有效帧头（处理缓冲区残留）
    if (!syncFrameHeader()) {
        is_timeout = true;
        return false;
    }

    // 2. 循环重传接收（配置的重试次数）
    for (int retry = 0; retry < serial_config_.retry_times; retry++) {
        // 读取完整帧
        read_len = read(serial_fd_, frame, FRAME_TOTAL_LEN);
        if (read_len != FRAME_TOTAL_LEN) {
            tcflush(serial_fd_, TCIFLUSH); // 清空错误数据
            continue;
        }

        // 解包+校验
        if (unpackFrame(frame, FRAME_TOTAL_LEN, data)) {
            return true;
        } else {
            std::cerr << "Receive frame error, retry: " << retry+1 << "/" << serial_config_.retry_times << std::endl;
            tcflush(serial_fd_, TCIFLUSH);
            continue;
        }
    }

    is_timeout = true;
    return false;
}

// 12. 实现公有成员函数：sendGimbalData
bool SerialPort::sendGimbalData(const GimbalData& data) {
    uint8_t frame[FRAME_TOTAL_LEN];
    uint16_t frame_len = 0;
    packFrame(data, frame, frame_len);

    // 循环重传发送
    for (int retry = 0; retry < serial_config_.retry_times; retry++) {
        ssize_t write_len = write(serial_fd_, frame, frame_len);
        if (write_len == frame_len) {
            return true;
        } else {
            std::cerr << "Send frame error, retry: " << retry+1 << "/" << serial_config_.retry_times << std::endl;
            tcflush(serial_fd_, TCOFLUSH); // 清空发送缓冲区
            usleep(1000); // 短延时后重试
            continue;
        }
    }

    return false;
}
