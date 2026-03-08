#include "serial_port.h"  // 对应你的头文件
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

// 协议常量（适配double类型：3个double=24字节）
const uint8_t FRAME_HEAD[] = {0xAA, 0x55};  // 帧头
const uint8_t FRAME_TAIL[] = {0x0D, 0x0A};  // 帧尾
const uint8_t DATA_LEN = 24;                // 3个double × 8字节 = 24字节

namespace hitcrt {
namespace serial {

// 计算异或校验和
static uint8_t calculate_checksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum ^= byte;
    }
    return checksum;
}

// 私double转8字节小端序
static std::vector<uint8_t> double_to_bytes(double value) {
    std::vector<uint8_t> bytes(8);
    uint8_t* p = reinterpret_cast<uint8_t*>(&value);
    for (int i = 0; i < 8; i++) {
        bytes[i] = p[i];  // 小端序（x86/STM32默认）
    }
    return bytes;
}

// 8字节小端序转double
static double bytes_to_double(const std::vector<uint8_t>& bytes) {
    if (bytes.size() != 8) return 0.0;
    double value;
    uint8_t* p = reinterpret_cast<uint8_t*>(&value);
    for (int i = 0; i < 8; i++) {
        p[i] = bytes[i];
    }
    return value;
}

// 封装角度为协议帧
static std::vector<uint8_t> pack_frame(const std::vector<double>& angles) {
    std::vector<uint8_t> frame;

    // 1. 帧头（2字节）
    frame.insert(frame.end(), FRAME_HEAD, FRAME_HEAD + 2);
    // 2. 数据长度（1字节）
    frame.push_back(DATA_LEN);
    // 3. 数据段：3个double（24字节）
    auto b1 = double_to_bytes(angles[0]);
    auto b2 = double_to_bytes(angles[1]);
    auto b3 = double_to_bytes(angles[2]);
    frame.insert(frame.end(), b1.begin(), b1.end());
    frame.insert(frame.end(), b2.begin(), b2.end());
    frame.insert(frame.end(), b3.begin(), b3.end());
    // 4. 校验和（1字节：帧头+长度+数据段）
    std::vector<uint8_t> check_data(frame.begin(), frame.end());
    frame.push_back(calculate_checksum(check_data));
    // 5. 帧尾（2字节）
    frame.insert(frame.end(), FRAME_TAIL, FRAME_TAIL + 2);

    return frame;
}

// 解析协议帧为角度
static bool unpack_frame(const std::vector<uint8_t>& frame, std::vector<double>& angles) {
    // 校验帧长度：2+1+24+1+2=30字节
    if (frame.size() != 30) {
        std::cerr << "[Serial] Frame length error! Expected 30, got " << frame.size() << std::endl;
        return false;
    }

    // 校验帧头
    if (frame[0] != FRAME_HEAD[0] || frame[1] != FRAME_HEAD[1]) {
        std::cerr << "[Serial] Frame head error!" << std::endl;
        return false;
    }

    // 校验数据长度
    if (frame[2] != DATA_LEN) {
        std::cerr << "[Serial] Data length error! Expected 24, got " << (int)frame[2] << std::endl;
        return false;
    }

    // 校验帧尾
    if (frame[28] != FRAME_TAIL[0] || frame[29] != FRAME_TAIL[1]) {
        std::cerr << "[Serial] Frame tail error!" << std::endl;
        return false;
    }

    // 校验和
    std::vector<uint8_t> check_data(frame.begin(), frame.begin() + 27);
    uint8_t expected = calculate_checksum(check_data);
    uint8_t actual = frame[27];
    if (expected != actual) {
        std::cerr << "[Serial] Checksum error! Expected " << (int)expected 
                  << ", got " << (int)actual << std::endl;
        return false;
    }

    // 解析3个角度
    std::vector<uint8_t> b1(frame.begin() + 3, frame.begin() + 11);
    std::vector<uint8_t> b2(frame.begin() + 11, frame.begin() + 19);
    std::vector<uint8_t> b3(frame.begin() + 19, frame.begin() + 27);
    angles[0] = bytes_to_double(b1);
    angles[1] = bytes_to_double(b2);
    angles[2] = bytes_to_double(b3);

    return true;
}


SerialPort::SerialPort(const std::string& port, int timeout) 
    : port(port), timeout(timeout), fd(-1) {
    memset(&old_tio, 0, sizeof(old_tio));
}

SerialPort::~SerialPort() {
    close_port();
}

// 波特率固化115200，8N1，无流控，非阻塞模式，带超时机制
bool SerialPort::open_port() {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[Serial] Failed to open port: " << port << std::endl;
        return false;
    }

    // 保存旧配置
    if (tcgetattr(fd, &old_tio) < 0) {
        std::cerr << "[Serial] Failed to get old serial config!" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    // 配置新串口参数
    struct termios new_tio;
    memset(&new_tio, 0, sizeof(new_tio));
    new_tio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  
    new_tio.c_cflag &= ~PARENB;  
    new_tio.c_cflag &= ~CSTOPB; 
    new_tio.c_cflag &= ~CRTSCTS; 

    // 非规范模式（不处理回车/换行）
    new_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tio.c_oflag &= ~OPOST;

    // 设置超时
    new_tio.c_cc[VTIME] = timeout / 100;  // 单位：0.1秒
    new_tio.c_cc[VMIN] = 0;               // 最小读取字节数

    // 应用配置
    if (tcsetattr(fd, TCSANOW, &new_tio) < 0) {
        std::cerr << "[Serial] Failed to set serial config!" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    std::cout << "[Serial] Port " << port << " opened successfully (115200 8N1)" << std::endl;
    return true;
}

// 关闭串口（恢复旧配置）
void SerialPort::close_port() {
    if (fd >= 0) {
        // 恢复旧配置
        tcsetattr(fd, TCSANOW, &old_tio);
        // 关闭文件描述符
        close(fd);
        fd = -1;
        std::cout << "[Serial] Port " << port << " closed" << std::endl;
    }
}

// 接收当前电机角度
bool SerialPort::receive_motor_angles(std::vector<double>& angles) {
    if (fd < 0) {
        std::cerr << "[Serial] Port not opened!" << std::endl;
        return false;
    }

    std::vector<uint8_t> buffer;
    std::vector<uint8_t> frame;
    bool frame_started = false;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (true) {
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed > timeout) {
            std::cerr << "[Serial] Receive timeout (" << timeout << "ms)!" << std::endl;
            return false;
        }

        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n != 1) continue;

        buffer.push_back(byte);
        int buf_len = buffer.size();

        // 检测帧头（0xAA 0x55）
        if (buf_len >= 2 && !frame_started) {
            if (buffer[buf_len-2] == FRAME_HEAD[0] && buffer[buf_len-1] == FRAME_HEAD[1]) {
                frame_started = true;
                frame.clear();
                frame.push_back(buffer[buf_len-2]);
                frame.push_back(buffer[buf_len-1]);
                buffer.clear();
                continue;
            }
        }

        // 帧已开始，接收直到30字节
        if (frame_started) {
            frame.push_back(byte);
            if (frame.size() == 30) {
                // 解析帧
                return unpack_frame(frame, angles);
            }
        }
    }
}

// 发送目标电机角度
bool SerialPort::send_motor_angles(const std::vector<double>& angles) {
    if (fd < 0) {
        std::cerr << "[Serial] Port not opened!" << std::endl;
        return false;
    }

    // 封装为协议帧
    std::vector<uint8_t> frame = pack_frame(angles);
    // 发送帧
    ssize_t n = write(fd, frame.data(), frame.size());
    if (n != frame.size()) {
        std::cerr << "[Serial] Send failed! Wrote " << n << " bytes (expected " << frame.size() << ")" << std::endl;
        return false;
    }

    std::cout << "[Serial] Sent angles: " 
              << angles[0] << ", " 
              << angles[1] << ", " 
              << angles[2] << std::endl;
    return true;
}

} // namespace serial
} // namespace hitcrt