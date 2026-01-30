#include "serial_port.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::init(const std::string& port_name, uint32_t baudrate) {
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        std::cerr << "Failed to open serial port: " << port_name << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // 8N1模式：8位数据位，无校验，1位停止位
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器状态
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    options.c_oflag &= ~OPOST;

    // 设置超时
    options.c_cc[VTIME] = 10; // 读取超时1秒
    options.c_cc[VMIN] = 8;   // 最小读取字节数（假设数据帧为8字节：yaw(4)+pitch(4)）

    tcsetattr(serial_fd_, TCSANOW, &options);
    tcflush(serial_fd_, TCIFLUSH);
    return true;
}

void SerialPort::close() {
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool SerialPort::receiveGimbalData(GimbalData& data) {
    uint8_t buffer[8];
    ssize_t len = read(serial_fd_, buffer, sizeof(buffer));
    if (len != sizeof(buffer)) {
        return false;
    }

    // 假设数据为小端序float类型，根据云台协议调整
    memcpy(&data.yaw, buffer, 4);
    memcpy(&data.pitch, buffer + 4, 4);
    return true;
}

bool SerialPort::sendGimbalData(const GimbalData& data) {
    uint8_t buffer[8];
    memcpy(buffer, &data.yaw, 4);
    memcpy(buffer + 4, &data.pitch, 4);

    ssize_t len = write(serial_fd_, buffer, sizeof(buffer));
    return len == sizeof(buffer);
}