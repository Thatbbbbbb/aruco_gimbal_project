#include "serial_port.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <cerrno>
#include <cstdint>

// 协议固定参数（适配 float 类型）
const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x5095, 0x411c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

constexpr uint16_t CRC16_INIT = 0xffff;
const uint8_t FRAME_HEAD[2] = {0x55, 0x00};  // 2字节帧头
const uint8_t float_BYTES = 4;                // float占4字节（核心修改）
const uint8_t DATA_BYTES = 3 * float_BYTES;   // 3个float占12字节
const uint16_t FRAME_TOTAL_LEN = 2 + DATA_BYTES + 2; // 总长度16字节（2+12+2）

namespace hitcrt {
namespace serial {

// CRC16-Modbus校验函数（不变）
uint16_t get_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc16 = CRC16_INIT;
  uint8_t byte;
  uint8_t i;

  while (len--) {
    byte = *data++;
    i = (crc16 ^ byte) & 0x00ff;
    crc16 = (crc16 >> 8) ^ CRC16_TABLE[i];
  }

  return crc16;
}

bool check_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc16 = (data[len - 1] << 8) | data[len - 2];
  return get_crc16(data, len - 2) == crc16;
}

// 核心修改：float转4字节小端序
static void float_to_bytes(float value, uint8_t* bytes) {
    uint8_t* p = reinterpret_cast<uint8_t*>(&value);
    // 小端序：低位在前
    bytes[0] = p[0];
    bytes[1] = p[1];
    bytes[2] = p[2];
    bytes[3] = p[3];
}

// 核心修改：4字节转float
static float bytes_to_float(const uint8_t* bytes) {
    float value;
    uint8_t* p = reinterpret_cast<uint8_t*>(&value);
    p[0] = bytes[0];
    p[1] = bytes[1];
    p[2] = bytes[2];
    p[3] = bytes[3];
    return value;
}

// 封装协议帧（适配float）
static bool pack_frame(const std::vector<float>& angles, uint8_t* frame, uint16_t& frame_len) {
    if (angles.size() != 3 || frame == nullptr) {
        std::cerr << "[Protocol] Invalid angles count or frame buffer!" << std::endl;
        return false;
    }

    frame_len = FRAME_TOTAL_LEN;
    uint8_t idx = 0;

    // 1. 写入2字节帧头
    frame[idx++] = FRAME_HEAD[0];
    frame[idx++] = FRAME_HEAD[1];

    // 2. 写入12字节数据（3个float）
    uint8_t float_bytes[float_BYTES];
    float_to_bytes(angles[0], float_bytes);
    memcpy(&frame[idx], float_bytes, float_BYTES); idx += float_BYTES;
    float_to_bytes(angles[1], float_bytes);
    memcpy(&frame[idx], float_bytes, float_BYTES); idx += float_BYTES;
    float_to_bytes(angles[2], float_bytes);
    memcpy(&frame[idx], float_bytes, float_BYTES); idx += float_BYTES;

    // 3. 计算并写入2字节CRC
    uint16_t crc_value = get_crc16(frame, FRAME_TOTAL_LEN - 2);
    frame[idx++] = static_cast<uint8_t>(crc_value & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc_value >> 8) & 0xFF);

    return true;
}

// 解析协议帧（适配float）
static bool unpack_frame(const uint8_t* frame, std::vector<float>& angles) {
    if (frame == nullptr) {
        std::cerr << "[Protocol] Null frame buffer!" << std::endl;
        return false;
    }

    // 1. 校验总长度（必须16字节）
    if (FRAME_TOTAL_LEN != 16) {
        std::cerr << "[Protocol] Frame length error! Expected 16, got " << FRAME_TOTAL_LEN << std::endl;
        return false;
    }

    // 2. 校验帧头
    if (frame[0] != FRAME_HEAD[0] || frame[1] != FRAME_HEAD[1]) {
        std::cerr << "[Protocol] Frame head error! Got 0x" << std::hex << (int)frame[0] 
                  << " 0x" << (int)frame[1] << std::dec << std::endl;
        return false;
    }

    // 3. 校验CRC
    uint16_t frame_crc = static_cast<uint16_t>(frame[14]) | (static_cast<uint16_t>(frame[15]) << 8);
    uint16_t calc_crc = get_crc16(frame, 14);
    if (frame_crc != calc_crc) {
        std::cerr << "[Protocol] CRC check failed! Frame CRC: 0x" << std::hex << frame_crc
                  << ", Calculated CRC: 0x" << calc_crc << std::dec << std::endl;
        return false;
    }

    // 4. 解析3个float角度
    angles.clear();
    angles.push_back(bytes_to_float(&frame[2]));   // 2-5字节：第一个float
    angles.push_back(bytes_to_float(&frame[6]));   // 6-9字节：第二个float
    angles.push_back(bytes_to_float(&frame[10]));  // 10-13字节：第三个float

    return true;
}

// 构造函数（timeout_ms：接收超时时间，毫秒）
SerialPort::SerialPort(const std::string& port, int timeout_ms) 
    : port(port), timeout(timeout_ms), fd(-1) {
    memset(&old_tio, 0, sizeof(old_tio));
}

// 析构函数
SerialPort::~SerialPort() {
    close_port();
}

// 打开串口（921600 8N1 非阻塞）
bool SerialPort::open_port() {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[Serial] Open port failed: " << port << " (" << strerror(errno) << ")" << std::endl;
        return false;
    }

    if (tcgetattr(fd, &old_tio) < 0) {
        std::cerr << "[Serial] Get old config failed! (" << strerror(errno) << ")" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    struct termios new_tio;
    memset(&new_tio, 0, sizeof(new_tio));
    new_tio.c_cflag = B921600 | CS8 | CLOCAL | CREAD; // 波特率根据硬件调整（如115200）
    new_tio.c_cflag &= ~PARENB;  // 无校验位
    new_tio.c_cflag &= ~CSTOPB;  // 1位停止位
    new_tio.c_cflag &= ~CRTSCTS; // 禁用硬件流控

    // 非规范模式
    new_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tio.c_oflag &= ~OPOST;

    // 接收超时设置
    new_tio.c_cc[VTIME] = timeout / 100; // 单位：0.1秒
    new_tio.c_cc[VMIN] = 0;              // 最小读取字节数为0

    // 清空缓冲区并应用配置
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &new_tio) < 0) {
        std::cerr << "[Serial] Set new config failed! (" << strerror(errno) << ")" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    std::cout << "[Serial] Port " << port << " opened (921600 8N1)" << std::endl;
    return true;
}

// 关闭串口
void SerialPort::close_port() {
    if (fd >= 0) {
        tcsetattr(fd, TCSANOW, &old_tio);
        close(fd);
        fd = -1;
        std::cout << "[Serial] Port " << port << " closed" << std::endl;
    }
}

// 发送float类型角度数据
bool SerialPort::send_motor_angles(const std::vector<float>& angles) {
    std::cout << "[Serial] Debug - Starting send_motor_angles" << std::endl;
    std::cout << "[Serial] Debug - fd = " << fd << std::endl;
    
    if (fd < 0) {
        std::cerr << "[Serial] Port not opened!" << std::endl;
        return false;
    }
    
    if (angles.size() != 3) {
        std::cerr << "[Serial] Angles count error! Need 3, got " << angles.size() << std::endl;
        return false;
    }

    std::cout << "[Serial] Debug - Angles: " << angles[0] << ", " << angles[1] << ", " << angles[2] << std::endl;

    uint8_t frame[FRAME_TOTAL_LEN] = {0};
    uint16_t frame_len = 0;
    
    if (!pack_frame(angles, frame, frame_len)) {
        std::cerr << "[Serial] pack_frame failed!" << std::endl;
        return false;
    }

    std::cout << "[Serial] Debug - Frame length: " << frame_len << std::endl;
    std::cout << "[Serial] Debug - Frame hex: ";
    for (int i = 0; i < frame_len; i++) {
        printf("%02X ", frame[i]);
    }
    std::cout << std::endl;

    ssize_t sent = write(fd, frame, frame_len);
    std::cout << "[Serial] Debug - write returned: " << sent << std::endl;
    
    if (sent != frame_len) {
        std::cerr << "[Serial] Send failed! Sent " << sent << " bytes (expected " << frame_len << ")" << std::endl;
        if (sent == -1) {
            std::cerr << "[Serial] Error details: " << strerror(errno) << std::endl;
        }
        return false;
    }

    std::cout << "[Serial] Sent angles (float): " << angles[0] << ", " << angles[1] << ", " << angles[2] << std::endl;
    return true;
}
// 接收float类型角度数据（修复帧头检测逻辑）
bool SerialPort::receive_motor_angles(std::vector<float>& angles) {
    if (fd < 0) {
        std::cerr << "[Serial] Port not opened!" << std::endl;
        return false;
    }

    uint8_t frame_buffer[FRAME_TOTAL_LEN] = {0};
    int recv_idx = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (true) {
        // 检查超时
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed > timeout) {
            std::cerr << "[Serial] Receive timeout (" << timeout << "ms)!" << std::endl;
            return false;
        }

        // 逐字节读取
        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n != 1) {
            usleep(500);
            continue;
        }

        // 修复后的帧头检测逻辑
        if (recv_idx == 0) {
            // 检测第一个帧头字节 0x55
            if (byte == FRAME_HEAD[0]) {
                frame_buffer[recv_idx++] = byte;
            }
        } else if (recv_idx == 1) {
            // 检测第二个帧头字节 0x00
            if (byte == FRAME_HEAD[1]) {
                frame_buffer[recv_idx++] = byte;
            } else {
                recv_idx = 0; // 不匹配则重置
            }
        } else {
            // 接收剩余数据
            frame_buffer[recv_idx++] = byte;

            // 接收满16字节，解析
            if (recv_idx == FRAME_TOTAL_LEN) {
                bool ret = unpack_frame(frame_buffer, angles);
                if (ret) {
                    std::cout << "[Serial] Received angles (float): " << angles[0] << ", " << angles[1] << ", " << angles[2] << std::endl;
                }
                return ret;
            }
        }
    }
}

} // namespace serial
} // namespace hitcrt