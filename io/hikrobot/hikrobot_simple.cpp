#include "hikrobot.hpp"
#include <libusb-1.0/libusb.h>
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{

HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");
    capture_start();
    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);
      if (capturing_) continue;
      capture_stop();
      reset_usb();
      capture_start();
    }
    capture_stop();
    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);
  img = data.img;
  timestamp = data.timestamp;
}

// 极简测试版本：不处理图像数据，只测试获取
void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");
    capturing_ = true;

    MV_FRAME_OUT raw;
    memset(&raw, 0, sizeof(MV_FRAME_OUT));

    while (!capture_quit_) {
      unsigned int ret;
      unsigned int nMsec = 200;

      tools::logger()->info("Getting image buffer...");
      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      
      if (ret != MV_OK) {
        if (ret != 0x80000001) {  // MV_E_USB_READ_DATA
          tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        }
        continue;
      }

      tools::logger()->info("Got frame: {}x{}", raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight);
      
      // 创建测试图像（黑色图像），不处理 raw 数据
      cv::Mat test_img(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));  // 红色图像
      auto timestamp = std::chrono::steady_clock::now();
      
      tools::logger()->info("Pushing test image to queue...");
      queue_.push({test_img.clone(), timestamp});
      tools::logger()->info("Push completed");

      // 释放 SDK 图像缓冲
      MV_CC_FreeImageBuffer(handle_, &raw);
      
      // 只处理一帧就退出循环（测试用）
      // break;
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  unsigned int ret;
  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }
  try {
    vid_ = std::stoi(vid_pid.substr(0, index), 0, 16);
    pid_ = std::stoi(vid_pid.substr(index + 1), 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }
  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");
  libusb_close(handle);
}

}  // namespace io
