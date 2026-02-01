#include "huaray_camera/huaray_camera_driver.hpp"

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

namespace HuarayCamera
{

HuarayCameraDriver::HuarayCameraDriver(rclcpp::Node* node) : Camera::CameraBase()
{
  node_ = node;
}

HuarayCameraDriver::~HuarayCameraDriver() { Stop(); }

bool HuarayCameraDriver::Initialize()
{
  int ret{};
  IMV_DeviceList device_list{};

  unsigned int interface_type = interfaceTypeAll;
  switch (params_.interface_type)
  {
    case 1:
      interface_type = interfaceTypeGige;
      Log("Enumerating GigE devices...", LOG_INFO);
      break;
    case 2:
      interface_type = interfaceTypeUsb3;
      Log("Enumerating USB3 devices...", LOG_INFO);
      break;
    default:
      interface_type = interfaceTypeAll;
      Log("Enumerating all interface types (GigE + USB3)...", LOG_INFO);
      break;
  }

  ret = IMV_EnumDevices(&device_list, interface_type);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_EnumDevices failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  {
    std::ostringstream oss;
    oss << "Found " << device_list.nDevNum << " device(s).";
    Log(oss.str(), LOG_INFO);
  }

  if (device_list.nDevNum == 0)
  {
    Log("Not found camera! Please check:", LOG_ERROR);
    Log("  1. Camera is connected and powered on", LOG_ERROR);
    Log("  2. Camera driver is installed properly", LOG_ERROR);
    Log("  3. For GigE: Check network settings and IP config", LOG_ERROR);
    Log("  4. For USB3: Check USB connection and permissions", LOG_ERROR);
    return false;
  }

  // 打印设备信息便于调试
  for (unsigned int i = 0; i < device_list.nDevNum; i++)
  {
    IMV_DeviceInfo* dev_info = &device_list.pDevInfo[i];
    std::ostringstream oss;
    oss << "Device[" << i << "]: Key=" << dev_info->cameraKey
        << ", Type=" << dev_info->nCameraType;
    Log(oss.str(), LOG_INFO);
  }

  // 使用设备键创建句柄（使用第一个设备）
  unsigned int device_index = 0;
  if (device_list.nDevNum > 1)
  {
    device_index = 1;  // 如果有多个设备，使用第二个（保持原有逻辑）
  }
  const char* camera_key = device_list.pDevInfo[device_index].cameraKey;
  ret = IMV_CreateHandle(&handle_, modeByCameraKey, (void*)camera_key);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_CreateHandle failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    handle_ = nullptr;
    return false;
  }
  Log("Handle created successfully.", LOG_INFO);

  ret = IMV_Open(handle_);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_Open failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    Log("Possible causes:", LOG_ERROR);
    Log("  1. Camera is already opened by another application", LOG_ERROR);
    Log("  2. Network/USB connection issue", LOG_ERROR);
    Log("  3. Insufficient permissions", LOG_ERROR);
    IMV_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  is_opened_.store(true);
  Log("Camera opened successfully.", LOG_INFO);

  // 设置缓冲区数量
  unsigned int n_image_node_num = 3;
  ret = IMV_SetBufferCount(handle_, n_image_node_num);
  if (IMV_OK != ret)
  {
    std::ostringstream oss;
    oss << "IMV_SetBufferCount failed: 0x" << std::hex << ret << " (non-fatal)";
    Log(oss.str(), LOG_WARN);
  }

  if (!params_.autocap)
  {
    ret = IMV_SetEnumFeatureSymbol(handle_, "AcquisitionMode", "Continuous");
    if (IMV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Acquisition Mode to Continuous fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }

    // 将触发模式设置为开启 (On)
    ret = IMV_SetEnumFeatureValue(handle_, "TriggerMode", 1);
    if (IMV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Mode to On fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }

    // 设置触发源为外部硬件触发 (Line0)
    ret = IMV_SetEnumFeatureSymbol(handle_, "TriggerSource", "Line0");
    if (IMV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Source to Line0 fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }

    // 设置触发激活方式为上升沿触发
    ret = IMV_SetEnumFeatureSymbol(handle_, "TriggerActivation", "RisingEdge");
    if (IMV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Activation to RisingEdge fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }
  }
  else
  {
    // 将触发模式设置为关闭 (Off)
    ret = IMV_SetEnumFeatureValue(handle_, "TriggerMode", 0);
    if (IMV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Mode to Off fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }
  }

  // 曝光、增益、白平衡等
  SetEnumSymbol("BalanceWhiteAuto", "Continuous");
  SetEnumSymbol("ExposureAuto", "Off");
  SetEnumSymbol("GainAuto", "Off");
  SetFloatValue("ExposureTime", params_.exposure_time);
  SetFloatValue("Gain", params_.gain);

  // 帧率
  ret = IMV_SetDoubleFeatureValue(handle_, "AcquisitionFrameRate", params_.frame_rate);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_SetDoubleFeatureValue(set framerate) failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  ret = IMV_StartGrabbing(handle_);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_StartGrabbing failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    IMV_Close(handle_);
    is_opened_.store(false);
    IMV_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  is_grabbing_.store(true);

  state_.store(Camera::CameraState::RUNNING);
  Log("Huaray camera initialized and started successfully.", LOG_INFO);
  return true;
}

void HuarayCameraDriver::Stop()
{
  state_.store(Camera::CameraState::STOPPED);

  if (handle_ == nullptr)
  {
    Log("CaptureStop: handle is null, nothing to do.", LOG_DEBUG);
    return;
  }

  int ret{};

  // 只有在正在采集时才停止采集
  if (is_grabbing_.load())
  {
    ret = IMV_StopGrabbing(handle_);
    if (ret != IMV_OK)
    {
      std::ostringstream oss;
      oss << "IMV_StopGrabbing failed: 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }
    else
    {
      Log("IMV_StopGrabbing succeeded.", LOG_DEBUG);
    }
    is_grabbing_.store(false);
  }

  // 只有在相机已打开时才关闭
  if (is_opened_.load())
  {
    ret = IMV_Close(handle_);
    if (ret != IMV_OK)
    {
      std::ostringstream oss;
      oss << "IMV_Close failed: 0x" << std::hex << ret;
      Log(oss.str(), LOG_WARN);
    }
    else
    {
      Log("IMV_Close succeeded.", LOG_DEBUG);
    }
    is_opened_.store(false);
  }

  // 销毁句柄
  ret = IMV_DestroyHandle(handle_);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_DestroyHandle failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_WARN);
  }
  else
  {
    Log("IMV_DestroyHandle succeeded.", LOG_DEBUG);
  }

  handle_ = nullptr;
  Log("Huaray camera stopped and handle destroyed.", LOG_INFO);
}

bool HuarayCameraDriver::Read(cv::Mat& img, rclcpp::Time& timestamp_ns)
{
  if (state_.load() == Camera::CameraState::STOPPED || handle_ == nullptr ||
      !is_grabbing_.load())
  {
    return false;
  }

  IMV_Frame frame{};
  int ret{};
  unsigned int n_msec = 100;

  auto start = std::chrono::steady_clock::now();
  ret = IMV_GetFrame(handle_, &frame, n_msec);

  if (ret != IMV_OK)
  {
    // 超时不算严重错误
    if (ret == IMV_TIMEOUT)
    {
      return false;
    }
    std::ostringstream oss;
    oss << "IMV_GetFrame failed: 0x" << std::hex << ret << ", switching to Stopped.";
    Log(oss.str(), LOG_ERROR);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start);
  if (duration_ns < std::chrono::nanoseconds(2'000'000))
  {
    IMV_ReleaseFrame(handle_, &frame);
    return false;
  }

  timestamp_ns = node_->now();

  // 更新图像尺寸
  image_width_ = static_cast<int>(frame.frameInfo.width);
  image_height_ = static_cast<int>(frame.frameInfo.height);

  // 将原始 buffer 封装为 cv::Mat
  cv::Mat raw_img(cv::Size(static_cast<int>(frame.frameInfo.width),
                           static_cast<int>(frame.frameInfo.height)),
                  CV_8U, frame.pData);

  const auto& frame_info = frame.frameInfo;
  auto pixel_type = frame_info.pixelFormat;

  static const std::unordered_map<IMV_EPixelType, int> TYPE_MAP = {
      {gvspPixelBayGR8, cv::COLOR_BayerGR2BGR},
      {gvspPixelBayRG8, cv::COLOR_BayerRG2BGR},
      {gvspPixelBayGB8, cv::COLOR_BayerGB2BGR},
      {gvspPixelBayBG8, cv::COLOR_BayerBG2BGR}};

  auto it = TYPE_MAP.find(pixel_type);
  if (it == TYPE_MAP.end())
  {
    std::ostringstream oss;
    oss << "Unsupported pixel format: 0x" << std::hex << pixel_type;
    Log(oss.str(), LOG_WARN);
    IMV_ReleaseFrame(handle_, &frame);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  cv::Mat dst_image;
  cv::cvtColor(raw_img, dst_image, it->second);
  img = dst_image;

  ret = IMV_ReleaseFrame(handle_, &frame);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_ReleaseFrame failed: 0x" << std::hex << ret << ", switching to Stopped.";
    Log(oss.str(), LOG_ERROR);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  return true;
}

Camera::CameraState HuarayCameraDriver::GetState() const { return state_.load(); }

void HuarayCameraDriver::SetParams(const Camera::CameraParams& params)
{
  params_ = params;
}

const Camera::CameraParams& HuarayCameraDriver::GetParams() const { return params_; }

int HuarayCameraDriver::GetImageWidth() const { return image_width_; }

int HuarayCameraDriver::GetImageHeight() const { return image_height_; }

void HuarayCameraDriver::SetFloatValue(const std::string& name, double value)
{
  if (handle_ == nullptr || !is_opened_.load())
  {
    return;
  }

  int ret = IMV_SetDoubleFeatureValue(handle_, name.c_str(), value);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_SetDoubleFeatureValue(\"" << name << "\", " << value << ") failed: 0x"
        << std::hex << ret;
    Log(oss.str(), LOG_WARN);
  }
}

void HuarayCameraDriver::SetEnumValue(const std::string& name, unsigned int value)
{
  if (handle_ == nullptr || !is_opened_.load())
  {
    return;
  }

  int ret = IMV_SetEnumFeatureValue(handle_, name.c_str(), value);
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_SetEnumFeatureValue(\"" << name << "\", " << value << ") failed: 0x"
        << std::hex << ret;
    Log(oss.str(), LOG_WARN);
  }
}

void HuarayCameraDriver::SetEnumSymbol(const std::string& name, const std::string& value)
{
  if (handle_ == nullptr || !is_opened_.load())
  {
    return;
  }

  int ret = IMV_SetEnumFeatureSymbol(handle_, name.c_str(), value.c_str());
  if (ret != IMV_OK)
  {
    std::ostringstream oss;
    oss << "IMV_SetEnumFeatureSymbol(\"" << name << "\", \"" << value << "\") failed: 0x"
        << std::hex << ret;
    Log(oss.str(), LOG_WARN);
  }
}

}  // namespace HuarayCamera

// 导出插件
PLUGINLIB_EXPORT_CLASS(HuarayCamera::HuarayCameraDriver, Camera::CameraBase)
