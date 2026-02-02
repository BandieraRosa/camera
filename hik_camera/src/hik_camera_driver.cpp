#include "hik_camera/hik_camera_driver.hpp"

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

namespace HikCamera
{

HikCamera::HikCamera(rclcpp::Node* node) : Camera::CameraBase(node) {}

HikCamera::~HikCamera() { Stop(); }

bool HikCamera::Initialize()
{
  unsigned int ret{};
  MV_CC_DEVICE_INFO_LIST device_list{};

  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_EnumDevices failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  if (device_list.nDeviceNum == 0)
  {
    Log("Not found camera!", LOG_ERROR);
    return false;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_CreateHandle failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_OpenDevice failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  unsigned int n_image_node_num = 3;
  ret = MV_CC_SetImageNodeNum(handle_, n_image_node_num);
  if (MV_OK != ret)
  {
    std::ostringstream oss;
    oss << "MV_CC_SetImageNodeNum failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  if (!params_.autocap)
  {
    ret = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
    if (MV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Acquisition Mode to Continuous fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_ERROR);
      return false;
    }

    // 将触发模式设置为开启 (On)
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Mode to On fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_ERROR);
      return false;
    }

    // 设置触发源为外部硬件触发 (Line0)
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerSource", "Line0");
    if (MV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Source to Line0 fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_ERROR);
      return false;
    }

    // 设置触发激活方式为上升沿触发
    ret = MV_CC_SetEnumValueByString(handle_, "TriggerActivation", "RisingEdge");
    if (MV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Activation to RisingEdge fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_ERROR);
      return false;
    }
  }
  else
  {
    // 将触发模式设置为关闭 (Off)
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (MV_OK != ret)
    {
      std::ostringstream oss;
      oss << "Set Trigger Mode to Off fail! 0x" << std::hex << ret;
      Log(oss.str(), LOG_ERROR);
      return false;
    }
  }

  SetEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  SetEnumValue("GainAuto", MV_GAIN_MODE_OFF);
  SetFloatValue("ExposureTime", params_.exposure_time);
  SetFloatValue("Gain", params_.gain);

  ret = MV_CC_SetEnumValue(handle_, "ADCBitDepth", 2);
  if (MV_OK != ret)
  {
    std::ostringstream oss;
    oss << "Set ADC Bit Depth to 8 Bits fail! 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  // 帧率
  ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                            static_cast<float>(params_.frame_rate));
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_SetFloatValue(set framerate) failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  // 获取图像信息
  MV_CC_GetImageInfo(handle_, &img_info_);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_StartGrabbing failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
    return false;
  }

  state_.store(Camera::CameraState::RUNNING);
  Log("Hik camera initialized and started.", LOG_INFO);
  return true;
}

void HikCamera::Stop()
{
  state_.store(Camera::CameraState::STOPPED);

  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_StopGrabbing failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_CloseDevice failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_DestroyHandle failed: 0x" << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
  }

  handle_ = nullptr;
  Log("Hik camera stopped and handle destroyed.", LOG_INFO);
}

bool HikCamera::Read(cv::Mat& img, rclcpp::Time& timestamp_ns)
{
  if (state_.load() == Camera::CameraState::STOPPED)
  {
    return false;
  }

  MV_FRAME_OUT raw{};
  unsigned int ret{};
  unsigned int n_msec = 100;

  auto start = std::chrono::steady_clock::now();
  ret = MV_CC_GetImageBuffer(handle_, &raw, n_msec);

  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_GetImageBuffer failed: 0x" << std::hex << ret
        << ", switching to Stopped.";
    Log(oss.str(), LOG_ERROR);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start);
  if (duration_ns < std::chrono::nanoseconds(2'000'000))
  {
    MV_CC_FreeImageBuffer(handle_, &raw);
    return false;
  }

  timestamp_ns = node_->now();

  // 将原始 buffer 封装为 cv::Mat
  cv::Mat raw_img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U,
                  raw.pBufAddr);

  const auto& frame_info = raw.stFrameInfo;
  auto pixel_type = frame_info.enPixelType;

  static const std::unordered_map<MvGvspPixelType, int> TYPE_MAP = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}};

  auto it = TYPE_MAP.find(pixel_type);
  if (it == TYPE_MAP.end())
  {
    MV_CC_FreeImageBuffer(handle_, &raw);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  cv::Mat dst_image;
  cv::cvtColor(raw_img, dst_image, it->second);
  img = dst_image;

  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_FreeImageBuffer failed: 0x" << std::hex << ret
        << ", switching to Stopped.";
    Log(oss.str(), LOG_ERROR);
    state_.store(Camera::CameraState::STOPPED);
    state_cv_.notify_all();
    return false;
  }

  return true;
}

Camera::CameraState HikCamera::GetState() const { return state_.load(); }

void HikCamera::SetParams(const Camera::CameraParams& params) { params_ = params; }

const Camera::CameraParams& HikCamera::GetParams() const { return params_; }

int HikCamera::GetImageWidth() const { return static_cast<int>(img_info_.nWidthMax); }

int HikCamera::GetImageHeight() const { return static_cast<int>(img_info_.nHeightMax); }

void HikCamera::SetFloatValue(const std::string& name, double value)
{
  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret =
      MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_SetFloatValue(\"" << name << "\", " << value << ") failed: 0x"
        << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
  }
}

void HikCamera::SetEnumValue(const std::string& name, unsigned int value)
{
  if (handle_ == nullptr)
  {
    return;
  }

  unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
  {
    std::ostringstream oss;
    oss << "MV_CC_SetEnumValue(\"" << name << "\", " << value << ") failed: 0x"
        << std::hex << ret;
    Log(oss.str(), LOG_ERROR);
  }
}

}  // namespace HikCamera

// 导出插件
PLUGINLIB_EXPORT_CLASS(HikCamera::HikCamera, Camera::CameraBase)
