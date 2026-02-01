#pragma once

#include <opencv2/imgproc.hpp>
#include <unordered_map>

#include "IMVApi.h"
#include "IMVDefines.h"
#include "camera_node/camera_base.hpp"

namespace HuarayCamera
{

/**
 * @brief 华睿相机驱动实现类
 *
 * 继承自 CameraBase，实现华睿工业相机的具体操作。
 * 作为 pluginlib 插件供 camera_node 加载。
 */
class HuarayCameraDriver : public Camera::CameraBase
{
 public:
  HuarayCameraDriver(rclcpp::Node* node = nullptr);
  ~HuarayCameraDriver() override;

  // === 实现基类接口 ===
  bool Initialize() override;
  void Stop() override;
  bool Read(cv::Mat& image, rclcpp::Time& timestamp_ns) override;
  Camera::CameraState GetState() const override;
  void SetParams(const Camera::CameraParams& params) override;
  const Camera::CameraParams& GetParams() const override;
  int GetImageWidth() const override;
  int GetImageHeight() const override;
  void SetFloatValue(const std::string& name, double value) override;
  void SetEnumValue(const std::string& name, unsigned int value) override;

 private:
  /**
   * @brief 设置枚举参数（符号）
   */
  void SetEnumSymbol(const std::string& name, const std::string& value);

  // 参数
  Camera::CameraParams params_;

  // SDK 句柄
  IMV_HANDLE handle_{nullptr};

  // 图像尺寸
  int image_width_{0};
  int image_height_{0};

  // 相机状态标志
  std::atomic<bool> is_opened_{false};
  std::atomic<bool> is_grabbing_{false};

  // 状态
  std::atomic<Camera::CameraState> state_{Camera::CameraState::STOPPED};
};

}  // namespace HuarayCamera
