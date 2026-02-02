#pragma once

#include <opencv2/imgproc.hpp>
#include <unordered_map>

#include "CameraParams.h"
#include "MvCameraControl.h"
#include "camera_node/camera_base.hpp"

namespace HikCamera
{

/**
 * @brief HIK相机驱动实现类
 *
 * 继承自 CameraBase，实现HIK工业相机的具体操作。
 * 作为 pluginlib 插件供 camera_node 加载。
 */
class HikCamera : public Camera::CameraBase
{
 public:
  HikCamera(rclcpp::Node* node = nullptr);
  ~HikCamera() override;

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
  // 参数
  Camera::CameraParams params_;

  // SDK 句柄
  void* handle_{nullptr};

  // 图像信息
  MV_IMAGE_BASIC_INFO img_info_{};

  // 状态
  std::atomic<Camera::CameraState> state_{Camera::CameraState::STOPPED};
};

}  // namespace HikCamera
