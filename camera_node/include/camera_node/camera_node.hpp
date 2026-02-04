#pragma once

#include <opencv2/imgproc.hpp>
#include <pluginlib/class_loader.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "camera_node/camera_base.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace Camera
{

/**
 * @brief 通用相机节点
 *
 * 该节点通过 pluginlib 动态加载相机驱动插件，
 * 负责ROS2话题发布、参数管理和生命周期管理。
 */
class CameraNode : public rclcpp::Node
{
 public:
  explicit CameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CameraNode() override;

 private:
  /**
   * @brief 守护线程保护结构
   */
  struct ProtectGuard
  {
    std::mutex mux;
    std::condition_variable is_quit;
    std::thread protect_thread;
  };

  /**
   * @brief 初始化参数
   */
  void InitializeParams();

  /**
   * @brief 加载相机驱动插件
   */
  bool LoadCameraPlugin();

  /**
   * @brief 根据camera_type匹配插件
   * @param camera_type 相机类型参数（包名、厂商名或完整类名）
   * @param available_plugins 可用插件列表
   * @return 匹配的完整类名，未匹配返回空字符串
   */
  std::string FindMatchingPlugin(const std::string& camera_type,
                                const std::vector<std::string>& available_plugins);

  /**
   * @brief 格式化插件列表用于错误提示
   * @param plugins 插件列表
   * @return 格式化后的字符串
   */
  std::string FormatPluginList(const std::vector<std::string>& plugins);

  /**
   * @brief 设置日志回调
   */
  void SetupLogCallback();

  /**
   * @brief 初始化相机
   */
  void InitializeCamera();

  /**
   * @brief 守护线程函数
   */
  void ProtectRunning();

  /**
   * @brief 采集线程函数
   */
  void CaptureLoop();

  // === 成员变量 ===

  // 插件加载器
  std::unique_ptr<pluginlib::ClassLoader<CameraBase>> camera_loader_;

  // 相机实例
  CameraBase::Ptr camera_;

  // 相机参数
  CameraParams params_;

  // 相机类型参数
  std::string camera_type_;

  // 是否需要旋转图像
  bool rotate_image_{false};
  int rotation_code_{-1};

  // 相机信息管理器
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  // ROS2消息
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // 运行状态
  std::atomic<bool> running_{true};

  // 线程
  std::thread capture_thread_;
  ProtectGuard guard_;

  // ROS2 publisher
  image_transport::CameraPublisher camera_pub_;
};

}  // namespace Camera
