#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <string>

namespace Camera
{

/**
 * @brief 相机参数结构体
 */
struct CameraParams
{
  double exposure_time;  // 曝光时间 (us)
  double gain;           // 增益
  bool autocap;          // 自动采集模式
  double frame_rate;     // 帧率
  std::string frame_id;
  std::string camera_name;
  int interface_type;  // 接口类型 (华睿相机用: 0=All, 1=GigE, 2=USB3)
};

/**
 * @brief 相机状态枚举
 */
enum class CameraState : uint8_t
{
  STOPPED,
  RUNNING
};

/**
 * @brief 相机基类
 */
class CameraBase
{
 public:
  using Ptr = std::shared_ptr<CameraBase>;
  using LogCallback = std::function<void(const std::string&, int)>;  // message, level

  CameraBase() = default;
  virtual ~CameraBase() = default;

  // 禁止拷贝
  CameraBase(const CameraBase&) = delete;
  CameraBase& operator=(const CameraBase&) = delete;

  /**
   * @brief 初始化相机
   * @return true 初始化成功
   */
  virtual bool Initialize() = 0;

  /**
   * @brief 停止相机并释放资源
   */
  virtual void Stop() = 0;

  /**
   * @brief 读取一帧图像
   * @param[out] image 输出图像
   * @param[out] timestamp 时间戳（纳秒）
   * @return true 读取成功
   */
  virtual bool Read(cv::Mat& image, rclcpp::Time& timestamp_ns) = 0;

  /**
   * @brief 获取相机状态
   */
  virtual CameraState GetState() const = 0;

  /**
   * @brief 设置相机参数
   */
  virtual void SetParams(const CameraParams& params) = 0;

  /**
   * @brief 获取相机参数
   */
  virtual const CameraParams& GetParams() const = 0;

  /**
   * @brief 获取图像宽度
   */
  virtual int GetImageWidth() const = 0;

  /**
   * @brief 获取图像高度
   */
  virtual int GetImageHeight() const = 0;

  /**
   * @brief 设置浮点参数
   */
  virtual void SetFloatValue(const std::string& name, double value) = 0;

  /**
   * @brief 设置枚举参数（数值）
   */
  virtual void SetEnumValue(const std::string& name, unsigned int value) = 0;

  /**
   * @brief 设置日志回调
   */
  void SetLogCallback(LogCallback callback) { log_callback_ = std::move(callback); }

  /**
   * @brief 获取状态变化通知的条件变量
   */
  std::condition_variable& GetStateCondition() { return state_cv_; }

  /**
   * @brief 获取状态变化通知的互斥锁
   */
  std::mutex& GetStateMutex() { return state_mutex_; }

  // 日志级别
  enum LogLevel
  {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARN = 2,
    LOG_ERROR = 3
  };

 protected:
  void Log(const std::string& message, int level = LOG_INFO)
  {
    if (log_callback_)
    {
      log_callback_(message, level);
    }
  }

  LogCallback log_callback_;
  std::condition_variable state_cv_;
  std::mutex state_mutex_;

  rclcpp::Node* node_{nullptr};
};

}  // namespace Camera
