#include "camera_node/camera_node.hpp"

using namespace std::chrono_literals;

namespace Camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("camera_node", options)
{
  // 初始化参数
  InitializeParams();
  RCLCPP_INFO(this->get_logger(), "Parameters have been initialized.");

  // 加载相机驱动插件
  if (!LoadCameraPlugin())
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to load camera plugin!");
    throw std::runtime_error("Failed to load camera plugin");
  }
  RCLCPP_INFO(this->get_logger(), "Camera plugin loaded: %s", camera_type_.c_str());

  // 设置日志回调
  SetupLogCallback();

  // 创建 publisher
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw",
                                                         rmw_qos_profile_sensor_data);
  RCLCPP_INFO(this->get_logger(), "Camera publisher created.");

  // 初始化相机
  InitializeCamera();
  RCLCPP_INFO(this->get_logger(), "Camera initialized.");

  // 创建守护线程
  guard_.protect_thread = std::thread(&CameraNode::ProtectRunning, this);
  RCLCPP_INFO(this->get_logger(), "Guard thread created.");

  // 设置相机信息管理器
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, params_.camera_name);
  auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://camera_node/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url))
  {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                camera_info_url.c_str());
  }

  // 预分配图像消息空间
  int width = camera_->GetImageWidth();
  int height = camera_->GetImageHeight();
  if (width > 0 && height > 0)
  {
    image_msg_.data.reserve(static_cast<size_t>(height * width) * 3);
    image_msg_.height = height;
    image_msg_.width = width;
  }

  // 创建取流线程
  capture_thread_ = std::thread(&CameraNode::CaptureLoop, this);
}

CameraNode::~CameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Destroying CameraNode...");

  running_.store(false);

  // 通知守护线程退出
  guard_.is_quit.notify_all();
  if (camera_)
  {
    camera_->GetStateCondition().notify_all();
  }

  // 先停采集线程
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }

  // 关闭相机
  if (camera_)
  {
    camera_->Stop();
  }

  // 再停守护线程
  if (guard_.protect_thread.joinable())
  {
    guard_.protect_thread.join();
  }

  RCLCPP_INFO(this->get_logger(), "CameraNode destroyed.");
}

void CameraNode::InitializeParams()
{
  // 相机类型参数（用于选择插件）
  camera_type_ = this->declare_parameter<std::string>("camera_type", "hik_camera");

  // 相机参数
  params_.exposure_time = this->declare_parameter<double>("exposure_time", 1000.0);
  params_.gain = this->declare_parameter<double>("gain", 16.0);
  params_.autocap = this->declare_parameter<bool>("autocap", true);
  params_.frame_rate = this->declare_parameter<double>("frame_rate", 249.0);
  params_.frame_id =
      this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  params_.camera_name =
      this->declare_parameter<std::string>("camera_name", "narrow_stereo");
  params_.interface_type = this->declare_parameter<int>("interface_type", 0);

  // 图像旋转参数
  rotate_image_ = this->declare_parameter<bool>("rotate_image", false);
  rotation_code_ = this->declare_parameter<int>("rotation_code", -1);
}

bool CameraNode::LoadCameraPlugin()
{
  try
  {
    // 创建插件加载器
    camera_loader_ = std::make_unique<pluginlib::ClassLoader<CameraBase>>(
        "camera_node", "Camera::CameraBase");

    // 根据camera_type参数确定插件名称
    std::string plugin_name;
    if (camera_type_ == "hik_camera" || camera_type_ == "hik")
    {
      plugin_name = "HikCamera::HikCamera";
    }
    else if (camera_type_ == "huaray_camera" || camera_type_ == "huaray")
    {
      plugin_name = "HuarayCamera::HuarayCamera";
    }
    else
    {
      // 尝试直接使用camera_type作为插件名称
      plugin_name = camera_type_;
    }

    RCLCPP_INFO(this->get_logger(), "Loading camera plugin: %s", plugin_name.c_str());

    // 加载插件
    camera_ = camera_loader_->createSharedInstance(plugin_name);

    if (!camera_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera instance");
      return false;
    }

    // 设置参数
    camera_->SetParams(params_);

    return true;
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load camera plugin: %s", ex.what());
    return false;
  }
}

void CameraNode::SetupLogCallback()
{
  camera_->SetLogCallback(
      [this](const std::string& message, int level)
      {
        switch (level)
        {
          case CameraBase::LOG_DEBUG:
            RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
            break;
          case CameraBase::LOG_INFO:
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            break;
          case CameraBase::LOG_WARN:
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
            break;
          case CameraBase::LOG_ERROR:
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            break;
          default:
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            break;
        }
      });
}

void CameraNode::InitializeCamera()
{
  if (!running_.load())
  {
    return;
  }

  if (!camera_->Initialize())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
  }
}

void CameraNode::ProtectRunning()
{
  RCLCPP_INFO(this->get_logger(), "Protect thread started.");

  std::unique_lock<std::mutex> lock(camera_->GetStateMutex());
  while (running_.load())
  {
    // 等待条件变量
    camera_->GetStateCondition().wait(
        lock, [this]
        { return (camera_->GetState() == CameraState::STOPPED) || (!running_.load()); });

    if (!running_.load())
    {
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Camera stopped, attempting to restart...");
    camera_->Stop();
    // 简单延时防止频繁重启
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!running_.load())
    {
      break;
    }

    camera_->Initialize();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(this->get_logger(), "Protect thread exit.");
}

void CameraNode::CaptureLoop()
{
  RCLCPP_INFO(this->get_logger(), "Capture thread started.");

  while (running_.load())
  {
    if (camera_->GetState() == CameraState::STOPPED)
    {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    cv::Mat image;
    rclcpp::Time timestamp_ns;

    bool ok = camera_->Read(image, timestamp_ns);
    if (!ok || image.empty())
    {
      continue;
    }

    // 如果需要旋转图像
    if (rotate_image_ && rotation_code_ >= 0)
    {
      cv::rotate(image, image, rotation_code_);
    }

    // 将 cv::Mat 转成 sensor_msgs::msg::Image
    image_msg_.header.stamp = timestamp_ns;
    image_msg_.header.frame_id = params_.frame_id;
    image_msg_.encoding = "rgb8";
    image_msg_.is_bigendian = false;
    image_msg_.height = image.rows;
    image_msg_.width = image.cols;
    image_msg_.step = static_cast<uint32_t>(image.cols * image.channels());
    image_msg_.data.assign(image.datastart, image.dataend);

    camera_pub_.publish(image_msg_, camera_info_msg_);
  }

  RCLCPP_INFO(this->get_logger(), "Capture thread exit.");
}

}  // namespace Camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Camera::CameraNode)
