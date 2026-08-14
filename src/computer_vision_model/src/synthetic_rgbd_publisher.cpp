#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

namespace computer_vision_model
{

class SyntheticRgbdPublisher : public rclcpp::Node
{
public:
  SyntheticRgbdPublisher()
  : Node("synthetic_rgbd_publisher")
  {
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<double>("fps", 10.0);
    max_frames_ = declare_parameter<int>("max_frames", 0);
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "camera_color_optical_frame");

    width_ = std::max(width_, 16);
    height_ = std::max(height_, 16);
    fps_ = std::max(fps_, 0.1);

    const auto qos = rclcpp::SensorDataQoS();
    color_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos);
    depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", qos);
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", qos);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / fps_)),
      [this]() {publish_frame();});
    RCLCPP_INFO(
      get_logger(),
      "Publishing synthetic aligned RGB-D at %dx%d, %.1f FPS",
      width_, height_, fps_);
  }

private:
  void publish_frame()
  {
    const auto stamp = now();

    sensor_msgs::msg::Image color;
    color.header.stamp = stamp;
    color.header.frame_id = frame_id_;
    color.height = static_cast<std::uint32_t>(height_);
    color.width = static_cast<std::uint32_t>(width_);
    color.encoding = sensor_msgs::image_encodings::BGR8;
    color.is_bigendian = false;
    color.step = static_cast<std::uint32_t>(width_ * 3);
    color.data.resize(static_cast<std::size_t>(height_ * width_ * 3));

    sensor_msgs::msg::Image depth;
    depth.header = color.header;
    depth.height = color.height;
    depth.width = color.width;
    depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth.is_bigendian = false;
    depth.step = static_cast<std::uint32_t>(width_ * sizeof(std::uint16_t));
    depth.data.resize(
      static_cast<std::size_t>(height_ * width_ * sizeof(std::uint16_t)));

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const std::size_t color_index =
          static_cast<std::size_t>((y * width_ + x) * 3);
        color.data[color_index] =
          static_cast<std::uint8_t>(255 * x / width_);
        color.data[color_index + 1] =
          static_cast<std::uint8_t>(255 * y / height_);
        color.data[color_index + 2] = 70;

        const double dx = x - width_ * 0.5;
        const double dy = y - height_ * 0.5;
        const bool fruit = dx * dx + dy * dy < 70.0 * 70.0;
        if (fruit) {
          color.data[color_index] = 20;
          color.data[color_index + 1] = 60;
          color.data[color_index + 2] = 230;
        }

        const std::uint16_t depth_mm =
          fruit ? 800 : static_cast<std::uint16_t>(1200 + x / 4);
        const std::size_t depth_index = static_cast<std::size_t>(
          (y * width_ + x) * sizeof(std::uint16_t));
        std::memcpy(
          depth.data.data() + depth_index, &depth_mm,
          sizeof(depth_mm));
      }
    }

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = color.header;
    camera_info.height = color.height;
    camera_info.width = color.width;
    const double fx = 525.0;
    const double fy = 525.0;
    const double cx = (width_ - 1) / 2.0;
    const double cy = (height_ - 1) / 2.0;
    camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_info.p =
    {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info.distortion_model = "plumb_bob";

    color_publisher_->publish(color);
    depth_publisher_->publish(depth);
    info_publisher_->publish(camera_info);

    ++published_frames_;
    if (max_frames_ > 0 && published_frames_ >= max_frames_) {
      RCLCPP_INFO(
        get_logger(), "Published requested %d frames; stopping",
        published_frames_);
      rclcpp::shutdown();
    }
  }

  int width_{640};
  int height_{480};
  double fps_{10.0};
  int max_frames_{0};
  int published_frames_{0};
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace computer_vision_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<computer_vision_model::SyntheticRgbdPublisher>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
