#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace computer_vision_model
{

class RgbdCaptureNode : public rclcpp::Node
{
public:
  RgbdCaptureNode()
  : Node("rgbd_capture_node")
  {
    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    output_directory_ = declare_parameter<std::string>(
      "output_directory", "/tmp/computer_vision_captures");
    require_depth_ = declare_parameter<bool>("require_depth", true);
    maximum_rgb_age_ms_ = declare_parameter<double>("maximum_rgb_age_ms", 0.0);
    capture_on_start_ = declare_parameter<bool>("capture_on_start", true);
    exit_after_capture_ =
      declare_parameter<bool>("exit_after_capture", false);
    require_matching_dimensions_ =
      declare_parameter<bool>("require_matching_dimensions", true);
    maximum_timestamp_delta_ms_ =
      declare_parameter<double>("maximum_timestamp_delta_ms", 100.0);
    depth_unit_m_per_value_ =
      declare_parameter<double>("depth_unit_m_per_value", 0.001);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 95);
    diagnostic_roi_radius_px_ =
      declare_parameter<int>("diagnostic_roi_radius_px", 10);

    jpeg_quality_ = std::clamp(jpeg_quality_, 1, 100);
    diagnostic_roi_radius_px_ = std::max(diagnostic_roi_radius_px_, 0);
    capture_pending_ = capture_on_start_;

    const auto qos = rclcpp::SensorDataQoS();
    rgb_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_rgb_ = std::move(msg);
        }
        attempt_automatic_capture();
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_depth_ = std::move(msg);
        }
        attempt_automatic_capture();
      });
    camera_info_subscription_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_camera_info_ = std::move(msg);
        }
        attempt_automatic_capture();
      });

    capture_service_ = create_service<std_srvs::srv::Trigger>(
      "/computer_vision/capture_latest",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = capture_latest(response->message);
      });

    status_timer_ = create_wall_timer(5s, [this]() {report_waiting_status();});

    RCLCPP_INFO(get_logger(), "%s capture node ready", require_depth_ ? "RGB-D" : "RGB-only");
    RCLCPP_INFO(get_logger(), "  RGB:        %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Depth:      %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  CameraInfo: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Output:     %s", output_directory_.c_str());
    if (capture_on_start_) {
      RCLCPP_INFO(
        get_logger(),
        "Waiting for the first valid synchronized RGB-D pair...");
    }
  }

private:
  struct Snapshot
  {
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
  };

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
  }

  bool get_snapshot(Snapshot & snapshot, std::string & error)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_rgb_ || (require_depth_ && !latest_depth_) || !latest_camera_info_) {
      error = require_depth_ ? "Waiting for RGB, depth, and CameraInfo messages" :
        "Waiting for RGB and CameraInfo messages";
      return false;
    }

    if (!require_depth_) {
      const double age_ms = (now().seconds() - stamp_seconds(latest_rgb_->header.stamp)) * 1000.0;
      if (maximum_rgb_age_ms_ > 0.0 && (age_ms > maximum_rgb_age_ms_ || age_ms < -100.0)) {
        error = "Waiting for a fresh RGB image with a matching ROS clock";
        return false;
      }
      if (latest_rgb_->width != latest_camera_info_->width ||
        latest_rgb_->height != latest_camera_info_->height ||
        latest_rgb_->header.frame_id != latest_camera_info_->header.frame_id)
      {
        error = "RGB and CameraInfo dimensions/frame do not match";
        return false;
      }
      if (latest_camera_info_->k[0] <= 0 || latest_camera_info_->k[4] <= 0) {
        error = "CameraInfo has no calibrated intrinsics";
        return false;
      }
      snapshot = {latest_rgb_, nullptr, latest_camera_info_};
      return true;
    }

    const double delta_ms =
      std::abs(
      stamp_seconds(latest_rgb_->header.stamp) -
      stamp_seconds(latest_depth_->header.stamp)) *
      1000.0;
    if (delta_ms > maximum_timestamp_delta_ms_) {
      std::ostringstream out;
      out << "Latest RGB/depth timestamps differ by " << std::fixed
          << std::setprecision(1) << delta_ms << " ms (limit "
          << maximum_timestamp_delta_ms_ << " ms)";
      error = out.str();
      return false;
    }

    if (require_matching_dimensions_ &&
      (latest_rgb_->width != latest_depth_->width ||
      latest_rgb_->height != latest_depth_->height))
    {
      std::ostringstream out;
      out << "RGB is " << latest_rgb_->width << "x" << latest_rgb_->height
          << " but depth is " << latest_depth_->width << "x"
          << latest_depth_->height
          << "; enable Astra Pro depth registration/alignment";
      error = out.str();
      return false;
    }

    snapshot = {latest_rgb_, latest_depth_, latest_camera_info_};
    return true;
  }

  cv::Mat color_to_bgr(const sensor_msgs::msg::Image & msg) const
  {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      throw std::runtime_error("RGB image is empty");
    }

    const std::string & encoding = msg.encoding;
    cv::Mat output;
    if (encoding == sensor_msgs::image_encodings::BGR8) {
      cv::Mat view(msg.height, msg.width, CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      output = view.clone();
    } else if (encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat view(msg.height, msg.width, CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(view, output, cv::COLOR_RGB2BGR);
    } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
      cv::Mat view(msg.height, msg.width, CV_8UC4,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(view, output, cv::COLOR_BGRA2BGR);
    } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
      cv::Mat view(msg.height, msg.width, CV_8UC4,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(view, output, cv::COLOR_RGBA2BGR);
    } else if (encoding == sensor_msgs::image_encodings::MONO8 ||
      encoding == sensor_msgs::image_encodings::TYPE_8UC1)
    {
      cv::Mat view(msg.height, msg.width, CV_8UC1,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(view, output, cv::COLOR_GRAY2BGR);
    } else {
      throw std::runtime_error("Unsupported RGB encoding: " + encoding);
    }
    return output;
  }

  cv::Mat depth_to_meters(const sensor_msgs::msg::Image & msg) const
  {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      throw std::runtime_error("Depth image is empty");
    }

    cv::Mat depth_m;
    if (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg.encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv::Mat view(msg.height, msg.width, CV_16UC1,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      view.convertTo(depth_m, CV_32FC1, depth_unit_m_per_value_);
    } else if (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      cv::Mat view(msg.height, msg.width, CV_32FC1,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      depth_m = view.clone();
    } else {
      throw std::runtime_error("Unsupported depth encoding: " + msg.encoding);
    }
    return depth_m;
  }

  static double median_valid_depth(
    const cv::Mat & depth_m, int center_x,
    int center_y, int half_width,
    int half_height, std::size_t & valid_count)
  {
    const int x1 = std::max(0, center_x - half_width);
    const int y1 = std::max(0, center_y - half_height);
    const int x2 = std::min(depth_m.cols, center_x + half_width + 1);
    const int y2 = std::min(depth_m.rows, center_y + half_height + 1);

    std::vector<float> values;
    values.reserve(static_cast<std::size_t>((x2 - x1) * (y2 - y1)));
    for (int y = y1; y < y2; ++y) {
      for (int x = x1; x < x2; ++x) {
        const float value = depth_m.at<float>(y, x);
        if (std::isfinite(value) && value > 0.0F) {
          values.push_back(value);
        }
      }
    }

    valid_count = values.size();
    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return static_cast<double>(*middle);
  }

  static cv::Mat make_depth_preview(const cv::Mat & depth_m)
  {
    cv::Mat valid_mask = (depth_m > 0.0F);
    cv::Mat finite_mask = (depth_m == depth_m);
    cv::bitwise_and(valid_mask, finite_mask, valid_mask);

    double min_depth = 0.0;
    double max_depth = 0.0;
    cv::minMaxLoc(
      depth_m, &min_depth, &max_depth, nullptr, nullptr,
      valid_mask);

    cv::Mat gray(depth_m.size(), CV_8UC1, cv::Scalar(0));
    if (max_depth > min_depth) {
      depth_m.convertTo(
        gray, CV_8UC1, 255.0 / (max_depth - min_depth),
        -255.0 * min_depth / (max_depth - min_depth));
      gray.setTo(0, ~valid_mask);
    }
    cv::Mat preview;
    cv::applyColorMap(gray, preview, cv::COLORMAP_TURBO);
    preview.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
    return preview;
  }

  static void write_image_atomically(
    const fs::path & destination,
    const cv::Mat & image,
    const std::vector<int> & parameters = {})
  {
    fs::path temporary = destination.parent_path() /
      (destination.stem().string() + ".tmp" +
      destination.extension().string());
    if (!cv::imwrite(temporary.string(), image, parameters)) {
      throw std::runtime_error("Failed to write " + temporary.string());
    }
    std::error_code ec;
    fs::rename(temporary, destination, ec);
    if (ec) {
      fs::remove(destination, ec);
      ec.clear();
      fs::rename(temporary, destination, ec);
    }
    if (ec) {
      throw std::runtime_error(
              "Failed to replace " + destination.string() +
              ": " + ec.message());
    }
  }

  static void write_text_atomically(
    const fs::path & destination,
    const std::string & content)
  {
    fs::path temporary = destination;
    temporary += ".tmp";
    {
      std::ofstream file(temporary);
      if (!file) {
        throw std::runtime_error("Failed to open " + temporary.string());
      }
      file << content;
    }
    std::error_code ec;
    fs::rename(temporary, destination, ec);
    if (ec) {
      fs::remove(destination, ec);
      ec.clear();
      fs::rename(temporary, destination, ec);
    }
    if (ec) {
      throw std::runtime_error(
              "Failed to replace " + destination.string() +
              ": " + ec.message());
    }
  }

  bool capture_latest(std::string & result_message)
  {
    Snapshot snapshot;
    if (!get_snapshot(snapshot, result_message)) {
      return false;
    }

    try {
      fs::create_directories(output_directory_);
      const cv::Mat rgb_bgr = color_to_bgr(*snapshot.rgb);
      if (!require_depth_) {
        const fs::path output(output_directory_);
        write_image_atomically(output / "latest_rgb.jpg", rgb_bgr,
          {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
        const auto & info = *snapshot.camera_info;
        std::ostringstream meta;
        meta << std::setprecision(17);
        meta << "capture:\n  rgb_stamp_sec: " << stamp_seconds(snapshot.rgb->header.stamp)
             << "\n  rgb_stamp: [" << snapshot.rgb->header.stamp.sec << ", "
             << snapshot.rgb->header.stamp.nanosec << "]\n  rgb_frame_id: \""
             << snapshot.rgb->header.frame_id << "\"\n  require_depth: false\n";
        meta << "rgb:\n  width: " << snapshot.rgb->width
             << "\n  height: " << snapshot.rgb->height << "\n";
        meta << "camera_intrinsics:\n  fx: " << info.k[0] << "\n  fy: " << info.k[4]
             << "\n  cx: " << info.k[2] << "\n  cy: " << info.k[5] << "\n";
        meta << "  distortion_model: \"" << info.distortion_model << "\"\n  d: [";
        for (std::size_t i = 0; i < info.d.size(); ++i) {
          if (i) {meta << ", ";}
          meta << info.d[i];
        }
        meta << "]\n  binning: [" << info.binning_x << ", " << info.binning_y << "]\n";
        meta << "  roi: [" << info.roi.x_offset << ", " << info.roi.y_offset
             << ", " << info.roi.width << ", " << info.roi.height << "]\n";
        write_text_atomically(output / "latest_capture.yaml", meta.str());
        result_message = (output / "latest_rgb.jpg").string();
        RCLCPP_INFO(get_logger(), "Saved RGB-only capture: %s", result_message.c_str());
        return true;
      }
      const cv::Mat depth_m = depth_to_meters(*snapshot.depth);

      cv::Mat depth_mm(depth_m.size(), CV_16UC1, cv::Scalar(0));
      for (int y = 0; y < depth_m.rows; ++y) {
        for (int x = 0; x < depth_m.cols; ++x) {
          const float meters = depth_m.at<float>(y, x);
          if (std::isfinite(meters) && meters > 0.0F) {
            const double millimeters =
              std::clamp(
              static_cast<double>(meters) * 1000.0, 1.0,
              65535.0);
            depth_mm.at<std::uint16_t>(y, x) =
              static_cast<std::uint16_t>(std::lround(millimeters));
          }
        }
      }

      const int center_x = depth_m.cols / 2;
      const int center_y = depth_m.rows / 2;
      std::size_t valid_count = 0;
      const double center_depth_m = median_valid_depth(
        depth_m, center_x, center_y, diagnostic_roi_radius_px_,
        diagnostic_roi_radius_px_, valid_count);

      const auto & k = snapshot.camera_info->k;
      const double fx = k[0];
      const double fy = k[4];
      const double cx = k[2];
      const double cy = k[5];
      const bool valid_intrinsics = fx > 0.0 && fy > 0.0;
      const bool valid_center_depth = std::isfinite(center_depth_m);
      const double point_x =
        valid_intrinsics && valid_center_depth ?
        (static_cast<double>(center_x) - cx) * center_depth_m / fx :
        std::numeric_limits<double>::quiet_NaN();
      const double point_y =
        valid_intrinsics && valid_center_depth ?
        (static_cast<double>(center_y) - cy) * center_depth_m / fy :
        std::numeric_limits<double>::quiet_NaN();

      const fs::path output(output_directory_);
      write_image_atomically(
        output / "latest_rgb.jpg", rgb_bgr,
        {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
      write_image_atomically(
        output / "latest_depth.png", depth_mm,
        {cv::IMWRITE_PNG_COMPRESSION, 3});
      write_image_atomically(
        output / "latest_depth_preview.jpg", make_depth_preview(depth_m),
        {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

      std::ostringstream metadata;
      metadata << std::fixed << std::setprecision(9);
      metadata << "capture:\n";
      metadata << "  id: \"" << snapshot.rgb->header.stamp.sec << "-"
               << std::setw(9) << std::setfill('0')
               << snapshot.rgb->header.stamp.nanosec << std::setfill(' ')
               << "\"\n";
      metadata << "  rgb_stamp_sec: "
               << stamp_seconds(snapshot.rgb->header.stamp) << "\n";
      metadata << "  depth_stamp_sec: "
               << stamp_seconds(snapshot.depth->header.stamp) << "\n";
      metadata << "  rgb_frame_id: \"" << snapshot.rgb->header.frame_id
               << "\"\n";
      metadata << "  depth_frame_id: \"" << snapshot.depth->header.frame_id
               << "\"\n";
      metadata << "rgb:\n";
      metadata << "  file: latest_rgb.jpg\n";
      metadata << "  width: " << snapshot.rgb->width << "\n";
      metadata << "  height: " << snapshot.rgb->height << "\n";
      metadata << "  source_encoding: \"" << snapshot.rgb->encoding
               << "\"\n";
      metadata << "depth:\n";
      metadata << "  file: latest_depth.png\n";
      metadata << "  stored_encoding: 16UC1\n";
      metadata << "  stored_unit: millimeters\n";
      metadata << "  source_encoding: \"" << snapshot.depth->encoding
               << "\"\n";
      metadata << "camera_intrinsics:\n";
      metadata << "  fx: " << fx << "\n";
      metadata << "  fy: " << fy << "\n";
      metadata << "  cx: " << cx << "\n";
      metadata << "  cy: " << cy << "\n";
      metadata << "diagnostic_center_sample:\n";
      metadata << "  note: \"Scene-center diagnostic only; not a fruit detection\"\n";
      metadata << "  pixel_u: " << center_x << "\n";
      metadata << "  pixel_v: " << center_y << "\n";
      metadata << "  roi_radius_px: " << diagnostic_roi_radius_px_ << "\n";
      metadata << "  valid_depth_pixels: " << valid_count << "\n";
      metadata << "  median_depth_m: " << center_depth_m << "\n";
      metadata << "  point_in_camera_optical_frame_m:\n";
      metadata << "    x: " << point_x << "\n";
      metadata << "    y: " << point_y << "\n";
      metadata << "    z: " << center_depth_m << "\n";
      write_text_atomically(output / "latest_capture.yaml", metadata.str());

      result_message = (output / "latest_rgb.jpg").string();
      RCLCPP_INFO(get_logger(), "Saved synchronized RGB-D capture:");
      RCLCPP_INFO(
        get_logger(), "  %s",
        (output / "latest_rgb.jpg").c_str());
      RCLCPP_INFO(
        get_logger(), "  %s",
        (output / "latest_depth.png").c_str());
      RCLCPP_INFO(
        get_logger(), "  %s",
        (output / "latest_capture.yaml").c_str());
      if (valid_center_depth) {
        RCLCPP_INFO(
          get_logger(),
          "Diagnostic scene-center median depth: %.3f m (%zu pixels)",
          center_depth_m, valid_count);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "No valid depth near image center; files were still saved");
      }
      return true;
    } catch (const std::exception & exception) {
      result_message = exception.what();
      RCLCPP_ERROR(get_logger(), "Capture failed: %s", exception.what());
      return false;
    }
  }

  void attempt_automatic_capture()
  {
    bool should_capture = false;
    {
      std::lock_guard<std::mutex> lock(capture_mutex_);
      if (capture_pending_ && !capture_in_progress_) {
        capture_in_progress_ = true;
        should_capture = true;
      }
    }
    if (!should_capture) {
      return;
    }

    std::string message;
    const bool success = capture_latest(message);
    {
      std::lock_guard<std::mutex> lock(capture_mutex_);
      capture_in_progress_ = false;
      if (success) {
        capture_pending_ = false;
      }
    }

    if (success && exit_after_capture_) {
      shutdown_timer_ = create_wall_timer(100ms, []() {rclcpp::shutdown();});
    }
  }

  void report_waiting_status()
  {
    if (!capture_pending_) {
      return;
    }
    std::string message;
    Snapshot unused;
    if (!get_snapshot(unused, message)) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_directory_;
  double maximum_rgb_age_ms_{0.0};
  bool require_depth_{true};
  bool capture_on_start_{true};
  bool exit_after_capture_{false};
  bool require_matching_dimensions_{true};
  double maximum_timestamp_delta_ms_{100.0};
  double depth_unit_m_per_value_{0.001};
  int jpeg_quality_{95};
  int diagnostic_roi_radius_px_{10};

  std::mutex data_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_rgb_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;

  std::mutex capture_mutex_;
  bool capture_pending_{true};
  bool capture_in_progress_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

} // namespace computer_vision_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<computer_vision_model::RgbdCaptureNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
