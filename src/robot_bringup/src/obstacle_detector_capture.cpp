#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace fs = std::filesystem;

class ObstacleDetectorCapture : public rclcpp::Node {
public:
  ObstacleDetectorCapture() : Node("obstacle_detector_capture") {
    // Parameters
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("odom_topic", "/ground_truth/odom");
    this->declare_parameter<std::string>("save_dir", "saved_capcture");
    this->declare_parameter<double>("min_contour_area", 400.0);
    this->declare_parameter<double>("capture_cooldown_sec", 2.0);
    this->declare_parameter<bool>("require_gate_10", true);
    this->declare_parameter<bool>("enable_imshow", false);

    image_topic_ = this->get_parameter("image_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    save_dir_ = this->get_parameter("save_dir").as_string();
    min_contour_area_ = this->get_parameter("min_contour_area").as_double();
    capture_cooldown_ = std::chrono::milliseconds(
        static_cast<int64_t>(this->get_parameter("capture_cooldown_sec").as_double() * 1000.0));
    require_gate_10_ = this->get_parameter("require_gate_10").as_bool();
    enable_imshow_ = this->get_parameter("enable_imshow").as_bool();

    // Ensure save directory exists
    try {
      if (!fs::exists(save_dir_)) {
        fs::create_directories(save_dir_);
      }
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle Detector aktif. Direktori: %s | Syarat Gerbang 10: %s",
                  fs::absolute(save_dir_).c_str(), require_gate_10_ ? "true" : "false");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Gagal membuat direktori %s: %s",
                   save_dir_.c_str(), e.what());
    }

    last_capture_yellow_ = std::chrono::steady_clock::now() - capture_cooldown_;
    last_capture_blue_ = std::chrono::steady_clock::now() - capture_cooldown_;

    // Subscribers & Publishers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10,
        std::bind(&ObstacleDetectorCapture::imageCallback, this,
                  std::placeholders::_1));

    gate_10_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/vision/gate_10_passed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && !gate_10_passed_) {
            gate_10_passed_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "[GATE TRIGGER] Sinyal gerbang 10 diterima! Deteksi kotak diaktifkan.");
          }
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          double x = msg->pose.pose.position.x;
          double y = msg->pose.pose.position.y;
          if (y > 1.2) {
            visited_top_corridor_ = true;
          }
          // Fallback location check: lower-left corner past gate 10
          if (visited_top_corridor_ && x < -0.8 && y < -0.75) {
            if (!gate_10_passed_) {
              gate_10_passed_ = true;
              RCLCPP_INFO(this->get_logger(),
                          "[ODOM TRIGGER] Posisi robot telah melewati gerbang ke-10 (x=%.2f, y=%.2f)!",
                          x, y);
            }
          }
        });

    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/vision/obstacle_detection/image", 10);

    RCLCPP_INFO(this->get_logger(), "Mendengarkan topik kamera: %s",
                image_topic_.c_str());
  }

private:
  struct BoxDetection {
    bool detected{false};
    cv::Rect bbox;
    cv::RotatedRect rot_rect;
    double area{0.0};
  };

  std::string getTimestampString() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << "_"
       << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
  }

  BoxDetection detectRectangularObstacle(const cv::Mat &hsv,
                                        const cv::Scalar &lower,
                                        const cv::Scalar &upper) {
    BoxDetection result;
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);

    // Morphological cleanup
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0.0;
    for (const auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < min_contour_area_) {
        continue;
      }

      double perimeter = cv::arcLength(c, true);
      if (perimeter <= 0.0) {
        continue;
      }

      // Check circularity to strictly reject round buoys!
      double circularity = (4.0 * M_PI * area) / (perimeter * perimeter);
      if (circularity > 0.60) {
        // Round shape: ignore (it's a buoy, not a rectangular box)
        continue;
      }

      cv::Rect bbox = cv::boundingRect(c);
      cv::RotatedRect rot_rect = cv::minAreaRect(c);

      float w = rot_rect.size.width;
      float h = rot_rect.size.height;
      if (w < 1.0f || h < 1.0f) {
        continue;
      }

      // Rectangular box has elongated aspect ratio (>= 1.5)
      float aspect_ratio = std::max(w, h) / std::min(w, h);
      float extent = static_cast<float>(area) / (w * h);

      if (aspect_ratio >= 1.45f && extent >= 0.55f) {
        if (area > max_area) {
          max_area = area;
          result.detected = true;
          result.bbox = bbox;
          result.rot_rect = rot_rect;
          result.area = area;
        }
      }
    }
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;

    // Check if Gate 10 condition is met
    bool capture_active = !require_gate_10_ || gate_10_passed_;

    if (!capture_active) {
      // Display lock banner
      cv::putText(frame, "OBSTACLE CAPTURE: LOCKED",
                  cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  cv::Scalar(0, 165, 255), 2);
    } else {
      cv::putText(frame, "OBSTACLE CAPTURE: ACTIVE (SETELAH GERBANG 10)",
                  cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  cv::Scalar(0, 255, 0), 2);

      cv::Mat hsv;
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

      // HSV Ranges for Boxes
      // Yellow Obstacle Bar: H in [20, 38], S in [100, 255], V in [100, 255]
      cv::Scalar yellow_lower(20, 80, 30);
      cv::Scalar yellow_upper(38, 255, 255);

      // Blue Obstacle Bar: H in [100, 130], S in [80, 255], V in [50, 255]
      cv::Scalar blue_lower(100, 80, 20);
      cv::Scalar blue_upper(130, 255, 255);

      BoxDetection yellow_det = detectRectangularObstacle(hsv, yellow_lower, yellow_upper);
      BoxDetection blue_det = detectRectangularObstacle(hsv, blue_lower, blue_upper);

      auto now = std::chrono::steady_clock::now();

      // Handle Yellow Obstacle detection and capture
      if (yellow_det.detected) {
        cv::rectangle(frame, yellow_det.bbox, cv::Scalar(0, 255, 255), 2);
        std::string label = "YELLOW RECT BOX (Area: " + std::to_string(static_cast<int>(yellow_det.area)) + ")";
        cv::putText(frame, label, cv::Point(yellow_det.bbox.x, std::max(20, yellow_det.bbox.y - 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

        if (now - last_capture_yellow_ >= capture_cooldown_) {
          std::string filename = save_dir_ + "/yellow_box_" + getTimestampString() + ".jpg";
          if (cv::imwrite(filename, frame)) {
            RCLCPP_INFO(this->get_logger(), "[CAPTURE] Kotak kuning persegi panjang disimpan: %s", filename.c_str());
            last_capture_yellow_ = now;
          }
        }
      }

      // Handle Blue Obstacle detection and capture
      if (blue_det.detected) {
        cv::rectangle(frame, blue_det.bbox, cv::Scalar(255, 0, 0), 2);
        std::string label = "BLUE RECT BOX (Area: " + std::to_string(static_cast<int>(blue_det.area)) + ")";
        cv::putText(frame, label, cv::Point(blue_det.bbox.x, std::max(20, blue_det.bbox.y - 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

        if (now - last_capture_blue_ >= capture_cooldown_) {
          std::string filename = save_dir_ + "/blue_box_" + getTimestampString() + ".jpg";
          if (cv::imwrite(filename, frame)) {
            RCLCPP_INFO(this->get_logger(), "[CAPTURE] Kotak biru persegi panjang disimpan: %s", filename.c_str());
            last_capture_blue_ = now;
          }
        }
      }
    }

    // Publish annotated debug frame
    sensor_msgs::msg::Image::SharedPtr out_msg =
        cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    debug_pub_->publish(*out_msg);

    if (enable_imshow_) {
      cv::imshow("Obstacle Detector", frame);
      cv::waitKey(1);
    }
  }

  std::string image_topic_;
  std::string odom_topic_;
  std::string save_dir_;
  double min_contour_area_;
  std::chrono::milliseconds capture_cooldown_;
  bool require_gate_10_;
  bool enable_imshow_;

  bool gate_10_passed_{false};
  bool visited_top_corridor_{false};

  std::chrono::steady_clock::time_point last_capture_yellow_;
  std::chrono::steady_clock::time_point last_capture_blue_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_10_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorCapture>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
