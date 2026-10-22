#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

class BuoyCentering : public rclcpp::Node {
public:
  BuoyCentering() : Node("buoy_centering") {
    // Parameters
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("odom_topic", "/ground_truth/odom");
    this->declare_parameter<bool>("publish_cmd_vel", false);
    this->declare_parameter<double>("kp_angular", 1.5);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("nominal_linear_speed", 0.25);
    this->declare_parameter<double>("min_contour_area", 100.0);
    this->declare_parameter<double>("min_circularity", 0.55);
    this->declare_parameter<bool>("enable_imshow", false);

    image_topic_ = this->get_parameter("image_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    publish_cmd_vel_ = this->get_parameter("publish_cmd_vel").as_bool();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    nominal_linear_speed_ = this->get_parameter("nominal_linear_speed").as_double();
    min_contour_area_ = this->get_parameter("min_contour_area").as_double();
    min_circularity_ = this->get_parameter("min_circularity").as_double();
    enable_imshow_ = this->get_parameter("enable_imshow").as_bool();

    // Subscribers & Publishers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10,
        std::bind(&BuoyCentering::imageCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&BuoyCentering::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/vision/buoy_centering/image", 10);
    gate_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vision/gate_count", 10);
    gate_10_passed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/gate_10_passed", 10);
    centering_error_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vision/centering_error", 10);
    gate_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/gate_detected", 10);

    RCLCPP_INFO(this->get_logger(),
                "Buoy Centering Node aktif. Kamera: %s | CmdVel: %s | Odom: %s",
                image_topic_.c_str(), cmd_vel_topic_.c_str(), odom_topic_.c_str());
  }

private:
  struct BuoyDetection {
    bool detected{false};
    cv::Point2f center{0.0f, 0.0f};
    float radius{0.0f};
    double area{0.0};
  };

  BuoyDetection detectRoundBuoy(const cv::Mat &mask) {
    BuoyDetection best;
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

      // Circularity: 4 * pi * Area / Perimeter^2 (1.0 for perfect circle)
      double circularity = (4.0 * M_PI * area) / (perimeter * perimeter);
      cv::Rect bbox = cv::boundingRect(c);
      float aspect_ratio = static_cast<float>(bbox.width) / static_cast<float>(bbox.height);

      // Check for round / spherical shape
      if (circularity >= min_circularity_ && aspect_ratio >= 0.65f && aspect_ratio <= 1.45f) {
        if (area > max_area) {
          max_area = area;
          best.detected = true;
          best.area = area;
          cv::minEnclosingCircle(c, best.center, best.radius);
        }
      }
    }
    return best;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Track when robot enters top corridor (Y > 1.2)
    if (current_y_ > 1.2) {
      visited_top_corridor_ = true;
    }

    // Gate 10 (Lower Left gate) is located at x ~ -1.90, y = -0.63
    // Once robot has done the loop and descends past y < -0.75 in the left section (x < -0.8),
    // it has definitively cleared gate 10 and entered the obstacle zone!
    if (visited_top_corridor_ && current_x_ < -0.8 && current_y_ < -0.75) {
      if (!gate_10_passed_) {
        gate_10_passed_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "[GATE STATUS] Robot telah melewati Gerbang ke-10! Koordinat: (%.2f, %.2f)",
                    current_x_, current_y_);
      }
    }
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
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Color Thresholds for Buoys
    // Red Buoy wraps around 0 and 180 in HSV
    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv, cv::Scalar(0, 90, 70), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(170, 90, 70), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    // Green Buoy: H in [35, 85], S in [90, 255], V in [60, 255]
    cv::Mat mask_green;
    cv::inRange(hsv, cv::Scalar(35, 90, 60), cv::Scalar(85, 255, 255), mask_green);

    // Morphological noise removal
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_green, mask_green, cv::MORPH_CLOSE, kernel);

    BuoyDetection red_buoy = detectRoundBuoy(mask_red);
    BuoyDetection green_buoy = detectRoundBuoy(mask_green);

    int img_w = frame.cols;
    int img_h = frame.rows;
    float center_x = img_w / 2.0f;

    // Draw center guideline (target vertical line)
    cv::line(frame, cv::Point(center_x, 0), cv::Point(center_x, img_h), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    geometry_msgs::msg::Twist twist;
    float current_error = 0.0f;
    bool is_detected = false;

    // Gate counter logic
    if (red_buoy.detected && green_buoy.detected) {
      if (red_buoy.radius > 12.0f && green_buoy.radius > 12.0f && !inside_gate_) {
        inside_gate_ = true;
      }
    } else {
      if (inside_gate_) {
        gate_count_++;
        inside_gate_ = false;
        RCLCPP_INFO(this->get_logger(), "[GATE PASSED] Berhasil melewati gerbang #%d / 10", gate_count_);
        if (gate_count_ >= 10) {
          gate_10_passed_ = true;
        }
      }
    }

    // Publish gate status
    std_msgs::msg::Int32 count_msg;
    count_msg.data = gate_count_;
    gate_count_pub_->publish(count_msg);

    std_msgs::msg::Bool bool_msg;
    bool_msg.data = gate_10_passed_;
    gate_10_passed_pub_->publish(bool_msg);

    // Visual overlay and centering calculations
    if (red_buoy.detected) {
      cv::circle(frame, red_buoy.center, static_cast<int>(red_buoy.radius), cv::Scalar(0, 0, 255), 2);
      cv::circle(frame, red_buoy.center, 3, cv::Scalar(0, 0, 255), -1);
      cv::putText(frame, "RED BUOY", cv::Point(red_buoy.center.x - 30, red_buoy.center.y - 12),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
    }

    if (green_buoy.detected) {
      cv::circle(frame, green_buoy.center, static_cast<int>(green_buoy.radius), cv::Scalar(0, 255, 0), 2);
      cv::circle(frame, green_buoy.center, 3, cv::Scalar(0, 255, 0), -1);
      cv::putText(frame, "GREEN BUOY", cv::Point(green_buoy.center.x - 35, green_buoy.center.y - 12),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
    }

    if (red_buoy.detected && green_buoy.detected) {
      // Both buoys visible: Calculate midpoint of gate
      cv::Point2f midpoint((red_buoy.center.x + green_buoy.center.x) / 2.0f,
                           (red_buoy.center.y + green_buoy.center.y) / 2.0f);

      // Connect buoys with gate line
      cv::line(frame, red_buoy.center, green_buoy.center, cv::Scalar(0, 255, 255), 2);
      cv::circle(frame, midpoint, 6, cv::Scalar(0, 255, 255), -1);

      // Normalized error: [-1.0 (far left) to +1.0 (far right)]
      float error_x = (midpoint.x - center_x) / center_x;
      current_error = error_x;
      is_detected = true;

      // Angular steering command: Turn towards midpoint
      double angular_cmd = -kp_angular_ * error_x;
      angular_cmd = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_cmd));

      // Linear speed: slow down when heading is misaligned
      double alignment_factor = std::max(0.0, 1.0 - std::abs(error_x));
      double linear_cmd = nominal_linear_speed_ * alignment_factor;

      if (std::abs(error_x) > 0.35) {
        linear_cmd = 0.05; // Crawl while adjusting heading
      }

      twist.linear.x = linear_cmd;
      twist.angular.z = angular_cmd;

      std::string status = "GATE CENTERED | Err: " + std::to_string(error_x).substr(0, 5) +
                           " | v: " + std::to_string(linear_cmd).substr(0, 4) +
                           " | w: " + std::to_string(angular_cmd).substr(0, 4);
      cv::putText(frame, status, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  cv::Scalar(0, 255, 255), 2);

    } else if (red_buoy.detected && !green_buoy.detected) {
      // Only Red Buoy visible: keep it on the left side of frame
      float offset = red_buoy.center.x - (center_x * 0.45f);
      float error_x = offset / center_x;
      current_error = error_x;
      is_detected = true;
      twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, -kp_angular_ * error_x));
      twist.linear.x = 0.15;

      cv::putText(frame, "TRACKING RED BUOY (Gate Left)", cv::Point(15, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

    } else if (!red_buoy.detected && green_buoy.detected) {
      // Only Green Buoy visible: keep it on the right side of frame
      float offset = green_buoy.center.x - (center_x * 1.55f);
      float error_x = offset / center_x;
      current_error = error_x;
      is_detected = true;
      twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, -kp_angular_ * error_x));
      twist.linear.x = 0.15;

      cv::putText(frame, "TRACKING GREEN BUOY (Gate Right)", cv::Point(15, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

    } else {
      cv::putText(frame, "SEARCHING FOR BUOYS...", cv::Point(15, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 180, 180), 1);
    }

    // Publish visual centering error & gate detected for waypoint follower
    std_msgs::msg::Float32 err_msg;
    err_msg.data = current_error;
    centering_error_pub_->publish(err_msg);

    std_msgs::msg::Bool det_msg;
    det_msg.data = is_detected;
    gate_detected_pub_->publish(det_msg);

    // Display Gate Progress on bottom of frame
    std::string gate_banner = "Gate Count: " + std::to_string(gate_count_) +
                              " / 10 | Passed Gate 10: " + (gate_10_passed_ ? "YES" : "NO");
    cv::putText(frame, gate_banner, cv::Point(15, img_h - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                gate_10_passed_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);

    // Publish velocity command if enabled and gate/buoy is in sight
    if (publish_cmd_vel_ && (red_buoy.detected || green_buoy.detected)) {
      cmd_pub_->publish(twist);
    }

    // Publish visual debug feed
    sensor_msgs::msg::Image::SharedPtr out_msg =
        cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    debug_pub_->publish(*out_msg);

    if (enable_imshow_) {
      cv::imshow("Buoy Centering", frame);
      cv::waitKey(1);
    }
  }

  std::string image_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  bool publish_cmd_vel_;
  double kp_angular_;
  double max_angular_speed_;
  double nominal_linear_speed_;
  double min_contour_area_;
  double min_circularity_;
  bool enable_imshow_;

  // Gate tracking variables
  int gate_count_{0};
  bool inside_gate_{false};
  bool gate_10_passed_{false};
  bool visited_top_corridor_{false};
  double current_x_{0.0};
  double current_y_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gate_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate_10_passed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr centering_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate_detected_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BuoyCentering>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
