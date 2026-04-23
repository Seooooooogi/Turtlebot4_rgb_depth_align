#ifndef OAK_D_ALIGN_CPP__OAKD_SENDER_HPP_
#define OAK_D_ALIGN_CPP__OAKD_SENDER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>

#include <depthai/depthai.hpp>

namespace oak_d_align_cpp
{

class OakdSender : public rclcpp::Node
{
public:
  explicit OakdSender(const rclcpp::NodeOptions & options);
  ~OakdSender() override;

private:
  // Pipeline construction
  void buildPipeline(bool subpixel, const std::string & preset);

  // Camera thread loop
  void cameraLoop();

  // Per-frame processing
  void processFrame(
    const std::shared_ptr<dai::ImgFrame> & rgb_frame,
    const std::shared_ptr<dai::ImgFrame> & depth_frame);

  // Overlay helpers (only used when enable_overlay_)
  cv::Mat colorizeDepth(const cv::Mat & depth);

  // DepthAI
  dai::Pipeline pipeline_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::DataOutputQueue> rgb_queue_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;

  // Socket constants
  static constexpr dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
  static constexpr dai::CameraBoardSocket LEFT_SOCKET = dai::CameraBoardSocket::CAM_B;
  static constexpr dai::CameraBoardSocket RIGHT_SOCKET = dai::CameraBoardSocket::CAM_C;

  // Parameters (cached at init)
  std::string robot_namespace_;
  double fps_ {30.0};
  int roi_size_ {5};
  double overlay_alpha_ {0.5};
  bool use_color_map_ {false};
  int jpeg_quality_overlay_ {80};
  bool enable_overlay_ {false};

  // Undistort maps (precomputed)
  cv::Mat undistort_map1_;
  cv::Mat undistort_map2_;

  // Publishers
  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_pub_;

  // Camera thread
  std::thread camera_thread_;
  std::atomic<bool> stop_flag_ {false};
};

}  // namespace oak_d_align_cpp

#endif  // OAK_D_ALIGN_CPP__OAKD_SENDER_HPP_
