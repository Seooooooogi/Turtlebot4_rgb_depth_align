#include "oak_d_align_cpp/oakd_sender.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace oak_d_align_cpp
{

OakdSender::OakdSender(const rclcpp::NodeOptions & options)
: rclcpp::Node("oakd_sender", options)
{
  // ── 파라미터 선언 (Python 노드 스키마와 1:1 일치) ─────────────────
  // robot_namespace: 비워두면 prefix 없이 /oakd/... 로 발행. TurtleBot4 배포 시 YAML 에서 "robot9" 로 override.
  declare_parameter<std::string>("robot_namespace", "");
  declare_parameter<double>("fps", 30.0);
  declare_parameter<bool>("stereo_subpixel", true);
  declare_parameter<std::string>("stereo_preset", "HIGH_DENSITY");
  declare_parameter<double>("ir_dot_projector_intensity", 0.0);
  declare_parameter<double>("ir_flood_intensity", 0.0);
  declare_parameter<int>("roi_size", 5);
  declare_parameter<double>("overlay_alpha", 0.5);
  declare_parameter<bool>("use_color_map", false);
  declare_parameter<int>("jpeg_quality_overlay", 80);
  declare_parameter<bool>("enable_overlay", false);

  robot_namespace_ = get_parameter("robot_namespace").as_string();
  fps_ = get_parameter("fps").as_double();
  const auto subpixel = get_parameter("stereo_subpixel").as_bool();
  const auto preset = get_parameter("stereo_preset").as_string();
  const auto ir_dot = get_parameter("ir_dot_projector_intensity").as_double();
  const auto ir_flood = get_parameter("ir_flood_intensity").as_double();
  roi_size_ = get_parameter("roi_size").as_int();
  overlay_alpha_ = get_parameter("overlay_alpha").as_double();
  use_color_map_ = get_parameter("use_color_map").as_bool();
  jpeg_quality_overlay_ = get_parameter("jpeg_quality_overlay").as_int();
  enable_overlay_ = get_parameter("enable_overlay").as_bool();

  // ── 파이프라인 구축 ────────────────────────────────────────────────
  buildPipeline(subpixel, preset);

  // ── 디바이스 오픈 (파이프라인 통째 주입) ──────────────────────────
  try {
    device_ = std::make_shared<dai::Device>(pipeline_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "dai::Device open 실패: %s", e.what());
    throw;
  }

  // ── 캘리브레이션 ──────────────────────────────────────────────────
  auto calib = device_->readCalibration();

  const auto distortion_model = calib.getDistortionModel(RGB_SOCKET);
  if (distortion_model != dai::CameraModel::Perspective) {
    RCLCPP_WARN(
      get_logger(),
      "RGB 카메라가 Perspective 모델이 아닙니다. 왜곡 보정이 이상할 수 있습니다.");
  }

  // ── IR illumination (device 부팅 후에만 설정 가능) ────────────────
  if (ir_dot > 0.0) {
    device_->setIrLaserDotProjectorBrightness(static_cast<float>(ir_dot));
    RCLCPP_INFO(get_logger(), "IR dot projector 활성화: intensity=%.3f", ir_dot);
  }
  if (ir_flood > 0.0) {
    device_->setIrFloodLightBrightness(static_cast<float>(ir_flood));
    RCLCPP_INFO(get_logger(), "IR flood LED 활성화: intensity=%.3f", ir_flood);
  }

  // ── 큐 핸들 ───────────────────────────────────────────────────────
  rgb_queue_ = device_->getOutputQueue("rgb", 4, false);
  depth_queue_ = device_->getOutputQueue("depth", 4, false);

  // ── undistort map 사전 계산 (매 프레임 호출 제거) ─────────────────
  // IspScale(2, 3): 1920×1080 → 1280×720
  const int rgb_w = 1920 * 2 / 3;
  const int rgb_h = 1080 * 2 / 3;
  auto K_vec = calib.getCameraIntrinsics(RGB_SOCKET, rgb_w, rgb_h);
  auto D_vec = calib.getDistortionCoefficients(RGB_SOCKET);

  cv::Mat K(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      K.at<double>(i, j) = static_cast<double>(K_vec[i][j]);
    }
  }
  cv::Mat D(1, static_cast<int>(D_vec.size()), CV_64F);
  for (size_t i = 0; i < D_vec.size(); ++i) {
    D.at<double>(0, static_cast<int>(i)) = static_cast<double>(D_vec[i]);
  }
  cv::initUndistortRectifyMap(
    K, D, cv::Mat(), K, cv::Size(rgb_w, rgb_h), CV_16SC2,
    undistort_map1_, undistort_map2_);

  // ── Publishers (image_transport: raw + compressed 플러그인 자동 발행) ──
  // robot_namespace 가 비어 있으면 prefix 없이, 지정되면 "/{ns}/..." 로.
  const std::string ns_prefix = robot_namespace_.empty() ? "" : "/" + robot_namespace_;
  const std::string rgb_topic = ns_prefix + "/oakd/rgb/image_raw/aligned";
  const std::string depth_topic = ns_prefix + "/oakd/stereo/image_raw/aligned";
  const std::string overlay_topic = ns_prefix + "/oakd/overlay/compressed";

  rgb_pub_ = image_transport::create_publisher(this, rgb_topic);
  depth_pub_ = image_transport::create_publisher(this, depth_topic);
  if (enable_overlay_) {
    overlay_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      overlay_topic, rclcpp::QoS(10));
  }

  RCLCPP_INFO(
    get_logger(),
    "===== OAK-D C++ 노드 가동 완료 =====\n"
    "  namespace      : %s\n"
    "  fps            : %.1f\n"
    "  stereo_preset  : %s\n"
    "  stereo_subpixel: %s\n"
    "  RGB output     : %dx%d (IspScale 2/3)\n"
    "  ir_dot         : %.3f %s\n"
    "  ir_flood       : %.3f %s\n"
    "  enable_overlay : %s\n"
    "  RGB topic      : %s\n"
    "  Depth topic    : %s\n"
    "====================================",
    robot_namespace_.c_str(), fps_, preset.c_str(),
    subpixel ? "true" : "false", rgb_w, rgb_h,
    ir_dot, ir_dot > 0.0 ? "(on)" : "(off)",
    ir_flood, ir_flood > 0.0 ? "(on)" : "(off)",
    enable_overlay_ ? "true" : "false",
    rgb_topic.c_str(), depth_topic.c_str());

  // ── 카메라 루프 스레드 시작 (ROS2 executor 와 독립) ────────────────
  camera_thread_ = std::thread(&OakdSender::cameraLoop, this);
}

OakdSender::~OakdSender()
{
  stop_flag_ = true;
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  if (device_) {
    device_->close();
  }
}

void OakdSender::buildPipeline(bool subpixel, const std::string & preset)
{
  auto camRgb = pipeline_.create<dai::node::ColorCamera>();
  auto left = pipeline_.create<dai::node::MonoCamera>();
  auto right = pipeline_.create<dai::node::MonoCamera>();
  auto stereo = pipeline_.create<dai::node::StereoDepth>();
  auto outRgb = pipeline_.create<dai::node::XLinkOut>();
  auto outDepth = pipeline_.create<dai::node::XLinkOut>();

  outRgb->setStreamName("rgb");
  outDepth->setStreamName("depth");

  // Mono 카메라 800P (C++ 기본값과 통일)
  left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  left->setBoardSocket(LEFT_SOCKET);
  left->setFps(static_cast<float>(fps_));

  right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  right->setBoardSocket(RIGHT_SOCKET);
  right->setFps(static_cast<float>(fps_));

  // RGB 1080P + IspScale 2/3 → 1280×720
  camRgb->setBoardSocket(RGB_SOCKET);
  camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  camRgb->setFps(static_cast<float>(fps_));
  camRgb->setIspScale(2, 3);

  // Stereo preset
  auto preset_mode = dai::node::StereoDepth::PresetMode::HIGH_DENSITY;
  if (preset == "HIGH_ACCURACY") {
    preset_mode = dai::node::StereoDepth::PresetMode::HIGH_ACCURACY;
  }
  stereo->setDefaultProfilePreset(preset_mode);
  // 핵심: VPU에서 CAM_A(RGB) 기준으로 depth 정렬 (ImageAlign 없이)
  stereo->setDepthAlign(RGB_SOCKET);
  stereo->setLeftRightCheck(true);
  stereo->setSubpixel(subpixel);

  // 링크
  camRgb->isp.link(outRgb->input);
  left->out.link(stereo->left);
  right->out.link(stereo->right);
  stereo->depth.link(outDepth->input);
}

void OakdSender::cameraLoop()
{
  while (!stop_flag_.load()) {
    try {
      auto rgb_frame = rgb_queue_->get<dai::ImgFrame>();
      auto depth_frame = depth_queue_->get<dai::ImgFrame>();
      if (!rgb_frame || !depth_frame) {
        continue;
      }
      processFrame(rgb_frame, depth_frame);
    } catch (const std::exception & e) {
      if (!stop_flag_.load()) {
        RCLCPP_ERROR(get_logger(), "Camera loop error: %s", e.what());
      }
    }
  }
}

void OakdSender::processFrame(
  const std::shared_ptr<dai::ImgFrame> & rgb_frame,
  const std::shared_ptr<dai::ImgFrame> & depth_frame)
{
  const auto stamp = now();
  const std::string frame_id = "oakd_aligned_frame";

  // RGB: cv::remap 으로 undistort (Python cv2.remap 과 동등)
  cv::Mat rgb_cv = rgb_frame->getCvFrame();
  cv::Mat rgb_undistorted;
  cv::remap(
    rgb_cv, rgb_undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);

  // Depth: 16UC1 그대로
  cv::Mat depth_cv = depth_frame->getFrame();

  // ── RGB 발행 (image_transport 가 compressed 자동 발행) ──
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    auto msg = cv_bridge::CvImage(header, "bgr8", rgb_undistorted).toImageMsg();
    rgb_pub_.publish(msg);
  }

  // ── Depth 발행 (16UC1) ──
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    auto msg = cv_bridge::CvImage(header, "16UC1", depth_cv).toImageMsg();
    depth_pub_.publish(msg);
  }

  // ── Overlay (enable_overlay_ == true 일 때만) ──
  if (enable_overlay_ && overlay_pub_) {
    cv::Mat depth_colored = colorizeDepth(depth_cv);
    if (depth_colored.size() != rgb_undistorted.size()) {
      cv::resize(depth_colored, depth_colored, rgb_undistorted.size());
    }
    cv::Mat overlay_img;
    cv::addWeighted(
      rgb_undistorted, 1.0 - overlay_alpha_,
      depth_colored, overlay_alpha_, 0.0, overlay_img);

    std::vector<uint8_t> jpeg_buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_overlay_};
    cv::imencode(".jpg", overlay_img, jpeg_buffer, encode_params);

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.format = "jpeg";
    msg.data = std::move(jpeg_buffer);
    overlay_pub_->publish(std::move(msg));
  }
}

cv::Mat OakdSender::colorizeDepth(const cv::Mat & depth)
{
  // Python 의 percentile + log-scale 알고리즘을 그대로 포팅.
  std::vector<uint16_t> valid_depths;
  valid_depths.reserve(depth.total());
  for (int y = 0; y < depth.rows; ++y) {
    const uint16_t * row = depth.ptr<uint16_t>(y);
    for (int x = 0; x < depth.cols; ++x) {
      if (row[x] > 0) {
        valid_depths.push_back(row[x]);
      }
    }
  }
  if (valid_depths.empty()) {
    return cv::Mat::zeros(depth.size(), CV_8UC3);
  }

  const size_t n = valid_depths.size();
  const size_t p3_idx = std::min(n - 1, (n * 3) / 100);
  const size_t p95_idx = std::min(n - 1, (n * 95) / 100);

  std::nth_element(valid_depths.begin(), valid_depths.begin() + p3_idx, valid_depths.end());
  const float min_depth = static_cast<float>(valid_depths[p3_idx]);

  std::nth_element(valid_depths.begin(), valid_depths.begin() + p95_idx, valid_depths.end());
  const float max_depth = static_cast<float>(valid_depths[p95_idx]);

  if (max_depth <= min_depth) {
    return cv::Mat::zeros(depth.size(), CV_8UC3);
  }

  const float log_min = std::log(min_depth);
  const float log_max = std::log(max_depth);
  const float log_range = log_max - log_min;
  const float scale = 255.0f / log_range;

  cv::Mat normalized(depth.size(), CV_8U);
  for (int y = 0; y < depth.rows; ++y) {
    const uint16_t * src_row = depth.ptr<uint16_t>(y);
    uint8_t * dst_row = normalized.ptr<uint8_t>(y);
    for (int x = 0; x < depth.cols; ++x) {
      if (src_row[x] == 0) {
        dst_row[x] = 0;
      } else {
        float log_val = std::log(static_cast<float>(src_row[x]));
        if (log_val < log_min) log_val = log_min;
        if (log_val > log_max) log_val = log_max;
        dst_row[x] = static_cast<uint8_t>((log_val - log_min) * scale);
      }
    }
  }

  cv::Mat colored;
  if (use_color_map_) {
    cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
  } else {
    cv::cvtColor(normalized, colored, cv::COLOR_GRAY2BGR);
  }
  // 무효(0) 픽셀을 검정으로 설정
  cv::Mat invalid_mask = (depth == 0);
  colored.setTo(cv::Scalar(0, 0, 0), invalid_mask);
  return colored;
}

}  // namespace oak_d_align_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(oak_d_align_cpp::OakdSender)
