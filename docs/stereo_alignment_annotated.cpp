/**
 * stereo_alignment_annotated.cpp
 *
 * 원본: src/depthai-ros/depthai_ros_driver/src/dai_nodes/sensors/stereo.cpp
 *
 * OAK-D Pro (RVC2)의 RGB-Depth 정렬이 C++ 드라이버 내부에서
 * 어떻게 구성되는지 설명하기 위한 주석 파일.
 *
 * ─── 정렬 흐름 요약 ───────────────────────────────────────────
 *
 *  CAM_B (Left Mono)  ──┐
 *                        ├──► StereoDepth ──► depth ──► [ROS2 토픽]
 *  CAM_C (Right Mono) ──┘         │
 *                                  └──► setDepthAlign(CAM_A)
 *                                            ▲
 *                                    CAM_A (RGB)를 기준으로
 *                                    깊이 맵을 재투영(reprojection)
 *
 * 정렬은 OAK-D 내부 VPU에서 처리 → RPi4 CPU 부하 없음.
 * ──────────────────────────────────────────────────────────────
 */

#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
// ... (기타 헤더 생략)

namespace depthai_ros_driver {
namespace dai_nodes {

// ─── 생성자: 스테레오 노드 초기화 ────────────────────────────────
Stereo::Stereo(const std::string& daiNodeName,
               std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<dai::Pipeline> pipeline,
               std::shared_ptr<dai::Device> device,
               dai::CameraBoardSocket leftSocket,   // 기본값: CAM_B (왼쪽 모노)
               dai::CameraBoardSocket rightSocket)  // 기본값: CAM_C (오른쪽 모노)
    : BaseNode(daiNodeName, node, pipeline) {

    // ── [1] 정렬 기준 소켓 결정 ────────────────────────────────
    // 일반 OAK-D: CAM_A(RGB)를 기준으로 깊이 정렬
    // OAK-D-SR / SR-POE 모델은 예외적으로 CAM_C 기준
    auto alignSocket = dai::CameraBoardSocket::CAM_A;
    if(device->getDeviceName() == "OAK-D-SR" || device->getDeviceName() == "OAK-D-SR-POE") {
        alignSocket = dai::CameraBoardSocket::CAM_C;
    }
    // YAML 파라미터 i_board_socket_id 로 오버라이드 가능
    ph->updateSocketsFromParams(leftSocket, rightSocket, alignSocket);

    // ── [2] 연결된 카메라 하드웨어 정보 수집 ──────────────────
    // 장치에서 카메라 소켓별 해상도·타입 정보를 읽어옴
    auto features = device->getConnectedCameraFeatures();
    for(auto f : features) {
        if(f.socket == leftSocket)       leftSensInfo  = f;
        else if(f.socket == rightSocket) rightSensInfo = f;
    }

    // ── [3] 좌/우 모노 카메라 래퍼 생성 ──────────────────────
    // SensorWrapper: 각 모노 카메라를 ROS2 노드로 감싸는 헬퍼
    left  = std::make_unique<SensorWrapper>(/* CAM_B */);
    right = std::make_unique<SensorWrapper>(/* CAM_C */);

    // ── [4] StereoDepth 노드 생성 ─────────────────────────────
    // StereoDepth: OAK-D VPU 내부에서 좌/우 이미지로 시차(disparity)→깊이 변환
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();

    // YAML 파라미터를 StereoDepth 노드에 적용 (subpixel, lr_check 등)
    ph->declareParams(stereoCamNode);   // ← 아래 파라미터 섹션 참고

    // ── [5] 파이프라인 연결 ───────────────────────────────────
    setXinXout(pipeline);
    left->link(stereoCamNode->left);    // CAM_B → StereoDepth.left
    right->link(stereoCamNode->right);  // CAM_C → StereoDepth.right
}


// ─── setXinXout: 출력 큐 설정 ────────────────────────────────────
// 어떤 데이터를 ROS2로 내보낼지 결정
void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    bool outputDisparity = ph->getParam<bool>("i_output_disparity");
    bool lowBandwidth    = ph->getParam<bool>("i_low_bandwidth");

    // i_output_disparity=true  → 시차 이미지(disparity) 발행 (정규화된 픽셀값)
    // i_output_disparity=false → 깊이 이미지(depth, mm 단위) 발행  ← 기본값
    std::function<void(dai::Node::Input)> stereoLinkChoice;
    if(outputDisparity || lowBandwidth) {
        stereoLinkChoice = [&](auto input) { stereoCamNode->disparity.link(input); };
    } else {
        stereoLinkChoice = [&](auto input) { stereoCamNode->depth.link(input); };
    }

    // i_publish_topic=true 일 때만 /oak/stereo/image_raw 토픽 생성
    if(ph->getParam<bool>("i_publish_topic")) {
        stereoPub = setupOutput(pipeline, stereoQName, stereoLinkChoice, ...);
    }

    // 좌/우 렉티파이(왜곡 보정) 이미지도 별도 토픽으로 발행 가능
    // → /oak/left/image_rect, /oak/right/image_rect
    if(ph->getParam<bool>("i_left_rect_publish_topic")) {
        leftRectPub = setupOutput(pipeline, leftRectQName,
            [&](auto input) { stereoCamNode->rectifiedLeft.link(input); }, ...);
    }
    if(ph->getParam<bool>("i_right_rect_publish_topic")) {
        rightRectPub = setupOutput(pipeline, rightRectQName,
            [&](auto input) { stereoCamNode->rectifiedRight.link(input); }, ...);
    }
}


// ─── setupStereoQueue: 깊이 토픽 발행 설정 ───────────────────────
void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {

    // ── [핵심] TF 프레임 결정 ─────────────────────────────────
    // i_align_depth=true  → RGB 카메라 광학 프레임(oak_rgb_camera_optical_frame) 기준
    // i_align_depth=false → 오른쪽 모노 카메라 프레임 기준 (정렬 없음)
    std::string tfPrefix;
    if(ph->getParam<bool>("i_align_depth")) {
        // i_socket_name = "rgb" (CAM_A) → RGB 카메라와 같은 TF 프레임으로 발행
        tfPrefix = getOpticalTFPrefix(ph->getParam<std::string>("i_socket_name"));
    } else {
        tfPrefix = getOpticalTFPrefix(getSocketName(rightSensInfo.socket));
    }

    utils::ImgConverterConfig convConfig;
    convConfig.tfPrefix  = tfPrefix;
    convConfig.encoding  = dai::RawImgFrame::Type::RAW8; // 16비트 깊이를 RAW8로 전달
    convConfig.isStereo  = true;

    utils::ImgPublisherConfig pubConf;
    pubConf.topicName   = "~/" + getName();      // /oak/stereo
    pubConf.topicSuffix = "/image_raw";          // → /oak/stereo/image_raw
    // i_board_socket_id: 카메라 정보(camera_info) 발행 시 사용할 소켓
    pubConf.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));

    stereoPub->setup(device, convConfig, pubConf);
}


// ─── 핵심 정렬 파라미터 (YAML로 제어) ────────────────────────────
//
// 출처: stereo_param_handler.cpp
//
// [정렬 활성화]
//   i_align_depth: true
//     → StereoDepth.setDepthAlign(CAM_A) 호출
//     → 깊이 맵을 RGB 카메라 시점으로 재투영 (VPU 내부 처리)
//     → /oak/stereo/image_raw 가 RGB와 픽셀 단위로 1:1 대응됨
//
// [정렬 기준 소켓]
//   i_board_socket_id: 0  (0=CAM_A=RGB)
//     → setDepthAlign()에 전달할 소켓 ID
//
// [정밀도 향상]
//   i_subpixel: true
//     → 정수 픽셀이 아닌 소수점 시차 계산 → 4m 이상 거리 오차 감소
//   i_subpixel_fractional_bits: 3  (2^3=8 단계 소수점 정밀도)
//
// [좌우 일관성 검사]
//   i_lr_check: true
//     → 좌→우, 우→좌 시차를 비교해 불일치 픽셀을 무효화
//     → 물체 경계 노이즈 감소
//
// [근거리 확장]
//   i_extended_disp: false
//     → true 시 최대 시차 범위 두 배 → 0.5m 미만 근거리 가능
//     → subpixel과 동시 사용 불가
//
// ─── 정렬 결과 토픽 ──────────────────────────────────────────────
//
//  /oak/rgb/image_raw          → RGB 이미지 (정렬 기준)
//  /oak/stereo/image_raw       → RGB에 정렬된 깊이 맵 (픽셀 1:1 대응)
//  /oak/stereo/camera_info     → 깊이 카메라 내부 파라미터 (RGB와 동일)
//  /oak/points                 → RGB 색상 포인트 클라우드
//
// ─────────────────────────────────────────────────────────────────


// ─── link: 다른 노드에서 깊이/렉티파이 출력을 가져갈 때 사용 ────
void Stereo::link(dai::Node::Input in, int linkType) {
    if(linkType == StereoLinkType::stereo) {
        stereoCamNode->depth.link(in);          // 깊이 맵 출력
    } else if(linkType == StereoLinkType::left) {
        stereoCamNode->rectifiedLeft.link(in);  // 렉티파이된 왼쪽 이미지
    } else if(linkType == StereoLinkType::right) {
        stereoCamNode->rectifiedRight.link(in); // 렉티파이된 오른쪽 이미지
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
