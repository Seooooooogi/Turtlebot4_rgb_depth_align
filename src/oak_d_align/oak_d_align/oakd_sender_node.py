import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import depthai as dai
import cv2
import numpy as np

class OakdSender(Node):
    def __init__(self):
        super().__init__('oakd_sender')
        
        # device를 먼저 None으로 선언하여, 생성 실패 시에도 에러가 나지 않게 방어
        self.device = None 

        # Parameters
        self.declare_parameter('robot_namespace', 'robot9')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('stereo_subpixel', True)
        self.declare_parameter('overlay_alpha', 0.5)
        self.declare_parameter('use_color_map', False)
        self.declare_parameter('roi_size', 20)
        self.declare_parameter('jpeg_quality_rgb', 85)
        self.declare_parameter('jpeg_quality_depth', 85)
        self.declare_parameter('jpeg_quality_overlay', 80)

        ns = self.get_parameter('robot_namespace').value
        self.FPS = self.get_parameter('fps').value
        self.alpha = self.get_parameter('overlay_alpha').value
        self.use_color_map = self.get_parameter('use_color_map').value
        self.roi_size = self.get_parameter('roi_size').value
        self.jpeg_quality_rgb = self.get_parameter('jpeg_quality_rgb').value
        self.jpeg_quality_depth = self.get_parameter('jpeg_quality_depth').value
        self.jpeg_quality_overlay = self.get_parameter('jpeg_quality_overlay').value

        # Publishers
        self.rgb_pub = self.create_publisher(
            CompressedImage, f'/{ns}/oakd/rgb/image_raw/aligned/compressed', 10)
        self.depth_pub = self.create_publisher(
            CompressedImage, f'/{ns}/oakd/stereo/image_raw/aligned/compressed', 10)
        self.overlay_pub = self.create_publisher(
            CompressedImage, f'/{ns}/oakd/overlay/compressed', 10)

        # Hardware socket constants
        self.RGB_SOCKET = dai.CameraBoardSocket.CAM_A
        self.LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
        self.RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C

        # ---------------------------------------------------------
        # [구조 변경] 1. 파이프라인을 가장 먼저 완성합니다.
        # ---------------------------------------------------------
        self._build_pipeline(self.get_parameter('stereo_subpixel').value)

        # ---------------------------------------------------------
        # [구조 변경] 2. 디바이스를 켤 때 파이프라인을 통째로 밀어넣어 한 방에 부팅합니다. (안정성 극대화)
        # ---------------------------------------------------------
        self.device = dai.Device(self.pipeline)

        # ---------------------------------------------------------
        # [구조 변경] 3. 캘리브레이션 데이터는 디바이스가 켜진 직후에 읽어옵니다.
        # ---------------------------------------------------------
        self.calibData = self.device.readCalibration()
        self.rgbDistortion = self.calibData.getDistortionCoefficients(self.RGB_SOCKET)

        if self.calibData.getDistortionModel(self.RGB_SOCKET) != dai.CameraModel.Perspective:
            self.get_logger().warn("RGB 카메라가 Perspective 모델이 아닙니다. 왜곡 보정이 이상할 수 있습니다.")

        # 큐 설정
        self.rgb_queue = self.device.getOutputQueue("rgb", 4, False)
        self.depth_queue = self.device.getOutputQueue("depth", 4, False)

        # undistort 맵 사전 계산 (매 프레임 getCameraIntrinsics 호출 제거)
        # IspScale(1,3): 1920x1080 -> 640x360
        rgb_w, rgb_h = 1920 // 3, 1080 // 3
        rgb_K = np.array(self.calibData.getCameraIntrinsics(self.RGB_SOCKET, rgb_w, rgb_h))
        rgb_D = np.array(self.rgbDistortion)
        self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
            rgb_K, rgb_D, None, rgb_K, (rgb_w, rgb_h), cv2.CV_16SC2)

        self.get_logger().info(f"OAK-D 노드 가동 완료 | namespace={ns} | fps={self.FPS}")

        # 카메라 루프를 별도 스레드로 분리 — ROS2 executor와 독립적으로 동작
        self._stop_event = threading.Event()
        self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._camera_thread.start()

    def _build_pipeline(self, stereo_subpixel: bool):
        self.pipeline = dai.Pipeline()

        camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        align = self.pipeline.create(dai.node.ImageAlign)
        outRgb = self.pipeline.create(dai.node.XLinkOut)
        outDepth = self.pipeline.create(dai.node.XLinkOut)
        outRgb.setStreamName("rgb")
        outDepth.setStreamName("depth")

        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setBoardSocket(self.LEFT_SOCKET)
        left.setFps(self.FPS)

        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setBoardSocket(self.RIGHT_SOCKET)
        right.setFps(self.FPS)

        camRgb.setBoardSocket(self.RGB_SOCKET)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(self.FPS)
        camRgb.setIspScale(1, 3)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.setDepthAlign(self.LEFT_SOCKET)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(stereo_subpixel)

        camRgb.isp.link(outRgb.input)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        stereo.depth.link(align.input)
        camRgb.isp.link(align.inputAlignTo)
        align.outputAligned.link(outDepth.input)

    def _camera_loop(self):
        while not self._stop_event.is_set():
            try:
                frameRgb = self.rgb_queue.get()
                frameDepth = self.depth_queue.get()
                self.run(frameRgb, frameDepth)
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"Camera loop error: {e}")

    def colorize_depth(self, frameDepth):
        invalidMask = frameDepth == 0
        try:
            minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
            maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)

            out_array = np.zeros_like(frameDepth, dtype=np.float32)
            logDepth = np.log(frameDepth.astype(np.float32), out=out_array, where=(frameDepth != 0))

            logMinDepth = np.log(minDepth)
            logMaxDepth = np.log(maxDepth)
            np.nan_to_num(logDepth, copy=False, nan=logMinDepth)

            logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)
            depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255)).astype(np.uint8)

            if self.use_color_map:
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
            else:
                depthFrameColor = cv2.cvtColor(depthFrameColor, cv2.COLOR_GRAY2BGR)

            depthFrameColor[invalidMask] = 0

        except Exception as e:
            depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
            
        return depthFrameColor

    def run(self, frameRgb, frameDepth):
        stamp = self.get_clock().now().to_msg()
        cvFrameRgb = frameRgb.getCvFrame()
        cvFrameDepth = frameDepth.getFrame()

        cvFrameUndistorted = cv2.remap(
            cvFrameRgb, self.undistort_map1, self.undistort_map2, cv2.INTER_LINEAR)

        h, w = cvFrameDepth.shape
        cx, cy = w // 2, h // 2
        r = self.roi_size
        roi = cvFrameDepth[max(0, cy - r):min(h, cy + r), max(0, cx - r):min(w, cx + r)]
        valid_depths = roi[roi > 0]

        if len(valid_depths) > 0:
            distance_m = float(np.median(valid_depths)) / 1000.0
            text = f"{distance_m:.2f} m"
            color = (0, 255, 0)
        else:
            text = "Out of Range"
            color = (0, 0, 255)

        cv2.line(cvFrameUndistorted, (cx - 15, cy), (cx + 15, cy), color, 2)
        cv2.line(cvFrameUndistorted, (cx, cy - 15), (cx, cy + 15), color, 2)
        cv2.putText(cvFrameUndistorted, text, (cx + 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        msg_rgb = CompressedImage()
        msg_rgb.header.stamp = stamp
        msg_rgb.header.frame_id = "oakd_aligned_frame"
        msg_rgb.format = "jpeg"
        msg_rgb.data = np.array(cv2.imencode('.jpg', cvFrameUndistorted, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_rgb])[1]).tobytes()
        self.rgb_pub.publish(msg_rgb)

        alignedDepthColorized = self.colorize_depth(cvFrameDepth)
        msg_d = CompressedImage()
        msg_d.header.stamp = stamp
        msg_d.header.frame_id = "oakd_aligned_frame"
        msg_d.format = "jpeg"
        msg_d.data = np.array(cv2.imencode('.jpg', alignedDepthColorized, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_depth])[1]).tobytes()
        self.depth_pub.publish(msg_d)

        if cvFrameUndistorted.shape[:2] != alignedDepthColorized.shape[:2]:
            alignedDepthColorized = cv2.resize(alignedDepthColorized, (cvFrameUndistorted.shape[1], cvFrameUndistorted.shape[0]))

        cvFrameOverlay = cv2.addWeighted(cvFrameUndistorted, 1 - self.alpha, alignedDepthColorized, self.alpha, 0)
        msg_overlay = CompressedImage()
        msg_overlay.header.stamp = stamp
        msg_overlay.header.frame_id = "oakd_aligned_frame"
        msg_overlay.format = "jpeg"
        msg_overlay.data = np.array(cv2.imencode('.jpg', cvFrameOverlay, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_overlay])[1]).tobytes()
        self.overlay_pub.publish(msg_overlay)

def main():
    rclpy.init()
    node = None
    try:
        node = OakdSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt 감지. 안전 종료를 시작합니다...")
    except Exception as e:
        print(f"[ERROR] 파이프라인 생성 또는 실행 중 치명적 에러 발생: {e}")
    finally:
        if node is not None:
            if hasattr(node, '_stop_event'):
                node._stop_event.set()
            if hasattr(node, '_camera_thread'):
                node._camera_thread.join(timeout=2.0)

            if getattr(node, 'device', None) is not None:
                node.device.close()
                print("[INFO] OAK-D 디바이스 자물쇠 해제 완료.")
            node.destroy_node()
            
        if rclpy.ok():
            rclpy.shutdown()
        
        print("[INFO] 노드가 완전히 종료되었습니다.")
        sys.exit(0)

if __name__ == '__main__':
    main()
