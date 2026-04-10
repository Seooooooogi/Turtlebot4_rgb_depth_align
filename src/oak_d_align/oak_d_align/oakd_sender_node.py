import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import depthai as dai
import cv2
import numpy as np
from datetime import timedelta


class OakdSender(Node):
    def __init__(self):
        super().__init__('oakd_sender')

        # Parameters (all configurable via YAML — no hardcoding)
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

        # Device and calibration
        self.device = dai.Device()
        self.calibData = self.device.readCalibration()
        self.rgbDistortion = self.calibData.getDistortionCoefficients(self.RGB_SOCKET)

        if self.calibData.getDistortionModel(self.RGB_SOCKET) != dai.CameraModel.Perspective:
            self.get_logger().warn(
                "RGB 카메라가 Perspective 모델이 아닙니다. 왜곡 보정이 이상할 수 있습니다.")

        # Build and start pipeline
        self._build_pipeline(self.get_parameter('stereo_subpixel').value)
        self.device.startPipeline(self.pipeline)
        self.queue = self.device.getOutputQueue("out", 8, False)

        self.get_logger().info(
            f"OAK-D 노드 가동 | namespace={ns} | fps={self.FPS}")

        self.create_timer(0.001, self.run)

    def _build_pipeline(self, stereo_subpixel: bool):
        self.pipeline = dai.Pipeline()

        camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        sync = self.pipeline.create(dai.node.Sync)
        align = self.pipeline.create(dai.node.ImageAlign)
        out = self.pipeline.create(dai.node.XLinkOut)
        out.setStreamName("out")

        # Mono cameras
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setBoardSocket(self.LEFT_SOCKET)
        left.setFps(self.FPS)

        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setBoardSocket(self.RIGHT_SOCKET)
        right.setFps(self.FPS)

        # RGB camera: 1920x1080 -> 640x360 via IspScale
        camRgb.setBoardSocket(self.RGB_SOCKET)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(self.FPS)
        camRgb.setIspScale(1, 3)

        # Stereo depth: hardware 1st-pass alignment
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setDepthAlign(self.LEFT_SOCKET)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(stereo_subpixel)

        # Post-processing filters (disabled by default — tune via config)
        # Uncomment and move thresholds to YAML when ready to enable:
        #
        # config = stereo.initialConfig.get()
        # config.postProcessing.spatialFilter.enable = True
        # config.postProcessing.spatialFilter.holeFillingRadius = 2
        # config.postProcessing.spatialFilter.numIterations = 1
        # config.postProcessing.spatialFilter.alpha = 0.5
        # config.postProcessing.spatialFilter.delta = 20
        #
        # config.postProcessing.temporalFilter.enable = True
        # config.postProcessing.temporalFilter.alpha = 0.4
        # config.postProcessing.temporalFilter.delta = 20
        # config.postProcessing.temporalFilter.persistencyMode = (
        #     dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4)
        #
        # config.postProcessing.speckleFilter.enable = True
        # config.postProcessing.speckleFilter.speckleRange = 50
        #
        # stereo.initialConfig.set(config)

        # Sync: match RGB + aligned depth frames
        sync.setSyncThreshold(timedelta(seconds=0.5 / self.FPS))

        # Link nodes
        camRgb.isp.link(sync.inputs["rgb"])
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        stereo.depth.link(align.input)
        camRgb.isp.link(align.inputAlignTo)
        align.outputAligned.link(sync.inputs["depth_aligned"])

        sync.out.link(out.input)

    def colorize_depth(self, frameDepth):
        invalidMask = frameDepth == 0
        try:
            minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
            maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)

            out_array = np.zeros_like(frameDepth, dtype=np.float32)
            logDepth = np.log(
                frameDepth.astype(np.float32), out=out_array, where=(frameDepth != 0))

            logMinDepth = np.log(minDepth)
            logMaxDepth = np.log(maxDepth)
            np.nan_to_num(logDepth, copy=False, nan=logMinDepth)

            logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)
            depthFrameColor = np.interp(
                logDepth, (logMinDepth, logMaxDepth), (0, 255)).astype(np.uint8)

            if self.use_color_map:
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
            else:
                depthFrameColor = cv2.cvtColor(depthFrameColor, cv2.COLOR_GRAY2BGR)

            depthFrameColor[invalidMask] = 0

        except IndexError:
            depthFrameColor = np.zeros(
                (frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
        except Exception as e:
            self.get_logger().error(f"Depth colorization error: {e}")
            depthFrameColor = np.zeros(
                (frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)

        return depthFrameColor

    def run(self):
        msgGroup = self.queue.tryGet()
        if msgGroup is None:
            return

        frameRgb = msgGroup["rgb"]
        frameDepth = msgGroup["depth_aligned"]

        if frameRgb is None or frameDepth is None:
            return

        stamp = self.get_clock().now().to_msg()

        cvFrameRgb = frameRgb.getCvFrame()
        cvFrameDepth = frameDepth.getFrame()  # uint16, mm

        # RGB undistort (EEPROM calibration)
        rgbIntrinsics = self.calibData.getCameraIntrinsics(
            self.RGB_SOCKET, int(cvFrameRgb.shape[1]), int(cvFrameRgb.shape[0]))
        cvFrameUndistorted = cv2.undistort(
            cvFrameRgb, np.array(rgbIntrinsics), np.array(self.rgbDistortion))

        # Center distance measurement
        h, w = cvFrameDepth.shape
        cx, cy = w // 2, h // 2
        r = self.roi_size
        roi = cvFrameDepth[max(0, cy - r):min(h, cy + r),
                           max(0, cx - r):min(w, cx + r)]
        valid_depths = roi[roi > 0]

        if len(valid_depths) > 0:
            distance_m = float(np.median(valid_depths)) / 1000.0
            text = f"{distance_m:.2f} m"
            color = (0, 255, 0)
        else:
            text = "Out of Range"
            color = (0, 0, 255)
            distance_m = None

        self.get_logger().debug(f"중앙 거리: {text} (px {cx},{cy})")

        cv2.line(cvFrameUndistorted, (cx - 15, cy), (cx + 15, cy), color, 2)
        cv2.line(cvFrameUndistorted, (cx, cy - 15), (cx, cy + 15), color, 2)
        cv2.putText(cvFrameUndistorted, text, (cx + 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

        # Publish RGB
        msg_rgb = CompressedImage()
        msg_rgb.header.stamp = stamp
        msg_rgb.header.frame_id = "oakd_aligned_frame"
        msg_rgb.format = "jpeg"
        msg_rgb.data = np.array(
            cv2.imencode('.jpg', cvFrameUndistorted,
                         [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_rgb])[1]).tobytes()
        self.rgb_pub.publish(msg_rgb)

        # Publish depth
        alignedDepthColorized = self.colorize_depth(cvFrameDepth)
        msg_d = CompressedImage()
        msg_d.header.stamp = stamp
        msg_d.header.frame_id = "oakd_aligned_frame"
        msg_d.format = "jpeg"
        msg_d.data = np.array(
            cv2.imencode('.jpg', alignedDepthColorized,
                         [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_depth])[1]).tobytes()
        self.depth_pub.publish(msg_d)

        # Publish overlay
        if cvFrameUndistorted.shape[:2] != alignedDepthColorized.shape[:2]:
            alignedDepthColorized = cv2.resize(
                alignedDepthColorized,
                (cvFrameUndistorted.shape[1], cvFrameUndistorted.shape[0]))

        cvFrameOverlay = cv2.addWeighted(
            cvFrameUndistorted, 1 - self.alpha, alignedDepthColorized, self.alpha, 0)

        msg_overlay = CompressedImage()
        msg_overlay.header.stamp = stamp
        msg_overlay.header.frame_id = "oakd_aligned_frame"
        msg_overlay.format = "jpeg"
        msg_overlay.data = np.array(
            cv2.imencode('.jpg', cvFrameOverlay,
                         [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_overlay])[1]).tobytes()
        self.overlay_pub.publish(msg_overlay)


def main():
    rclpy.init()
    node = OakdSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
