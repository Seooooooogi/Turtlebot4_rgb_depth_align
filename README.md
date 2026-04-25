# Turtlebot4 RGB/Depth Align

OAK-D 카메라의 RGB 와 Depth 영상을 픽셀 단위로 정렬하여 ROS2 토픽으로 발행하는 커스텀 패키지.
TurtleBot4 의 벤더 OAK-D 노드(`depthai_ros_driver`) 를 대체하는 standalone ROS2 노드를 제공한다.

## 정렬 방식

1. **VPU 하드웨어 정렬** — `stereo->setDepthAlign(RGB_SOCKET)` 으로 OAK-D 의 Movidius VPU 가
   StereoDepth 출력을 RGB 카메라 시점으로 재투영. CPU 부하 0.
2. **RGB IspScale 다운스케일** — `camRgb->setIspScale(num, den)` 로 1920×1080 sensor 출력을
   ISP 단계에서 축소. 기본 `(2, 3)` → 1280×720, `(1, 3)` → 640×360.
3. **EEPROM 캘리브레이션 기반 undistort** — `cv::initUndistortRectifyMap` 사전 계산 +
   매 프레임 `cv::remap` 으로 RGB 렌즈 왜곡 보정. Depth 는 16UC1 (mm) 그대로 발행.

## 환경

| 항목 | 값 |
|------|-----|
| ROS2 | Humble |
| depthai (C++) | `/opt/ros/humble` 의 vendor 패키지 (`depthai_ros_driver` 와 공유) |
| 타겟 플랫폼 | PC (x86_64) + TurtleBot4 (Raspberry Pi 4, arm64) |
| 컴파일러 | C++17 |

---

## 설치

### 1. 저장소 클론

```bash
git clone https://github.com/Seooooooogi/Turtlebot4_rgb_depth_align.git ~/Turtlebot4_rgb_depth_align
cd ~/Turtlebot4_rgb_depth_align
```

### 2. OAK-D udev 룰 (최초 1회)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select oak_d_align_cpp --symlink-install
source install/setup.bash
```

---

## 실행

### PC standalone 실행

```bash
ros2 run oak_d_align_cpp oakd_sender \
  --ros-args --params-file install/oak_d_align_cpp/share/oak_d_align_cpp/config/oakd_params.yaml
```

### 토픽 확인

```bash
ros2 topic list | grep oakd
ros2 topic hz /oakd/rgb/image_raw/aligned/compressed
```

### 발행 토픽

`<robot_namespace>` 는 launch 의 `PushRosNamespace` 또는 `ROBOT_NAMESPACE` 환경변수에 따라
자동 prefix. PC standalone 실행 시엔 root namespace 라 prefix 없이 `/oakd/...` 로 발행.

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/<robot_namespace>/oakd/rgb/image_raw/aligned` | `sensor_msgs/Image` | undistort 된 RGB |
| `/<robot_namespace>/oakd/rgb/image_raw/aligned/compressed` | `sensor_msgs/CompressedImage` | RGB JPEG |
| `/<robot_namespace>/oakd/stereo/image_raw/aligned` | `sensor_msgs/Image` | RGB 시점 정렬된 Depth (16UC1, mm) |
| `/<robot_namespace>/oakd/stereo/image_raw/aligned/compressedDepth` | `sensor_msgs/CompressedImage` | Depth PNG |
| `/<robot_namespace>/oakd/overlay/compressed` | `sensor_msgs/CompressedImage` | RGB+Depth 블렌딩 (`enable_overlay: true` 일 때만) |

---

## 파라미터

`src/oak_d_align_cpp/config/oakd_params.yaml` 에서 수정. YAML 키는 노드 namespace 매칭을 위해
`/**/oakd_sender:` 와일드카드 사용.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fps` | `30.0` | 카메라 프레임레이트 |
| `stereo_preset` | `HIGH_DENSITY` | StereoDepth preset (`HIGH_DENSITY` \| `HIGH_ACCURACY`) |
| `stereo_subpixel` | `true` | Subpixel 활성 (depth 정밀도 향상) |
| `rgb_isp_num` / `rgb_isp_den` | `2` / `3` | IspScale 비율 — sensor 1920×1080 → `(num/den) × 1080`. `(1,3)` → 640×360 |
| `ir_dot_projector_intensity` | `0.0` | IR dot projector intensity (0.0~1.0). RPi4 는 USB 전력 한계로 Y-adapter 권장 |
| `ir_flood_intensity` | `0.0` | IR flood LED intensity |
| `enable_overlay` | `false` | RGB+Depth 블렌딩 토픽 발행 (CPU 부하 추가) |
| `overlay_alpha` | `0.5` | overlay 시 depth 비율 (0=RGB only, 1=Depth only) |
| `use_color_map` | `false` | Depth colorize 시 JET 컬러맵 (false=grayscale) |
| `jpeg_quality_overlay` | `80` | overlay JPEG 품질 |
| `roi_size` | `5` | 거리 측정 ROI 예약 키 (현재 미사용) |

---

## 패키지 구조

```
Turtlebot4_rgb_depth_align/
├── src/oak_d_align_cpp/
│   ├── config/oakd_params.yaml
│   ├── include/oak_d_align_cpp/oakd_sender.hpp
│   ├── launch/
│   │   ├── oakd_sender.launch.py            # PC standalone
│   │   └── turtlebot4_bringup.launch.py     # TurtleBot4 통합 (vendor robot/joy/rplidar/description + oakd_sender)
│   ├── src/
│   │   ├── oakd_sender.cpp                  # composable + standalone 공용
│   │   └── oakd_sender_main.cpp             # standalone entry
│   ├── systemd/override.conf                # turtlebot4.service drop-in
│   ├── CMakeLists.txt
│   └── package.xml
├── tools/overlay.py                         # RGB-Depth 정렬 시각 검증
└── README.md
```

---

## 알려진 이슈

- **Depth 정밀도 한계** — 4m 거리 측정 시 절대 오차 ~80cm + 시간 변동 ±20cm 잔존.
  근본 원인: OAK-D PRO 의 75mm baseline (장거리 ≥4m 부적합) + EEPROM calibration
  정밀도 한계. StereoDepth 후처리 필터 (median, confidence_threshold,
  subpixel_fractional_bits, speckle, threshold) 모두 파라미터로 노출되어 환경별
  활성 가능하나 default OFF — 활성 시 valid pixel 솎아내며 시각 align 평가가
  도리어 어려워지는 부작용 확인됨. 추가 향상은 (a) Y-adapter 후 IR projector,
  (b) Tier 0 fabrication 정책 결정 후 temporal filter, (c) 하드웨어 교체
  (OAK-D LR long baseline 150mm) 가 후보.
- **ComposableNodeContainer 적재 race** — TurtleBot4 launch 통합 중 일부 부팅에서
  `LoadComposableNodes` 가 silent 하게 미호출. 현재 `Node` action 으로 fallback.

---

## TurtleBot4 클린 설치 (`oak_d_align_cpp`)

C++ 패키지를 TurtleBot4 의 `turtlebot4.service` 자동 기동 체인에 통합. 벤더 unit
(`/lib/systemd/system/turtlebot4.service`) 과 launch 파일 (`/opt/ros/humble/share/turtlebot4_bringup/`)
은 직접 수정하지 않는다.

`<robot_IP>` 는 본인 TurtleBot4 의 IP, `<robot_namespace>` 는 `/etc/turtlebot4/setup.bash` 의
`ROBOT_NAMESPACE` 값으로 치환.

### 1. PC: 위 "설치" 단계 완료

### 2. TurtleBot4 워크스페이스 디렉터리 준비

```bash
ssh ubuntu@<robot_IP> 'mkdir -p ~/turtlebot_align_ws/src'
```

### 3. PC → TurtleBot4 소스 동기화

```bash
rsync -avh --exclude=build --exclude=install --exclude=log \
  ~/Turtlebot4_rgb_depth_align/src/oak_d_align_cpp/ \
  ubuntu@<robot_IP>:~/turtlebot_align_ws/src/oak_d_align_cpp/
```

### 4. TurtleBot4 측 arm64 빌드

PC ↔ 로봇 바이너리 비호환이라 로봇에서 native build 필요:
```bash
ssh ubuntu@<robot_IP> \
  'cd ~/turtlebot_align_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select oak_d_align_cpp --symlink-install'
```

### 5. systemd drop-in 설치

```bash
ssh ubuntu@<robot_IP> '\
  sudo install -d /etc/systemd/system/turtlebot4.service.d && \
  sudo install -m 0644 \
    ~/turtlebot_align_ws/install/oak_d_align_cpp/share/oak_d_align_cpp/systemd/override.conf \
    /etc/systemd/system/turtlebot4.service.d/override.conf'
```

drop-in 의 ExecStart 가 벤더 `turtlebot4-start` 를 우회하고
`ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py` 를 직접 실행한다.

### 6. 서비스 reload + restart

```bash
ssh ubuntu@<robot_IP> 'sudo systemctl daemon-reload && sudo systemctl restart turtlebot4.service'
```

### 7. 검증

**TurtleBot4 측 — 새 PID + 노드 가동 로그:**
```bash
ssh ubuntu@<robot_IP> \
  'systemctl show -p MainPID --value turtlebot4.service && \
   journalctl -u turtlebot4.service --since "30 seconds ago" --no-pager | grep -E "OAK-D|RGB output|RGB topic|Depth topic"'
```

기대 로그:
```
node namespace : /<robot_namespace>
RGB output     : 640x360 (IspScale 1/3)
RGB topic      : /<robot_namespace>/oakd/rgb/image_raw/aligned
Depth topic    : /<robot_namespace>/oakd/stereo/image_raw/aligned
```

**PC 측 — 토픽 발행률:**
```bash
source /etc/turtlebot4_discovery/setup.bash
ros2 topic hz /<robot_namespace>/oakd/rgb/image_raw/aligned/compressed
ros2 topic hz /<robot_namespace>/oakd/stereo/image_raw/aligned/compressedDepth
```

**PC 측 — 시각 검증:**
```bash
source /etc/turtlebot4_discovery/setup.bash
python3 tools/overlay.py --namespace <robot_namespace> --subpath oakd --aligned --compressed
```
"q" 키로 창 종료.
