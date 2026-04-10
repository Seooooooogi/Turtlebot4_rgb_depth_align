# Turtlebot4 RGB/Depth Align

OAK-D 카메라의 RGB 영상과 Depth 영상을 픽셀 단위로 정렬하여 ROS2 토픽으로 발행하는 커스텀 패키지.
TurtleBot4의 기존 OAK-D 패키지를 대체하는 것을 목표로 한다.

## 정렬 방식 (3단계)

1. `stereo.setDepthAlign(LEFT_SOCKET)` — VPU 하드웨어 1차 정렬
2. `dai.node.ImageAlign` — RGB 시점(640×360)으로 Depth 재투영
3. `cv2.undistort` — EEPROM 캘리브레이션 기반 RGB 렌즈 왜곡 보정

## 환경

| 항목 | 버전 |
|------|------|
| ROS2 | Humble |
| Python | 3.10+ |
| depthai | 2.23.0 (고정) |
| 타겟 플랫폼 | TurtleBot4 (Raspberry Pi 4) |

---

## 설치

### 1. 저장소 클론

```bash
git clone https://github.com/Seooooooogi/Turtlebot4_rgb_depth_align.git ~/turtlebot_align_ws
cd ~/turtlebot_align_ws
```

### 2. depthai 의존성 설치

```bash
pip install -r src/oak_d_align/requirements.txt
```

### 3. OAK-D udev 룰 설정 (최초 1회)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. 빌드

```bash
# bash
colcon build --symlink-install
source install/setup.bash

# zsh (oh-my-zsh)
colcon build --symlink-install
source install/setup.zsh
```

---

## 실행

### 기본 실행

```bash
ros2 launch oak_d_align oakd_sender.launch.py
```

정상 기동 로그:
```
[oakd_sender]: OAK-D 노드 가동 | namespace=robot9 | fps=30.0
```

### 토픽 확인

```bash
ros2 topic list | grep robot9
ros2 topic hz /robot9/oakd/rgb/image_raw/aligned/compressed
```

### 발행 토픽

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/robot9/oakd/rgb/image_raw/aligned/compressed` | `CompressedImage` | 왜곡 보정된 RGB + 중앙 거리 오버레이 |
| `/robot9/oakd/stereo/image_raw/aligned/compressed` | `CompressedImage` | 컬러라이즈된 Depth 맵 |
| `/robot9/oakd/overlay/compressed` | `CompressedImage` | RGB + Depth 블렌딩 오버레이 |

---

## 파라미터

`src/oak_d_align/config/oakd_params.yaml` 에서 수정.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `robot_namespace` | `robot9` | 토픽 prefix |
| `fps` | `30.0` | 카메라 프레임레이트 |
| `stereo_subpixel` | `true` | 서브픽셀 Depth 활성화 |
| `overlay_alpha` | `0.5` | Depth 블렌딩 비율 (0=RGB, 1=Depth) |
| `use_color_map` | `false` | Depth 컬러맵 (false=회색조) |
| `roi_size` | `20` | 중앙 거리 측정 ROI 크기 (px) |
| `jpeg_quality_rgb` | `85` | RGB JPEG 품질 |
| `jpeg_quality_depth` | `85` | Depth JPEG 품질 |
| `jpeg_quality_overlay` | `80` | Overlay JPEG 품질 |

---

## 패키지 구조

```
src/oak_d_align/
├── config/
│   └── oakd_params.yaml       # 파라미터 (하드코딩 금지)
├── launch/
│   └── oakd_sender.launch.py
├── oak_d_align/
│   └── oakd_sender_node.py    # 메인 노드
├── requirements.txt           # depthai==2.23.0
├── package.xml
└── setup.py
```

---

## 알려진 이슈

- **사각지대(Hole)**: RGB 시점으로 Depth 재투영 시 Mono 카메라 사각 영역이 검은색으로 표시됨
- **Edge bleeding**: 물체 경계에서 Depth 값 번짐 — Spatial/Temporal/Speckle 필터 튜닝으로 완화 예정
- **4m 이상 오차**: 현재 약 1m 오차 발생 → Phase 2 후처리 필터 작업으로 개선 예정
