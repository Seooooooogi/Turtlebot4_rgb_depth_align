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
| depthai | 2.28.0 (고정) |
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
├── requirements.txt           # depthai==2.28.0
├── package.xml
└── setup.py
```

---

## 알려진 이슈

- **사각지대(Hole)**: RGB 시점으로 Depth 재투영 시 Mono 카메라 사각 영역이 검은색으로 표시됨
- **Edge bleeding**: 물체 경계에서 Depth 값 번짐 — Spatial/Temporal/Speckle 필터 튜닝으로 완화 예정
- **4m 이상 오차**: 현재 약 1m 오차 발생 → Phase 2 후처리 필터 작업으로 개선 예정

---

## TurtleBot4 클린 설치 (`oak_d_align_cpp`)

C++ 포팅 패키지 (`src/oak_d_align_cpp`) 를 TurtleBot4 의 `turtlebot4.service` 자동 기동
체인에 통합하는 절차. 벤더 `oakd.launch.py` 는 systemd drop-in 으로 우회되며,
`/lib/systemd/system/turtlebot4.service` 와 `/opt/ros/humble/share/turtlebot4_bringup/`
는 직접 수정하지 않는다 (벤더 immutability).

### 전제 (vendor 셋업 완료 상태)

- TurtleBot4 네트워크/IP 셋업 완료 — 본 README 기준 `192.168.1.13`
- 벤더 `turtlebot4.service` 가 enabled (부팅 시 자동 기동)
- `/etc/turtlebot4_discovery/setup.bash` 존재 (PC 측 discovery server 환경)
- PC ↔ TurtleBot4 SSH 키 인증 가능 (`ssh ubuntu@192.168.1.13` 무비밀번호)
- 환경변수: `ROBOT_NAMESPACE=/robot9` (벤더 `/etc/turtlebot4/setup.bash`)

### 1. PC 측 — 저장소 클론 + 자체 빌드 검증

```bash
git clone https://github.com/Seooooooogi/Turtlebot4_rgb_depth_align.git ~/Turtlebot4_rgb_depth_align
cd ~/Turtlebot4_rgb_depth_align
source /opt/ros/humble/setup.bash
colcon build --packages-select oak_d_align_cpp --symlink-install
```

### 2. PC 측 — discovery 환경 alias

`~/.bashrc` 에 추가 (1회):
```bash
alias tb='source /etc/turtlebot4_discovery/setup.bash'
```

이후 PC 에서 로봇 토픽 구독 시 `tb` 입력 → `RMW_IMPLEMENTATION`,
`ROS_DISCOVERY_SERVER`, `ROS_DOMAIN_ID` 자동 주입.

### 3. TurtleBot4 측 — 워크스페이스 디렉터리 준비

```bash
ssh ubuntu@192.168.1.13 'mkdir -p ~/turtlebot_align_ws/src'
```

### 4. PC → TurtleBot4 — 소스 동기화

```bash
rsync -avh --exclude=build --exclude=install --exclude=log \
  ~/Turtlebot4_rgb_depth_align/src/oak_d_align_cpp/ \
  ubuntu@192.168.1.13:~/turtlebot_align_ws/src/oak_d_align_cpp/
```

### 5. TurtleBot4 측 — arm64 빌드

amd64 ↔ arm64 바이너리 비호환이라 로봇에서 별도 native build:
```bash
ssh ubuntu@192.168.1.13 \
  'cd ~/turtlebot_align_ws \
   && source /opt/ros/humble/setup.bash \
   && colcon build --packages-select oak_d_align_cpp --symlink-install'
```

### 6. TurtleBot4 측 — systemd drop-in 설치

벤더 unit 파일 (`/lib/systemd/system/turtlebot4.service`) 은 수정하지 않고,
drop-in 디렉터리에 override 만 추가:
```bash
ssh ubuntu@192.168.1.13 \
  'sudo install -d /etc/systemd/system/turtlebot4.service.d \
   && sudo install -m 0644 \
        ~/turtlebot_align_ws/install/oak_d_align_cpp/share/oak_d_align_cpp/systemd/override.conf \
        /etc/systemd/system/turtlebot4.service.d/override.conf'
```

drop-in 의 ExecStart 가 벤더 `turtlebot4-start` 를 우회하고
`ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py` 를 직접 실행한다.

### 7. TurtleBot4 측 — 서비스 reload + restart

```bash
ssh ubuntu@192.168.1.13 'sudo systemctl daemon-reload && sudo systemctl restart turtlebot4.service'
```

### 8. 검증

**TurtleBot4 측 — 새 PID + 노드 가동 확인:**
```bash
ssh ubuntu@192.168.1.13 \
  'systemctl show -p MainPID --value turtlebot4.service \
   && journalctl -u turtlebot4.service --since "30 seconds ago" --no-pager \
        | grep -E "OAK-D|RGB output|RGB topic|Depth topic"'
```

기대 로그:
```
node namespace : /robot9
RGB output     : 640x360 (IspScale 1/3)
RGB topic      : /robot9/oakd/rgb/image_raw/aligned
Depth topic    : /robot9/oakd/stereo/image_raw/aligned
```

**PC 측 — 토픽 발행률 확인 (`tb` 환경 필요):**
```bash
tb
ros2 topic hz /robot9/oakd/rgb/image_raw/aligned/compressed
ros2 topic hz /robot9/oakd/stereo/image_raw/aligned/compressedDepth
```

**PC 측 — 시각 검증 (overlay):**
```bash
tb
python3 tools/overlay.py --namespace robot9 --subpath oakd --aligned --compressed
```
"q" 키로 창 종료.

### 클린 재설치 (full reset)

기존 워크스페이스/드롭인 모두 제거 후 처음부터:
```bash
ssh ubuntu@192.168.1.13 '\
  sudo systemctl stop turtlebot4.service && \
  sudo rm -f /etc/systemd/system/turtlebot4.service.d/override.conf && \
  sudo systemctl daemon-reload && \
  rm -rf ~/turtlebot_align_ws/{build,install,log,src/oak_d_align_cpp}'
```
이후 4단계 (rsync) 부터 다시 수행.

### 트러블슈팅

| 증상 | 원인 / 조치 |
|------|-------------|
| `ros2 topic hz` 가 sample 0 (PC) | `tb` 미실행. discovery server 환경변수 미주입 상태. |
| 노드 로그에 `IspScale 2/3` 로 떨어짐 | YAML 키가 `oakd_sender:` 로 되어 있음 → `/**/oakd_sender:` 와일드카드 필요 (namespace 매칭). |
| `journalctl` 에 `Load Library` 미출현 | `ComposableNodeContainer` 적재 race. 현재 launch 는 `Node` action 으로 fallback 되어 해당 없음. |
| 벤더 `oakd.launch.py` 가 같이 뜸 | drop-in 미적용. `systemctl status turtlebot4.service` 의 `Drop-In:` 라인 확인. |
| `ros2 topic hz` 의 raw 토픽이 ~3 Hz | WiFi 대역폭 한계. compressed 경로 사용 권장. |
