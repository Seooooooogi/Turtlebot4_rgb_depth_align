# OAK-D Pro RGB/Depth Alignment — 드라이버 3종 비교 분석

> 측정 환경: OAK-D Pro (RVC2, USB3 SuperSpeed), PC (x86_64), ROS2 Humble  
> 측정일: 2026-04-13  
> 참고: TurtleBot4 내장 드라이버(arm64)는 PC 실행 불가 → 코드 분석만 수행

---

## 1. 드라이버 개요

| 항목 | TurtleBot4 내장 드라이버 | 빌드한 C++ 드라이버 | Python 커스텀 노드 |
|------|--------------------------|---------------------|-------------------|
| **패키지** | `depthai_ros_driver` | `depthai_ros_driver` | `oak_d_align` |
| **버전** | 2.11.2 (arm64 바이너리) | 2.12.2 (소스 빌드) | 커스텀 |
| **언어** | C++ | C++ | Python (rclpy) |
| **아키텍처** | arm64 (RPi4 전용) | x86_64 / arm64 | x86_64 / arm64 |
| **실행 가능 (PC)** | ❌ | ✅ | ✅ |

---

## 2. RGB/Depth Alignment 구현 방식

### 2-1. TurtleBot4 내장 드라이버 (v2.11.2)

**실제 적용 설정** (`turtlebot4_bringup/config/oakd_pro.yaml` — 수정 후 상태):

```yaml
camera:
  i_pipeline_type: RGBD       # Depth 파이프라인 활성
  i_nn_type: none
  i_enable_ir: false           # IR dot projector 비활성 (brightness 설정값 무시됨)
  i_laser_dot_brightness: 100  # i_enable_ir: false로 인해 실제 미적용
rgb:
  i_resolution: '1080'         # 센서 1080P → IspScale(2/3) 자동 적용 → ISP 출력 1280×720
  i_width: 640                 # 발행 크기 오버라이드 → 640×480
  i_height: 480
  i_publish_topic: true
stereo:                        # depth 활성화를 위해 추가된 섹션
  i_width: 640                 # rgb.i_width 읽어 동일하게 맞춤
  i_height: 480
  i_align_depth: true          # CAM_A(RGB) 기준 정렬 활성
  i_depth_preset: HIGH_ACCURACY
  i_subpixel: true
  i_lr_check: true
depth:
  enabled: true
  i_publish_topic: true
```

```
CAM_B (Left Mono) ──┐
                     ├──► StereoDepth [setDepthAlign(CAM_A)] ──► /oakd/stereo/.. (640×480)
CAM_C (Right Mono) ──┘
CAM_A (RGB, 1080P→IspScale→640×480) ────────────────────────► /oakd/rgb/..     (640×480)
```

**정렬 구성 분석**:

Stereo 노드는 `rgb.i_width = 640`을 읽어 depth 출력도 640×480으로 맞추므로, 픽셀 수는 일치합니다. `i_align_depth: true` + 동일 해상도로 **기본 정렬은 작동**합니다.

**잠재적 문제 — `camera_info` K 행렬 불일치**:

| 단계 | 해상도 | 설명 |
|------|--------|------|
| 센서 원본 | 1920×1080 | `i_resolution: '1080'` |
| ISP 출력 | 1280×720 | IspScale(2/3) 자동 적용 |
| 발행 이미지 | 640×480 | `i_width: 640` YAML 오버라이드 |
| camera_info K | 1280×720 기준 계산 | ← 발행 이미지(640×480)와 불일치 |

`camera_info`의 내부 파라미터(K 행렬)가 1280×720 기준으로 계산되지만 실제 이미지는 640×480으로 발행되면, PointCloud 생성이나 3D 재투영 시 스케일 오차가 발생합니다.

**IR 비활성 영향**:  
`i_enable_ir: false`로 IR dot projector가 꺼져 있어, 텍스처 없는 표면(벽, 바닥)에서 depth 오차가 큽니다. `i_laser_dot_brightness: 100` 설정값은 `i_enable_ir: false`로 인해 무시됩니다.

---

### 2-2. 빌드한 C++ 드라이버 (v2.12.2)

```
CAM_B (Left Mono, 800P) ──┐
                            ├──► StereoDepth [setDepthAlign(CAM_A)] ──► /oak/stereo/image_raw (1280×720)
CAM_C (Right Mono, 800P) ──┘
CAM_A (RGB, IspScale 2/3) ────────────────────────────────────────► /oak/rgb/image_raw (1280×720)
```

**핵심 차이점**:
- `IspScale(2, 3)` 자동 적용 → RGB 1920×1080 → **1280×720** 축소 (내장 드라이버와 달리)
- Mono 해상도 **800P (1280×800)** 사용 → Depth 정밀도 향상
- RGB와 Depth **동일 해상도(1280×720)** + **동일 TF 프레임**(`oak_rgb_camera_optical_frame`) → 픽셀 1:1 대응 보장
- PointCloud(`/oak/points`) 추가 제공

---

### 2-3. Python 커스텀 노드 (`oakd_sender_node.py`)

```
CAM_B (Left Mono, 800P) ──┐
                            ├──► StereoDepth [setDepthAlign(CAM_A)] ──► /robot9/oakd/stereo/.. (JPEG)
CAM_C (Right Mono, 800P) ──┘
CAM_A (RGB, IspScale 2/3) ────────────────────────────────────────► /robot9/oakd/rgb/..     (JPEG)
                                                                   ► /robot9/oakd/overlay/.. (JPEG)
```

**CPU 후처리 단계** (VPU 이후 Python에서 처리):
1. `cv2.remap()` — RGB 왜곡 보정 (undistort)
2. `colorize_depth()` — 깊이 log-scale 시각화 (NumPy)
3. `cv2.addWeighted()` — RGB + Depth overlay 생성
4. JPEG 인코딩 × 3 (rgb / depth / overlay)

---

## 3. 성능 비교 (PC 실측)

### 3-1. fps

| 드라이버 | RGB fps | Depth fps | 비고 |
|----------|---------|-----------|------|
| TurtleBot4 내장 | — | — | arm64 전용, 측정 불가 |
| C++ 빌드 (v2.12.2) | **30 fps** (안정) | **30 fps** (안정화 후) | 초기 수 초간 불안정 |
| Python 커스텀 | **~15 fps** | **~15 fps** | CPU 후처리 병목 |

### 3-2. 해상도

| 드라이버 | RGB | Depth (Mono 기반) | Depth 해상도 | 픽셀 정렬 |
|----------|-----|-------------------|--------------|-----------|
| TurtleBot4 내장 | 640×480 (1080P→IspScale→오버라이드) | 기본 해상도 | 640×480 | ✅ 일치 (단, camera_info K 불일치 가능) |
| C++ 빌드 | 1280×720 | 800P (1280×800) | 1280×720 | ✅ 완전 일치 |
| Python 커스텀 | 1280×720 (JPEG) | 800P (1280×800) | 1280×720 | ✅ 완전 일치 |

### 3-3. IR Dot Projector

| 드라이버 | 기본값 | 단위 | 실효 강도 |
|----------|--------|------|-----------|
| TurtleBot4 내장 | `i_laser_dot_brightness: 800` | mA (0~1200) | **67%** |
| C++ 빌드 | `i_laser_dot_brightness: 800` | mA (0~1200) | **67%** |
| Python 커스텀 | `ir_dot_projector_intensity: 0.667` | 비율 (0~1) | **67%** ← 맞춤 |

### 3-4. 발행 토픽

| 토픽 | TurtleBot4 내장 | C++ 빌드 | Python 커스텀 |
|------|-----------------|----------|---------------|
| RGB 이미지 | `/oak/rgb/image_raw` (raw) | `/oak/rgb/image_raw` (raw) | `/{ns}/oakd/rgb/.../compressed` (JPEG) |
| Depth 이미지 | `/oak/stereo/image_raw` (raw 16bit) | `/oak/stereo/image_raw` (raw 16bit) | `/{ns}/oakd/stereo/.../compressed` (JPEG) |
| RGB 왜곡보정 | `/oak/rgb/image_rect` | `/oak/rgb/image_rect` | ✅ 내장 (cv2.remap) |
| Overlay | ❌ | ❌ | ✅ `/{ns}/oakd/overlay/compressed` |
| PointCloud | `/oak/points` | `/oak/points` | ❌ |
| IMU | `/oak/imu/data` | `/oak/imu/data` | ❌ |
| 압축 토픽 | ✅ (image_transport) | ✅ (image_transport) | JPEG만 |

---

## 4. 핵심 차이 요약

```
┌─────────────────────────────────────────────────────────────────┐
│  TurtleBot4 내장 (v2.11.2)                                      │
│  RGB 640×480 ────┐  ← 1080P→IspScale→YAML 오버라이드           │
│  Depth 640×480 ──┘  ← i_align_depth: true, 해상도 일치         │
│  → 기본 정렬 ✅, IR 비활성 ⚠️, camera_info K 불일치 가능 ⚠️   │
├─────────────────────────────────────────────────────────────────┤
│  C++ 빌드 (v2.12.2)                                              │
│  RGB 1280×720 ───┐  ← IspScale(2,3) 자동 적용                  │
│  Depth 1280×720 ─┘  ← 동일 TF 프레임, 동일 해상도              │
│  → 픽셀 1:1 대응 ✅, PointCloud ✅, 30fps ✅                    │
├─────────────────────────────────────────────────────────────────┤
│  Python 커스텀                                                   │
│  RGB 1280×720 ───┐  ← IspScale(2,3) + cv2.remap(undistort)    │
│  Depth 1280×720 ─┘  ← setDepthAlign(CAM_A) 단일 정렬          │
│  → 픽셀 1:1 대응 ✅, Overlay ✅, ~15fps ⚠️ (CPU 병목)          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. 결론 및 TurtleBot4 통합 방향

### 내장 드라이버 실제 상태 (스크린샷 기준)
- **Depth 파이프라인 활성**: `i_pipeline_type: RGBD` + `stereo`, `depth` 섹션 추가로 depth 작동 중
- **기본 정렬 구성됨**: `i_align_depth: true` + RGB/Depth 동일 640×480 → 픽셀 수 일치
- **IR 비활성**: `i_enable_ir: false` → 텍스처 없는 표면 depth 오차 큼 (벽, 바닥)
- **camera_info 불일치 가능성**: `i_resolution: '1080'` + `i_width: 640` 혼용으로 K 행렬 스케일 오차
- **해상도 낮음**: 640×480 → C++ 빌드(1280×720), Python 커스텀(1280×720) 대비 해상도 절반

### 권장 방향
| 시나리오 | 권장 방법 |
|----------|-----------|
| RPi4 배포, 낮은 CPU 부하 | C++ 빌드 드라이버 + IspScale(2,3) 설정 |
| 실시간 overlay 필요 | Python 커스텀 노드 (fps 제약 있음) |
| PointCloud + Navigation | C++ 드라이버 (`/oak/points` 활용) |
| 완전한 정렬 보장 | ImageAlign 노드 추가 (미구현 — Phase 3-C 과제) |

> Phase 3-C에서 C++ 드라이버를 TurtleBot4 bringup에 통합할 때,  
> `i_isp_num: 2`, `i_isp_den: 3` 파라미터를 명시적으로 설정하여  
> 내장 드라이버의 해상도 불일치 문제를 원천 차단해야 함.
