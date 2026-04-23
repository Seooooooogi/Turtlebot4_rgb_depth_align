# Architecture Decision Records

결정 시 항상 여기에 기록. 추가 조건: 새 의존성 추가, 기존 패턴 교체, 데이터 파이프라인 변경, 노드 구조 재편.

## Template
```markdown
# [Decision Title]
## Context: 왜 이 결정이 필요한가
## Decision: 무엇을 선택했는가
## Consequences: 트레이드오프, 알려진 제약
```

---

## ADR-001: Initial Stack & Architecture Decisions

**Context**: `/project-init` 인터뷰에서 결정된 초기 스택 및 Hard Rules.

**Decision**:
- Language: Python (depthai + rclpy + OpenCV)
- Interface: ROS2 노드 (백그라운드 서비스)
- Deployment: TurtleBot4 Raspberry Pi 4 로컬
- AI/LLM: 영구 제외 (RPi4 부하 제약)

**Hard Rules origin**:
- `vendor immutability`: 기존 `turtlebot4_bringup` 수정 시 업스트림 업데이트로 덮어써질 위험
- `dry-run first`: 실 로봇 테스트 비용(이동, 재부팅)이 크므로 PC 사전 검증 필수
- `no RPi CPU load`: RPi4는 11fps도 벅참 — 모든 이미지 연산은 OAK-D VPU로

---

## ADR-002: 3단계 RGB-Depth 정렬 방식 채택

**Context**: RGB(1080P, 16:9)와 Depth(800P, 16:10)의 해상도·시점 불일치.

**Decision**:
1. `stereo.setDepthAlign(LEFT_SOCKET)` — VPU 1차 하드웨어 정렬
2. `dai.node.ImageAlign` → 640x360 공통 해상도로 재투영
3. `cv2.undistort` (EEPROM 캘리브레이션 데이터) — RGB 렌즈 왜곡 보정

**Consequences**:
- 재투영 과정에서 Mono 카메라 사각지대 → 검은 구멍 발생 (known issue)
- Edge bleeding 존재 → Phase 2 후처리 필터로 완화 예정
- 4m 이상 거리에서 1m 오차 → 스펙 한계 + 필터 미적용이 원인으로 추정

**Superseded**: 이후 ADR-004 에서 1단계 `StereoDepth.setDepthAlign(CAM_A)` 로 단순화. ImageAlign/cv2.undistort 단계는 C++ 드라이버와의 동작 통일을 위해 제거됨.

---

## ADR-003: `oak_d_align_cpp` C++ 포팅 결정

**Context**:
- Python 노드(`src/oak_d_align/`)의 CPU 후처리(cv2.remap + colorize + JPEG 인코딩 × 3)가 병목 — PC 실측 ~15 fps, RPi4 배포 시 2 fps 수준으로 추정.
- `depthai_ros_driver` v2.12.2 가 동일 VPU 정렬을 제공하나, 커스텀 파이프라인/파라미터 제어권을 유지하고 싶음.
- TurtleBot4 bringup 에 composable node 로 통합해야 함 (Phase 3-C).

**Decision**:
- 신규 패키지 `src/oak_d_align_cpp/` (ament_cmake) 생성, Python 노드를 C++ 로 1:1 포팅.
- Python 패키지는 **존치** (호환성·롤백). 두 패키지 공존 허용.
- 아키텍처 요점:
  - 파이프라인: `ColorCamera(1080P, IspScale 2/3) + 2×MonoCamera(800P) + StereoDepth(setDepthAlign CAM_A)` — VPU 처리 유지.
  - CPU 후처리: `cv::remap` (undistort) + image_transport publishing (JPEG 인코딩을 `compressed_image_transport` 플러그인으로 위임).
  - Overlay 스트림: `enable_overlay: false` 기본값, 검증 시에만 토글.
  - Composable node 등록 (`RCLCPP_COMPONENTS_REGISTER_NODE`).

**Measured**:
| 항목 | Python (raw aligned) | C++ (compressed aligned) |
|------|---------------------|--------------------------|
| PC 평균 fps | ~15 | **~28.94** (window=205, stddev 5.4ms) |
| CPU 후처리 | cv2.remap + colorize + JPEG ×3 | cv::remap + image_transport zero-copy |
| 정렬 품질 | OK | **동등** (docs/overlay_cpp.png 와 docs/overlay.png 비교) |

**Consequences**:
- fps 약 2× 개선 (PC 실측). RPi4 배포 시 기대 개선폭은 더 큼 (Python 대비 C++ GIL/NumPy 오버헤드 제거 효과가 ARM에서 상대적으로 크기 때문).
- Python/C++ 두 구현 유지 → 파라미터 스키마는 동일하게 유지했으나 변경 시 두 곳 동기화 부담 존재.
- image_transport compressed 자동 발행에 의존 → `compressed_image_transport` 플러그인 설치 전제.
- TurtleBot4 배포는 arm64 에서 재빌드 필요 (바이너리 호환 없음).

---
