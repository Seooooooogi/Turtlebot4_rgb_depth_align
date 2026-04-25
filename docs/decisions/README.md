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

## ADR-004: TurtleBot4 bringup 통합 — composable 토폴로지 / systemd / YAML 입력

**Context** (Phase 3-C):
- `oak_d_align_cpp`을 TurtleBot4 `turtlebot4.service` 자동 기동 체인에 합류시키되, 벤더 `oakd.launch.py` 는 제외해야 함.
- 벤더 immutability 제약: `/opt/ros/humble/share/turtlebot4_bringup/`, `/lib/systemd/system/turtlebot4.service` 직접 수정 금지.
- 브리프가 제시한 composable 적재 옵션: (a) `depthai_ros_driver` container 합류, (b) 자체 container, (c) standalone.

**Decision**:

1. **Composable 적재 — 옵션 (b) 자체 container**
   - 이유: 벤더 `oakd.launch.py` 를 제외하면 `depthai_ros_driver` 가 띄우는 container 자체가 존재하지 않으므로 (a) 는 구조적으로 적용 불가. (a) 시도→실패 가 아니라 *전제 자체가 성립 안 함*.
   - `oak_d_align_cpp` 의 `OakdSender` 컴포넌트를 `oakd_align_container` 라는 신규 `ComposableNodeContainer` 에 단독 적재. `extra_arguments=[{'use_intra_process_comms': True}]` 로 향후 동일 container 노드 추가 시 zero-copy 가능.
   - (c) standalone 은 image_transport publisher 의 inter-process 시리얼라이즈 비용을 다시 안고 가야 하므로 후순위.

2. **Custom launch — `src/oak_d_align_cpp/launch/turtlebot4_bringup.launch.py`**
   - 벤더 `standard.launch.py` 구조(`PushRosNamespace` + `GroupAction` + 5 `IncludeLaunchDescription` + 조건부 `diagnostics`)를 그대로 미러링하되, `oakd_launch_file` include 만 `oakd_align_container` 로 교체.
   - 환경 변수 `ROBOT_NAMESPACE`, `TURTLEBOT4_DIAGNOSTICS` 동작 동일.
   - 알려진 한계: `oak_d_align_cpp::OakdSender` 가 `image_transport::create_publisher(this, "/oakd/...")` 로 절대 토픽 경로를 사용하므로 `PushRosNamespace` 가 토픽에 prefix 를 붙이지 못함 → `/oakd/rgb/image_raw/aligned/...` 가 root 에 발행됨 (벤더 oakd 는 `/robot9/oakd/...`). C++ 코드 변경은 이번 스코프 OUT — 후속 작업에서 상대경로 + `robot_namespace` 파라미터 정리 필요.

3. **systemd — drop-in `/etc/systemd/system/turtlebot4.service.d/override.conf`**
   - 벤더 unit 미수정. `ExecStart=` 를 빈 줄로 reset 후 새 ExecStart 주입.
   - 새 ExecStart 가 직접 처리:
     1. `source /etc/turtlebot4/setup.bash` (RMW, FastDDS profile, `ROBOT_NAMESPACE=/robot9` 등)
     2. `source /home/ubuntu/turtlebot_align_ws/install/setup.bash` (커스텀 워크스페이스 overlay)
     3. `HOME=/home/ubuntu`, `ROS_HOME=/home/ubuntu/.ros`, `ROS_LOG_DIR=/tmp` 명시 (setpriv 가 HOME reset 안 해서 rcutils 가 `~` 확장 실패하던 회귀 방지)
     4. `setpriv --reuid ubuntu ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py`
   - 벤더의 `turtlebot4-start` → `mklaunch` → `/etc/ros/humble/turtlebot4.d/` 경로는 우회 (해당 경로의 심볼릭링크가 벤더 `standard.launch.py` 를 가리키므로 그대로 두면 `oakd.launch.py` 가 다시 들어옴).

4. **YAML 입력 — `src/oak_d_align_cpp/config/oakd_params.yaml`**
   - 브리프의 `docs/oakd_pro_turtlebot4.yaml` 사용 지시는 `depthai_ros_driver` 시절의 잔재. 그 YAML 의 키(`/oakd: ros__parameters: camera: i_pipeline_type:`) 는 우리 노드(`oakd_sender: ros__parameters: fps:`) 와 스키마 불일치 — 로드해도 declared 파라미터가 모두 default 로 남고 에러도 안 나는 silent failure 가 발생.
   - 결정: 이미 존재하는 `src/oak_d_align_cpp/config/oakd_params.yaml` 을 default 로 사용. launch 인자 `oakd_params_file` 로 override 가능.
   - `docs/oakd_pro_turtlebot4.yaml` 은 `depthai_ros_driver` 를 다시 쓰게 될 경우의 참조용으로 보존 (현재는 미사용).

**Verification (수행 결과, 2026-04-25)**:

| 항목 | 결과 |
|------|------|
| PC `colcon build --packages-select oak_d_align_cpp` (amd64) | error 0 |
| Bot `colcon build --packages-select oak_d_align_cpp` (arm64) | error 0 (deprecation warning만) |
| `systemctl restart turtlebot4.service` | active, 새 PID 3711 |
| `journalctl` | `Loaded: rclcpp_components::NodeFactoryTemplate<oak_d_align_cpp::OakdSender>` + `===== OAK-D C++ 노드 가동 완료 =====` 확인 |
| 벤더 `depthai_ros_driver` 프로세스 부재 | `ps -ef \| grep depthai_ros_driver` → 0건 |
| `ros2 topic hz /oakd/rgb/image_raw/aligned` | ~24 Hz |
| `ros2 topic hz /oakd/stereo/image_raw/aligned` | ~29 Hz |
| `ros2 topic hz /oakd/rgb/image_raw/aligned/compressed` | ~25 Hz |
| `ros2 topic hz /oakd/stereo/image_raw/aligned/compressedDepth` | ~8 Hz (PNG 인코딩 RPi4 부하 — 알려진 한계) |
| `tools/overlay.py --compressed --aligned --subpath oakd` 시각 검증 | user 측 GUI 실행 단계 |

**Consequences**:
- TurtleBot4 부팅 시 `oak_d_align_cpp` 가 자동 기동, 벤더 `depthai_ros_driver` 는 띄우지 않음.
- `compressedDepth` rate (~8 Hz) 가 raw rate (~29 Hz) 대비 낮음 — PC 측 `tools/overlay.py --compressed` 사용 시 Depth latency 가 더 크게 보일 수 있음. 향후 후처리 압축 파라미터 튜닝 또는 PNG 대신 Zstd-RVL 등 대안 인코딩 검토.
- Exit Criteria 의 PC dry-run runtime 단계 (≥10Hz 토픽 발행) 는 OAK-D 한 대 제약상 PC 에서 직접 못 함. Bot 측 동일 코드/파이프라인 검증으로 갈음.
- 토픽 namespace 누락 (`/oakd/...` vs 기대 `/robot9/oakd/...`) 후속 패치 필요. `tools/overlay.py` 는 `--namespace` 없이 `--subpath oakd` 만으로 동작하므로 즉시 검증은 가능.

### Update (2026-04-25): Phase 3-C 후속 보정

원 결정의 4개 항목 중 (1) Composable 토폴로지 와 (4) YAML 입력 운용 측면이 후속 검증에서
틀렸음이 드러나 보정. 트리거 3가지가 같은 빌드 사이클에 묶여 진행됨:
(i) RPi4 USB/CPU 부하 절감 요구 (compressedDepth ~8Hz 한계),
(ii) `docs/ros2_principals.md` Rule 5 (절대 토픽 경로 하드코딩) 위반 발견,
(iii) `ComposableNodeContainer` ↔ `launch_ros LoadComposableNodes` handshake 가 두 번
연속 실패 (`Load Library` 미호출 — 동일 launch 가 첫 부팅엔 정상 동작했으나 재현 불가).

**1. Composable 토폴로지: (b) → (c) standalone Node action**
- `launch_ros` 가 적재 service 호출을 silent 하게 못 보내는 race. `ros2 run oak_d_align_cpp oakd_sender`
  단독 실행은 정상 (RGB output 정상, 토픽 발행 정상) → 코드 무관, 적재 메커니즘 자체 문제.
- 노드 1개 단계에선 zero-copy 효과 0 이므로 (c) 로 fallback. 미래 후처리 노드 추가 시점에
  (b) 재시도 시 `LoadComposableNodes` 액션을 `ComposableNodeContainer` 와 분리하고
  `RegisterEventHandler(OnProcessIO)` 로 timing 명시 권장.
- 변경: `launch/turtlebot4_bringup.launch.py` 의 `ComposableNodeContainer` → `Node` action.

**2. Namespace 정석화 (Rule 5 준수)**
- `OakdSender` 가 `image_transport::create_publisher(this, "/oakd/...")` 로 절대 경로 + 자체
  `robot_namespace` 파라미터 prefix 조립을 사용해 `PushRosNamespace` 우회. 원 ADR 의 "토픽 namespace
  누락" 한계의 root cause 였음.
- 변경:
  - 코드 토픽 경로 `"/oakd/..."` → `"oakd/..."` (선두 슬래시 제거).
  - `robot_namespace` 파라미터 + 자체 조립 로직 모두 폐기.
  - 토픽 namespace 는 launch `PushRosNamespace(ROBOT_NAMESPACE)` → 노드 namespace → 자동 prefix.
- 결과: `/robot9/oakd/rgb/image_raw/aligned` (벤더 OAK-D 패턴과 일치).

**3. YAML 노드명 매칭 함정 해결**
- 원 YAML 키 `oakd_sender:` 는 root namespace 의 `/oakd_sender` 만 매칭. 노드 namespace 가
  `/robot9` 로 push 되면 `/robot9/oakd_sender` 가 되어 매칭 실패 → 전 파라미터가 declare default 로
  떨어짐 (silent failure). standalone (root ns) 검증에선 매칭됐기에 부분 검증으로는 못 잡음.
- 변경: `oakd_sender:` → `/**/oakd_sender:` 와일드카드.
- 부수 효과: 원 ADR 의 검증 표 (fps 30.0, HIGH_DENSITY 등) 는 default 와 동일해서 차이를 못
  드러냈을 뿐, 사실 yaml 매칭 자체가 안 되고 있었음. `rgb_isp_num/den` 신규 파라미터 추가로
  처음 표면화.

**4. RGB 해상도 파라미터 노출 (BRIEF Scope OUT 의 user override)**
- 동기: RPi4 USB/CPU 부하 절감 (compressedDepth ~8Hz 한계).
- 신규 파라미터: `rgb_isp_num` (default 2), `rgb_isp_den` (default 3) — `setIspScale(num, den)`.
- 검증: `num<=0 || den<=0 || num>den` 시 throw (Rule 7 명시 실패 — fail-fast).
- 적용: yaml 에서 `1/3` → 1920×1080 × 1/3 = **640×360**.

**Verification (수행 결과, 2026-04-25 — 원 표 갱신값)**

| 항목 | 결과 |
|------|------|
| `colcon build` (PC amd64 / Bot arm64) | error 0 |
| `systemctl restart turtlebot4.service` | active, 새 PID 7682 (총 3회 restart 후 안정 — 1차 namespace 적용 확인 / 2차 yaml 매칭 확인 / 3차 standalone fallback 확정) |
| journalctl 노드 가동 로그 | `node namespace : /robot9` / `RGB output : 640x360 (IspScale 1/3)` / `RGB topic : /robot9/oakd/rgb/image_raw/aligned` |
| 벤더 `depthai_ros_driver` 부재 | `ps -ef \| grep depthai_ros_driver` → 0건 |
| `ros2 topic hz /robot9/oakd/rgb/image_raw/aligned` (raw) | ~18.9 Hz (이전 1280×720 시 ~2.5 Hz, 7.5× 개선) |
| `ros2 topic hz /robot9/oakd/stereo/image_raw/aligned` (raw) | ~14.4 Hz |
| `ros2 topic hz /robot9/oakd/rgb/image_raw/aligned/compressed` | ~28.6 Hz (노드 발행률 정상) |
| **`ros2 topic hz /robot9/oakd/stereo/image_raw/aligned/compressedDepth`** | **~20.2 Hz (이전 ~8 Hz, 5× 개선 — 핵심 동기 충족)** |
| `tools/overlay.py --namespace robot9 --subpath oakd --aligned --compressed` 시각 검증 | 통과 (`docs/overlay.png` — RGB-Depth 윤곽 정렬, 깨짐/오프셋 없음) |

**Updated Consequences**:
- 원 한계 (토픽 namespace 누락) 해소.
- composable 적재 race 비결정성은 root cause 미규명 — `launch_ros` 의 `LoadComposableNodes`
  자동 트리거 (composable_node_descriptions 인자 syntactic sugar) 가 어떤 조건에서 silent 실패.
  미래 zero-copy 필요 시 재시도 권장 (옵션 (b) → 단 명시적 액션 분리).
- depth 품질 별도 한계 (이번 시각 검증에서 표면화):
  - `stereo_preset: HIGH_DENSITY` (중거리 최적) — 근거리 검증 장면엔 `HIGH_ACCURACY` 가 더 적합.
  - IR projector/flood off — 텍스처 없는 면 (모니터, 평면 벽) 매칭 실패 노이즈 주 원인.
    Y-adapter 확보 후 활성 권장 (RPi4 USB 전력 한계).
  - depthai 후처리 필터 (median, confidence threshold, LR check threshold) 미노출.
  → 별도 phase 권장 ("depth precision tuning").
- BRIEF Scope OUT (`C++ 코드 수정`) 위반 — user 명시 override 로 진행. 본 update 가 그 근거.

---
