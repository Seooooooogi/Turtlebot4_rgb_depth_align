# Brief: Phase 3-C — Integrate oak_d_align_cpp into TurtleBot4 bringup

**Branch**: `feature/oak-d-align-cpp`
**Created**: 2026-04-25

## Goal

TurtleBot4의 `standard.launch.py` 구성 요소(robot/joy/rplidar/description)는 그대로 기동하면서 기본 `oakd.launch.py` 대신 `oak_d_align_cpp`의 정렬 노드를 띄우는 커스텀 bringup launch를 작성하고, `turtlebot4.service` systemd 자동 기동까지 통합한 뒤 main으로 머지한다.

## Scope IN

- `src/oak_d_align_cpp/launch/turtlebot4_bringup.launch.py` 신규 작성 — `standard.launch.py`의 robot/joy/rplidar/description 그대로 포함, 벤더 `oakd.launch.py` 호출 제외
- depthai_ros_driver가 띄우는 기존 container에 oak_d_align_cpp 노드를 intra-process(zero-copy)로 합류 시도(목표). 검증 실패 시 차선 (b) 자체 container 생성 또는 (c) standalone node 로 fallback — 어느 경로로 갔는지 ADR-004로 기록
- 기존 `docs/oakd_pro_turtlebot4.yaml`을 launch 파라미터 입력으로 그대로 사용
- 정렬된 RGB/Depth 토픽 경로 명시 (네임스페이스/토픽명 변경 필요 시 launch 인자로)
- TurtleBot4 arm64 재빌드: 워크스페이스 rsync → `colcon build --packages-select oak_d_align_cpp`
- `turtlebot4.service` systemd 자동 기동 통합 — 가능하면 systemd drop-in (`/etc/systemd/system/turtlebot4.service.d/*.conf`) 또는 unit 파일의 ExecStart override (벤더 파일 직접 편집 금지)
- PC dry-run: launch 정상 기동 + 정렬 토픽 발행 확인
- TurtleBot4 SSH 실기 검증: `journalctl -u turtlebot4.service` 에서 새 pid + oak_d_align_cpp 기동 로그 확인
- PC 측에서 `tools/overlay.py --compressed --aligned --subpath oakd` 시각 검증
- 검증 통과 후 `feature/oak-d-align-cpp` → `main` 머지

## Scope OUT (잠금)

- 새 YAML 작성/조정 — 기존 `docs/oakd_pro_turtlebot4.yaml` 외 별도 파일 만들지 않음
- RViz config / 시각화 launch 자산 추가
- PC 기준선(28fps, 정렬 완벽) 대비 fps·정렬 품질의 정량 비교 — 이번 done 기준은 "시각적으로 깨지지 않음" 까지
- `oak_d_align_cpp` C++ 코드 수정 (PC에서 이미 검증, 이번 작업은 통합만)
- Python `oak_d_align` 패키지 변경

## Constraints

- **벤더 immutability**: `turtlebot4_bringup` 패키지(launch/config/unit 파일) 직접 수정 금지. 모든 신규 자산은 `src/oak_d_align_cpp/` 하위 또는 systemd drop-in 디렉터리에만
- **하드웨어 안전 (Tier 0 회귀 금지)**: YAML 미변경 원칙 유지 — `i_usb_speed: SUPER_PLUS`, `i_enable_ir: true` 어떤 경우에도 활성화 금지 (RPi4 부팅 루프/USB 과전류 위험)
- **WiFi 대역폭 제약**: 실기 PC 시각 검증은 raw 토픽 불가 — `compressedDepth` + `--compressed` 경로 사용
- **YAML 노드명 매칭**: TurtleBot4 노드명은 `oakd` (PC의 `oak`와 다름) — wildcard 또는 `/oakd:` 키 사용
- **머지 게이트**: 모든 Exit Criteria 통과 후에만 main 머지

## Exit Criteria

- [ ] `colcon build --packages-select oak_d_align_cpp`가 PC(amd64)와 TurtleBot4(arm64)에서 모두 에러 0으로 완료
- [ ] PC에서 `ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py` 실행 시 에러 없이 기동, 정렬된 RGB/Depth 토픽이 `ros2 topic list` 에 등장하고 `ros2 topic hz` 로 ≥10Hz 발행 확인
- [ ] TurtleBot4에서 `sudo systemctl restart turtlebot4.service` 후 `journalctl -u turtlebot4.service --since "1 minute ago"` 출력에 (a) 새 PID, (b) oak_d_align_cpp 기동 로그, (c) 벤더 `oakd.launch.py`/기존 OAK-D 노드 부재가 모두 확인됨
- [ ] PC에서 `tools/overlay.py --compressed --aligned --subpath oakd` 실행 시 RGB/Depth 정렬 overlay 가 검은 화면/깨짐/오프셋 없이 표시됨
- [ ] Composable 적재 경로((a)/(b)/(c) 중 무엇으로 갔는지)가 `docs/decisions/README.md` ADR-004 로 기록됨
- [ ] `feature/oak-d-align-cpp` → `main` 머지 완료 (방식은 user 결정)

## Risk Flags

- depthai_ros_driver v2.12.2 가 외부 컴포저블 노드의 동일 container 합류를 허용하는지 미검증 — 실패 시 차선 (b)/(c)로 전환하되, 그 결정 자체를 ADR-004에 기록
- `turtlebot4.service` 의 ExecStart override 방식이 벤더 immutability(unit 파일 수정 금지)와 충돌하지 않도록 systemd drop-in 사용 — 직접 편집 시 OS 업데이트로 롤백 가능
- 과거 systemd restart 후 이전 PID 잔존 현상 — 매 검증마다 journalctl로 새 PID 확인 필수
- `--compressed` 경로의 정렬 품질이 raw 경로와 동등한지 시각 외 정량 검증은 OUT 처리 — 이번 머지 후 별도 확인 필요할 수 있음
- main 머지 후 회귀: PC 단독 사용 시 기존 `oak_d_align_cpp` standalone 동작이 깨지지 않는지 quick smoke test 권장 (Exit Criteria엔 미포함, 자체 점검)
