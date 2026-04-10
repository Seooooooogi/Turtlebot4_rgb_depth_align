# turtlebot_align_ws — Development Roadmap

마감: 2026-05-01 | 담당: 지능1팀

---

## Phase 1: ROS2 패키지 구조 수립 ✅
- [x] 1-1. 커스텀 패키지 스캐폴딩 (`src/oak_d_align/`) — `package.xml`, `setup.py`, `setup.cfg`
- [x] 1-2. Python 스크립트 → rclpy 노드로 래핑 (`OakdSender`)
- [x] 1-3. YAML 파라미터 파일 작성 (`config/oakd_params.yaml` — fps, subpixel, alpha, roi_size 등)
- [x] 1-4. Launch 파일 작성 + 실행 검증 완료
- [x] 1-5. depthai 2.28.0 호환성 수정 (Sync/ImageAlign API 대응)
- [x] 1-6. 카메라 루프 스레드 분리 + blocking get()으로 프레임 동기화
- [x] 1-7. undistort 맵 사전 계산 (cv2.remap으로 CPU 부하 감소)
- [x] 1-8. 안전 종료 처리 (device.close, stop_event, thread.join)

## Phase 2: 4m 거리 Depth 정밀도 향상 (목표: 4m 오차 1m 이하)

### 2-1. 스펙 분석 ✅
- baseline 75mm, 4m에서 disparity ≈ 12px → 1px 오차 = 330mm (물리적 한계)
- IR dot projector 미활성화 확인 — 텍스처 없는 표면 오차의 주원인
- subpixel=False → 재활성화 시 오차 ~10mm 수준으로 개선 가능
- RPi4에서 IR 활성화 시 전력 900mA 초과 가능 → Y-adapter 필요할 수 있음

### 2-2. 하드웨어 레벨 개선 (Active Stereo + Subpixel) — 부분 완료
- [x] 2-2-1. IR dot projector 활성화 (YAML: `ir_dot_projector_intensity: 0.5`)
- [x] 2-2-2. IR flood LED 활성화 옵션 추가 (YAML: `ir_flood_intensity: 0.0`)
- [x] 2-2-3. `stereo_subpixel: true` 재활성화
- [ ] 2-2-4. IR intensity 최적값 튜닝 (줄자 측정 후 진행) ← **보류**

### 2-3. VPU 후처리 필터 튜닝 ← **보류** (TurtleBot4 통합 후 실측 환경에서 진행)
- [ ] 2-3-1. Spatial Filter 파라미터 YAML화 및 활성화
- [ ] 2-3-2. Temporal Filter 파라미터 YAML화 및 활성화
- [ ] 2-3-3. Speckle Filter 파라미터 YAML화 및 활성화
- [ ] 2-3-4. Edge bleeding 억제 — 필터 조합 최적화

### 2-4. 정밀도 검증 ← **보류** (줄자 및 실측 환경 필요)
- [ ] 2-4-1. 거리별 오차 수치 기록 (1m / 2m / 3m / 4m)
- [ ] 2-4-2. 필터 적용 전/후 비교

## Phase 3: TurtleBot4 통합 (목표: bringup 시 자동 실행) ← **현재 진행**
- [ ] 3-1. `turtlebot4_bringup` launch 구조 분석 (TF, base_link 간 거리)
- [ ] 3-2. 기존 OAK-D 노드와 충돌 여부 확인 (토픽/노드 네임스페이스)
- [ ] 3-3. 커스텀 launch 파일에서 기존 OAK-D 노드 disable + 대체 노드 실행
- [ ] 3-4. TurtleBot4 실기 테스트 (fps 및 동작 확인)

## Phase 4: 마무리
- [ ] 4-1. `dai.VideoEncoder` 활용 RPi4 CPU 부담 완화 (백로그 → 필요시)
- [ ] 4-2. 테스트 스위트 작성
- [ ] 4-3. README 및 파라미터 문서화

## Backlog (미정)
- [ ] `dai.VideoEncoder`를 통한 압축 스트리밍
- [ ] TF 구조 완전 파악 및 base_link 거리 보정
- [ ] 멀티 카메라 지원 고려
