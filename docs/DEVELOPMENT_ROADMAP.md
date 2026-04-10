# turtlebot_align_ws — Development Roadmap

마감: 2026-05-01 | 담당: 지능1팀

---

## Phase 1: ROS2 패키지 구조 수립 (목표: colcon build 통과) ✅
- [x] 1-1. 커스텀 패키지 스캐폴딩 (`src/oak_d_align/`) — `package.xml`, `setup.py`, `setup.cfg`
- [x] 1-2. Python 스크립트 → rclpy 노드로 래핑 (`OakdSender`)
- [x] 1-3. YAML 파라미터 파일 작성 (`config/oakd_params.yaml` — fps, subpixel, alpha, roi_size 등)
- [ ] 1-4. Launch 파일 작성 + PC dry-run 검증 ← launch 파일 생성 완료, **실제 dry-run 미실시**

## Phase 2: Depth 정밀도 개선 (목표: 4m 오차 1m 이하)
- [ ] 2-1. OAK-D 카메라 스펙 확인 (최대 유효 Depth 범위)
- [ ] 2-2. Spatial / Temporal / Speckle Filter 파라미터 튜닝
- [ ] 2-3. Edge bleeding 억제 (필터 조합 최적화)
- [ ] 2-4. 정밀도 측정 지그 구성 및 수치 기록

## Phase 3: TurtleBot4 통합 (목표: bringup 시 자동 실행)
- [ ] 3-1. `turtlebot4_bringup` launch 구조 분석 (TF, base_link 간 거리)
- [ ] 3-2. 기존 OAK-D 노드와 충돌 여부 확인 (토픽/노드 네임스페이스)
- [ ] 3-3. 커스텀 launch 파일에서 기존 OAK-D 노드 disable + 대체 노드 실행
- [ ] 3-4. TurtleBot4 실기 테스트 (11fps 이상 유지 확인)

## Phase 4: 마무리
- [ ] 4-1. `dai.VideoEncoder` 활용 RPi4 CPU 부담 완화 (백로그 → 필요시)
- [ ] 4-2. 테스트 스위트 작성
- [ ] 4-3. README 및 파라미터 문서화

## Backlog (미정)
- [ ] `dai.VideoEncoder`를 통한 압축 스트리밍
- [ ] TF 구조 완전 파악 및 base_link 거리 보정
- [ ] 멀티 카메라 지원 고려
