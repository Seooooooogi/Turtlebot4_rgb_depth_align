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
