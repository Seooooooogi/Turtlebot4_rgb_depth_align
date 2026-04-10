# turtlebot_align_ws v1.0

## Hard Rules (never bend)

Global rules → see [~/.claude/rules/ai-constitution.md](~/.claude/rules/ai-constitution.md)

Project-specific additions:
1. **vendor immutability**: `turtlebot4_bringup` 및 OAK-D 벤더 패키지 직접 수정 금지 — 커스텀 패키지는 `src/` 하위에만
2. **dry-run first**: 실제 TurtleBot4에 배포 전 반드시 PC에서 launch dry-run 검증
3. **no RPi CPU load**: 라즈베리파이4 CPU 부하 연산 금지 — 이미지 처리는 OAK-D VPU(하드웨어) 우선
4. **no fabrication on sensor data**: Depth/RGB 프레임 누락 시 null 반환 — 보간값 생성 금지

## Quick Ref

```bash
# 의존성 설치 (depthai 버전 고정)
pip install -r src/oak_d_align/requirements.txt

# OAK-D udev 룰 (최초 1회)
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 빌드 (zsh 환경)
cd ~/turtlebot_align_ws && colcon build --symlink-install
source install/setup.zsh

# 테스트
pytest src/<pkg>/test/ -q

# Launch dry-run (PC 검증)
ros2 launch <pkg> <launch_file>.launch.py --ros-args --log-level debug

# 토픽 확인
ros2 topic list | grep -E "rgb|depth|camera"
ros2 topic echo /camera/rgb/image_raw --no-arr
```

## Secrets Policy
- 로봇 IP, SSH 키, API 키 → 환경변수 또는 `.env` (커밋 금지)
- `.env.example`이 템플릿 — 실제 값 없음
- 새 자격증명 → `.env.example`에 플레이스홀더 추가

## Dev Conventions
- 테스트 없이 done 선언 금지
- 새 기능: 환경변수 feature flag로 opt-in (default OFF)
- 커스텀 코드는 `src/` 하위 패키지에만 — 벤더 패키지 직접 수정 불가
- 파라미터: YAML 파일로만 — Python/Launch 파일에 하드코딩 금지
- 커밋: 논리적 단위 1개, 명시적 요청 시에만

## Focus Areas (2026-04 기준)
1. **Depth 정밀도**: 4m 거리에서 1m 이상 오차 → 후처리 필터(Spatial/Temporal/Speckle) 파라미터 튜닝
2. **TurtleBot4 통합**: 커스텀 OAK-D 정렬 노드 → `turtlebot4_bringup` launch 시스템에 충돌 없이 통합

## Compact Instructions
Preserve on compaction:
1. Hard Rules (global + project-specific)
2. Current active branch / uncommitted file list
3. Pending tasks and their status
4. Active errors or bugs being investigated
5. Dev Conventions
6. File paths modified in this session
