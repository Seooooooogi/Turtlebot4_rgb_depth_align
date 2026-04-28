"""TurtleBot4 bringup shim — vendor oakd.launch.py 자리에 oak_d_align_cpp 끼워넣기.

================================================================================
역할 (docs/turtlebot4_bringup_integration.md Layer 2)
================================================================================
Vendor `turtlebot4_bringup/launch/standard.launch.py` 와 동일한 흐름을 재현하되,
`oakd.launch.py` 만 의도적으로 누락하고 그 자리에 우리의 `oakd_sender` 노드를
끼워넣는다. 나머지 robot/joy/rplidar/description/diagnostics 는 vendor launch
파일을 IncludeLaunchDescription 으로 그대로 호출 — vendor 자산 미수정 (Hard Rule #1).

================================================================================
실행 경로
================================================================================
(a) PC standalone 검증:
    ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py
(b) TurtleBot4 운영: systemd/override.conf 로 자동 실행
    ExecStart=... ros2 launch oak_d_align_cpp turtlebot4_bringup.launch.py

================================================================================
namespace 흐름 (토픽 prefix 결정 메커니즘)
================================================================================
$ROBOT_NAMESPACE (env, e.g. "/robot9") → PushRosNamespace → GroupAction 내부의
모든 노드의 effective namespace 가 됨 → oakd_sender 코드는 상대 경로만 선언
(예: "oakd/rgb/image_raw/aligned") → 최종 토픽: "/robot9/oakd/rgb/image_raw/aligned"

================================================================================
ComposableNodeContainer 가 아니라 standalone Node 인 이유 (ADR-004 Update)
================================================================================
ComposableNodeContainer + LoadComposableNodes 핸드셰이크가 비결정 — 두 번째
재시작에서 component library 가 silent 로 로드되지 않는 케이스 발견. 컨테이너
안에 oakd_sender 단독 거주이므로 zero-copy intra-process 이득은 어차피 0.
운영 launch 는 standalone Node action 으로 fallback. 개발용 composable 검증은
`oakd_sender.launch.py` 에 보존.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lc = LaunchContext()

    # ── Vendor 동작 보존을 위한 환경변수 두 개 ────────────────────────────────
    # TURTLEBOT4_DIAGNOSTICS: vendor standard.launch.py 와 동일하게 '1' 일 때만
    #   diagnostics include. /etc/turtlebot4/setup.bash 에서 export 됨.
    # ROBOT_NAMESPACE: 모든 토픽의 prefix. 예) "/robot9" → 토픽들이 /robot9/...
    diagnostics_enable = EnvironmentVariable('TURTLEBOT4_DIAGNOSTICS', default_value='1')
    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')

    # ── 패키지 share 디렉터리 해소 ────────────────────────────────────────────
    # vendor 패키지 3 종은 IncludeLaunchDescription 으로 호출 — 직접 수정 없음.
    # oak_d_align_cpp 만 우리 쪽 패키지로 default param 파일 경로 결정에 사용.
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_oak_d_align_cpp = get_package_share_directory('oak_d_align_cpp')

    default_params_file = os.path.join(pkg_oak_d_align_cpp, 'config', 'oakd_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    # param_file:        vendor turtlebot4.yaml — robot.launch.py 가 사용 (create3 등)
    # oakd_params_file:  우리 oakd_sender 가 사용 — 운영 시 override 가능
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='TurtleBot4 robot param file (vendor)'
    )
    oakd_params_file_cmd = DeclareLaunchArgument(
        'oakd_params_file',
        default_value=default_params_file,
        description='oak_d_align_cpp parameter YAML'
    )

    param_file = LaunchConfiguration('param_file')
    oakd_params_file = LaunchConfiguration('oakd_params_file')

    # ── vendor turtlebot4.yaml 의 namespace rewrite ──────────────────────────
    # vendor robot.launch.py 가 받는 turtlebot4.yaml 은 root key 가 고정 namespace.
    # ROBOT_NAMESPACE 가 변경되어도 YAML 의 root key 는 그대로이므로 nav2_common 의
    # RewrittenYaml 로 root_key 만 동적 치환. 우리 oakd_params.yaml 은 root key 가
    # `/**/oakd_sender:` 와일드카드라서 별도 rewrite 불필요.
    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    joy_teleop_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'joy_teleop.launch.py'])
    diagnostics_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_diagnostics, 'launch', 'diagnostics.launch.py'])
    rplidar_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'rplidar.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])

    # ── 우리 노드: standalone executable (composable 미사용) ──────────────────
    # 이유: ComposableNodeContainer + LoadComposableNodes 핸드셰이크가 비결정 —
    # 두 번째 재시작에서 component library 가 silent 로 로드되지 않는 케이스 발견.
    # 컨테이너 안에 oakd_sender 단독 거주이므로 zero-copy intra-process 이득 0.
    # 따라서 standalone Node 로 fallback. ADR-004 updated.
    # parameters=[oakd_params_file] — YAML 한 파일만 전달 (override 시 -p 키 추가).
    # output='screen' — systemd 환경에서 stdout 가 journalctl 에 캡처됨.
    oakd_node = Node(
        package='oak_d_align_cpp',
        executable='oakd_sender',
        name='oakd_sender',
        parameters=[oakd_params_file],
        output='screen',
    )

    # ── actions 묶음 ─────────────────────────────────────────────────────────
    # vendor standard.launch.py 와 동일 순서로 robot/joy/rplidar/description/
    # diagnostics 를 호출. 단, vendor oakd.launch.py 자리에 oakd_node 가 들어감.
    # PushRosNamespace 가 GroupAction 의 첫 action 이라야 후속 모든 노드에 적용됨.
    actions = [
        # PushRosNamespace: GroupAction 내부 모든 노드의 effective namespace 설정.
        # vendor 와 동일한 동작 — turtlebot4 의 모든 토픽이 /<ROBOT_NAMESPACE>/...
        PushRosNamespace(namespace),

        # robot.launch.py: create3 driver + topic bridge. vendor turtlebot4.yaml
        # 을 namespace rewrite 후 전달.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
            launch_arguments=[('model', 'standard'),
                              ('param_file', namespaced_param_file)]),

        # joy_teleop.launch.py: 조이스틱 teleop. namespace 명시 전달.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joy_teleop_launch_file]),
            launch_arguments=[('namespace', namespace)]),

        # rplidar.launch.py: 2D lidar.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch_file])),

        # ★ vendor oakd.launch.py 자리 — 의도적 누락. 우리 노드로 대체.
        # 우리 노드는 setDepthAlign(RGB_SOCKET) 으로 정렬한 토픽을 발행:
        #   /<ns>/oakd/rgb/image_raw/aligned
        #   /<ns>/oakd/stereo/image_raw/aligned
        #   (+ image_transport 가 /compressed, /compressedDepth 자동 발행)
        oakd_node,

        # robot_description.launch.py: URDF + robot_state_publisher.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_launch_file]),
            launch_arguments=[('model', 'standard')]),
    ]

    # ── diagnostics 조건부 추가 ───────────────────────────────────────────────
    # vendor standard.launch.py 와 동일하게 TURTLEBOT4_DIAGNOSTICS=1 일 때만 include.
    # LaunchContext.perform 으로 즉시 evaluate — IfCondition 대신 사용한 이유는
    # 단순한 환경변수 체크라 lazy substitution 이 필요 없기 때문.
    if diagnostics_enable.perform(lc) == '1':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([diagnostics_launch_file]),
            launch_arguments=[('namespace', namespace)]))

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)
    ld.add_action(oakd_params_file_cmd)
    ld.add_action(GroupAction(actions))
    return ld
