# Robot Nav2 Package

Navigation2 자율 주행 패키지

## 📁 폴더 구조

```
robot_nav2/
├── config/
│   └── nav2_params.yaml          # ★ Navigation2 파라미터
│
├── launch/
│   ├── navigation.launch.py      # 네비게이션 런치 (맵 기반)
│   └── slam_navigation.launch.py # SLAM + 네비게이션 동시 실행
│
├── CMakeLists.txt
└── package.xml
```

---

## 🗺️ Navigation Stack 구성

### TF 프레임 구조

```
map → odom → base_link → sensors
      ↑
   (EKF 제공)
```

### 토픽 연결

| 컴포넌트 | 입력 토픽 | 출력 토픽 |
|----------|-----------|-----------|
| AMCL | `/scan`, `/map` | TF (map→odom) |
| Global Planner | `/map`, `/goal_pose` | `/plan` |
| Local Planner | `/scan`, `/plan` | `/cmd_vel` |
| BT Navigator | `/goal_pose` | 액션 조정 |

---

## ⚙️ 주요 파라미터 (nav2_params.yaml)

### AMCL (위치 추정)

```yaml
amcl:
  base_frame_id: "base_link"
  odom_frame_id: "odom"
  global_frame_id: "map"
  scan_topic: scan
  robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

### BT Navigator (행동 트리)

```yaml
bt_navigator:
  odom_topic: /odometry/filtered  # EKF 오도메트리 사용
  robot_base_frame: base_link
```

### Controller (로컬 플래너)

```yaml
controller_server:
  controller_plugins: ["FollowPath"]
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
```

### Planner (글로벌 플래너)

```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_navfn_planner::NavfnPlanner"
```

---

## 🚀 사용법

### 저장된 맵으로 네비게이션

```bash
# 1. 맵 서버 + AMCL + Navigation 시작
ros2 launch robot_nav2 navigation.launch.py map:=/path/to/map.yaml

# 2. RViz에서 초기 위치 설정 (2D Pose Estimate)
# 3. 목표 지점 설정 (2D Goal Pose)
```

### SLAM + 네비게이션 동시 실행

```bash
# SLAM으로 맵 생성하면서 네비게이션
ros2 launch robot_nav2 slam_navigation.launch.py
```

---

## 📐 로봇 풋프린트

```yaml
# 4WD 로봇의 풋프린트 (사각형)
footprint: "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"
```

---

## 🛠️ 코스트맵 설정

### Global Costmap

```yaml
global_costmap:
  robot_base_frame: base_link
  global_frame: map
  resolution: 0.05
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Local Costmap

```yaml
local_costmap:
  global_frame: odom
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```

---

## 🎯 네비게이션 액션

### Python으로 목표 전송

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.y = 2.0

navigator.goToPose(goal_pose)
```

### CLI로 목표 전송

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}}"
```

---

## 🔧 트러블슈팅

### "Transform timeout" 에러
- TF가 정상 발행되는지 확인
- `ros2 run tf2_tools view_frames` 로 TF 트리 확인

### 로봇이 장애물을 피하지 못함
- inflation_radius 증가
- cost_scaling_factor 조정

### 경로가 생성되지 않음
- 맵이 정상 로드됐는지 확인
- 목표 지점이 장애물 위가 아닌지 확인
