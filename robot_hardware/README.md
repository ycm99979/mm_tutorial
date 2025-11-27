# Robot Hardware Package

MD 모터 드라이버를 위한 ros2_control Hardware Interface 패키지

## 📁 폴더 구조

```
robot_hardware/
├── include/md_hardware/          # 헤더 파일
│   ├── md_hardware.hpp           # 2WD Hardware Interface
│   └── md_4wd_hardware.hpp       # 4WD Hardware Interface
│
├── src/                          # 소스 파일
│   ├── md_hardware.cpp           # 2WD 구현체
│   ├── md_4wd_hardware.cpp       # 4WD 구현체
│   ├── com.cpp                   # (참고용) MD 통신 코드
│   └── md_controller.cpp         # (참고용) MD 컨트롤러
│
├── config/                       # 설정 파일 (YAML)
│   ├── md_4wd_hardware.yaml      # ★ 4WD 하드웨어 파라미터
│   ├── md_4wd_controllers.yaml   # ★ 4WD 컨트롤러 설정
│   └── md_controllers.yaml       # 2WD 컨트롤러 설정
│
├── urdf/                         # 로봇 모델 정의
│   ├── md_4wd_robot.urdf.xacro   # ★ 4WD 로봇 URDF
│   └── md_robot.urdf.xacro       # 2WD 로봇 URDF
│
├── launch/                       # 런치 파일
│   ├── md_4wd_test.launch.py     # ★ 4WD 테스트 런치
│   └── md_hardware_test.launch.py # 2WD 테스트 런치
│
├── CMakeLists.txt                # 빌드 설정
├── package.xml                   # 패키지 의존성
└── md_hardware_plugin.xml        # pluginlib 등록
```

---

## 🔧 주요 파일 설명

### Hardware Interface (C++)

| 파일 | 설명 |
|------|------|
| `md_hardware.hpp/cpp` | 2륜 차동 구동 (1개 MD 드라이버) |
| `md_4wd_hardware.hpp/cpp` | **4륜 개별 제어 (2개 MD 드라이버)** |

### 설정 파일 (YAML)

| 파일 | 용도 | 주요 파라미터 |
|------|------|--------------|
| `md_4wd_hardware.yaml` | 하드웨어 설정 | 시리얼 포트, 드라이버 ID, 휠 크기, 기어비 |
| `md_4wd_controllers.yaml` | 컨트롤러 설정 | 속도 제한, 오도메트리, TF 프레임 |

### URDF

| 파일 | 설명 |
|------|------|
| `md_4wd_robot.urdf.xacro` | 4륜 로봇 모델 + ros2_control 설정 |

---

## 🚀 사용법

### 4WD 실행

```bash
# 기본 실행
ros2 launch robot_hardware md_4wd_test.launch.py

# 포트 변경
ros2 launch robot_hardware md_4wd_test.launch.py port:=/dev/ttyUSB1

# 파라미터 오버라이드
ros2 launch robot_hardware md_4wd_test.launch.py wheel_radius:=0.06 wheel_separation:=0.35
```

### 속도 명령 전송

```bash
# 전진 (use_stamped_vel: false 설정 시)
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 회전
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### 상태 확인

```bash
# 조인트 상태
ros2 topic echo /joint_states

# 오도메트리
ros2 topic echo /diff_drive_controller/odom

# 컨트롤러 상태
ros2 control list_controllers
```

---

## ⚙️ 파라미터 수정

### 자주 변경하는 파라미터

| 파라미터 | 파일 | 설명 |
|----------|------|------|
| `port` | `md_4wd_hardware.yaml` | 시리얼 포트 |
| `front_driver_id` | `md_4wd_hardware.yaml` | 전방 드라이버 ID |
| `rear_driver_id` | `md_4wd_hardware.yaml` | 후방 드라이버 ID |
| `wheel_radius` | 둘 다 | 휠 반지름 (m) |
| `wheel_separation` | 둘 다 | 좌우 휠 간격 (m) |
| `gear_ratio` | `md_4wd_hardware.yaml` | 기어비 |
| `poles` | `md_4wd_hardware.yaml` | 모터 극 수 |
| `max_velocity` | `md_4wd_controllers.yaml` | 최대 속도 제한 |

### ⚠️ 중요

`wheel_radius`와 `wheel_separation`은 **두 파일 모두** 동일하게 설정해야 합니다:
- `config/md_4wd_hardware.yaml` (엔코더 계산용)
- `config/md_4wd_controllers.yaml` (오도메트리 계산용)

---

## 🔌 하드웨어 연결

### 4WD 모터 드라이버 매핑

```
┌─────────────────────────────────────────────────────────────┐
│                    RS-485 Bus (Single Port)                 │
│  ┌─────────────────────────┐  ┌─────────────────────────┐  │
│  │   Front Driver (ID=1)   │  │   Rear Driver (ID=2)    │  │
│  │  CH1: Front Left (FL)   │  │  CH1: Rear Left (RL)    │  │
│  │  CH2: Front Right (FR)  │  │  CH2: Rear Right (RR)   │  │
│  └─────────────────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 시리얼 포트 권한

```bash
# dialout 그룹 추가 (최초 1회)
sudo usermod -aG dialout $USER
# 재로그인 필요

# 권한 확인
groups | grep dialout
```

---

## 🐛 문제 해결

### 시리얼 포트 에러

```
IOException: No such file or directory
```

**해결:**
1. USB-RS485 변환기 연결 확인: `ls /dev/ttyUSB*`
2. 포트 변경: `port:=/dev/ttyUSB0`
3. 권한 확인: `sudo chmod 666 /dev/ttyUSB0`

### 컨트롤러 로드 실패

```
Could not find controller 'diff_drive_controller'
```

**해결:**
```bash
sudo apt install ros-humble-diff-drive-controller
```

### 휠이 반대로 회전

**해결:**
- 드라이버의 채널 배선 교체
- 또는 `gear_ratio`를 음수로 설정

---

## 📝 빌드

```bash
cd ~/frbot_ws
colcon build --packages-select robot_hardware --symlink-install
source install/setup.bash
```

---

## 📚 참고

- [ros2_control 문서](https://control.ros.org/)
- [diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- MD 모터 드라이버 매뉴얼
