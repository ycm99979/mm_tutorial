# FRBot Workspace - Source Packages

4륜 스키드 스티어 모바일 매니퓰레이터 로봇을 위한 ROS2 패키지 모음

## 📁 패키지 구조

```
src/
├── FAST_LIO_ROS2/        # 🔵 LiDAR-Inertial Odometry (FAST-LIO)
├── livox_ros_driver2/    # 🔵 Livox LiDAR 드라이버 (MID-360)
├── realsense-ros/        # 📷 Intel RealSense 카메라 드라이버 (D455)
├── robot_bringup/        # 🚀 통합 실행 (런치 + 설정)
├── robot_description/    # 📐 로봇 모델 (URDF/Xacro)
├── robot_gazebo/         # 🎮 Gazebo 시뮬레이션
├── robot_hardware/       # ⚙️ 하드웨어 인터페이스 (ros2_control)
├── robot_nav2/           # 🗺️ Navigation2 자율주행
├── robot_slam/           # 📍 SLAM (Cartographer)
└── serial/               # 📡 시리얼 통신 라이브러리
```

---

## 📦 패키지별 역할

### 🚀 robot_bringup
**실제 로봇 실행을 위한 통합 패키지**

| 기능 | 설명 |
|------|------|
| 센서 설정 | RealSense, Velodyne, IMU 설정 파일 |
| EKF | robot_localization 센서 융합 |
| 런치 파일 | 하드웨어 + EKF 통합 실행 |

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

---

### 📐 robot_description
**로봇 URDF/Xacro 모델 정의**

| 기능 | 설명 |
|------|------|
| URDF | 로봇 기구학, 링크, 조인트 정의 |
| Meshes | 3D 모델 파일 (STL/DAE) |
| 매니퓰레이터 | 4DOF 로봇팔 포함 |

```bash
ros2 launch robot_description mobile_manipulator.launch.py
```

---

### 🎮 robot_gazebo
**Gazebo 시뮬레이션 환경**

| 기능 | 설명 |
|------|------|
| Worlds | 시뮬레이션 월드 파일 (SDF) |
| Models | 테이블, 문, 가전제품 등 모델 |
| 런치 | Gazebo + ros2_control 통합 |

```bash
ros2 launch robot_gazebo frbot_gz_sim.launch.py
```

---

### ⚙️ robot_hardware
**실제 하드웨어 제어 (ros2_control)**

| 기능 | 설명 |
|------|------|
| MD Motor Driver | MD 듀얼채널 모터 드라이버 인터페이스 |
| 4WD 제어 | 4륜 개별 속도 제어 |
| RS-485 통신 | 시리얼 통신 프로토콜 |

```bash
ros2 launch robot_hardware md_4wd_test.launch.py
```

---

### 🗺️ robot_nav2
**Navigation2 자율주행**

| 기능 | 설명 |
|------|------|
| 경로 계획 | Global/Local Planner |
| 장애물 회피 | Costmap 설정 |
| 목표점 주행 | Action Server |

```bash
ros2 launch robot_nav2 navigation.launch.py
```

---

### 📍 robot_slam
**SLAM (Simultaneous Localization and Mapping)**

| 기능 | 설명 |
|------|------|
| Cartographer | Google Cartographer 2D SLAM |
| 지도 생성 | LiDAR 기반 맵핑 |

```bash
ros2 launch robot_slam cartographer.launch.py
```

---

### 📡 serial
**시리얼 통신 라이브러리**

| 기능 | 설명 |
|------|------|
| Cross-platform | Linux/Windows 지원 |
| robot_hardware 의존성 | MD Motor 통신에 사용 |

---

## 🔄 실행 순서

### 시뮬레이션

```bash
# 1. Gazebo 시뮬레이션
ros2 launch robot_gazebo frbot_gz_sim.launch.py

# 2. SLAM (새 터미널)
ros2 launch robot_slam cartographer.launch.py use_sim_time:=true

# 3. Navigation (새 터미널)
ros2 launch robot_nav2 navigation.launch.py use_sim_time:=true
```

### 실제 하드웨어

```bash
# 1. 로봇 하드웨어 + EKF
ros2 launch robot_bringup robot_bringup.launch.py

# 2. SLAM (새 터미널)
ros2 launch robot_slam cartographer.launch.py

# 3. Navigation (새 터미널)
ros2 launch robot_nav2 navigation.launch.py
```

---

## 📝 빌드

```bash
cd ~/frbot_ws
colcon build --symlink-install
source install/setup.bash
```

### ⚠️ livox_ros_driver2 빌드 시 주의사항

`livox_ros_driver2` 패키지는 ROS2 Humble에서 빌드 시 추가 CMake 플래그가 필요합니다:

```bash
# 에러 발생 시:
# CMake Error: LIVOX_INTERFACES_INCLUDE_DIRECTORIES - NOTFOUND

# 해결 방법: ROS_EDITION과 HUMBLE_ROS 플래그 추가
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble"
```

| 문제 | 원인 | 해결 |
|------|------|------|
| `LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND` | CMakeLists.txt에서 ROS2 버전 분기 조건 미충족 | `-DHUMBLE_ROS="humble"` 플래그 추가 |
| ROS1 코드로 빌드 시도 | `ROS_EDITION` 미설정 | `-DROS_EDITION="ROS2"` 플래그 추가 |

### 전체 빌드 명령어

```bash
# 1. livox_ros_driver2 먼저 (특수 플래그 필요)
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble"

# 2. 나머지 패키지
colcon build --packages-skip livox_ros_driver2 --symlink-install
```

---

## 🔗 의존성

- ROS2 Humble
- ros2_control
- Navigation2
- Cartographer
- Gazebo Fortress
- robot_localization
- Livox SDK2 (`/usr/local/lib/liblivox_lidar_sdk_shared.so`)
- PCL (Point Cloud Library)

---

## 📦 추가 패키지 설명

### 🔵 FAST_LIO_ROS2
**고속 LiDAR-Inertial Odometry**

| 기능 | 설명 |
|------|------|
| LiDAR + IMU 융합 | 고정밀 오도메트리 |
| 실시간 처리 | ikd-Tree 기반 빠른 매칭 |
| 출력 토픽 | `/Odometry` (nav_msgs/Odometry) |

```bash
ros2 launch fast_lio mapping.launch.py
```

---

### 🔵 livox_ros_driver2
**Livox LiDAR 드라이버 (MID-360, HAP 등)**

| 기능 | 설명 |
|------|------|
| MID-360 지원 | 비반복 스캔 패턴 LiDAR |
| 내장 IMU | MID-360 내장 IMU 데이터 출력 |
| 출력 토픽 | `/livox/lidar`, `/livox/imu` |

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

---


### 📷 realsense-ros
**Intel RealSense 카메라 드라이버 (D455)**

| 기능 | 설명 |
|------|------|
| RGB-D 카메라 | Color + Depth 스트림 |
| 내장 IMU | D455 내장 IMU 데이터 출력 |
| 출력 토픽 | `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu` |

| 서브패키지 | 설명 |
|------------|------|
| `realsense2_camera` | 카메라 드라이버 노드 |
| `realsense2_camera_msgs` | 커스텀 메시지 |
| `realsense2_description` | URDF/Xacro 모델 |

```bash
ros2 launch realsense2_camera rs_launch.py
```

---

## 🔧 삭제 가능한 불필요 파일/폴더

디스크 공간 절약을 원할 경우 아래 항목 삭제 가능:

| 경로 | 용량 | 설명 |
|------|------|------|
| `*/.git/` | ~273MB | Git 히스토리 (외부 패키지) |
| `*/.github/` | ~1MB | GitHub Actions 설정 |
| `FAST_LIO_ROS2/doc/` | ~128MB | 문서, GIF, PDF |
| `livox_ros_driver2/launch_ROS1/` | ~32KB | ROS1 런치파일 (불필요) |
| `serial/tests/` | ~100KB | 테스트 코드 |
| `serial/examples/` | ~50KB | 예제 코드 |

```bash
# 선택적 삭제 명령어
rm -rf src/*/.git src/*/.github
rm -rf src/FAST_LIO_ROS2/doc
rm -rf src/livox_ros_driver2/launch_ROS1
```

---

## 🛠️ 하드웨어 구성

| 구성요소 | 모델 | 역할 |
|----------|------|------|
| LiDAR | Livox MID-360 | 3D 포인트클라우드 + IMU |
| 카메라 | Intel RealSense D455 | RGB-D + IMU |
| 모터 드라이버 | MD (듀얼채널) x2 | 4륜 개별 제어 |
| 통신 | RS-485 | 모터 드라이버 통신 |

---

## 📊 센서 융합 (EKF)

```
┌─────────────────────────────────────────────────────┐
│                    EKF Filter                        │
│              (robot_localization)                    │
└─────────────────┬───────────────────────────────────┘
                  │
    ┌─────────────┼─────────────┬─────────────┐
    │             │             │             │
┌───▼───┐    ┌───▼───┐    ┌───▼───┐    ┌───▼───┐
│ odom0 │    │ odom1 │    │ imu0  │    │ imu1  │
│ (휠)  │    │(LiDAR)│    │(외장) │    │(D455) │
└───┬───┘    └───┬───┘    └───┬───┘    └───┬───┘
    │            │            │            │
┌───▼────┐  ┌───▼────┐   ┌───▼───┐   ┌───▼────┐
│diff_drv│  │FAST-LIO│   │  IMU  │   │RealSense│
│controller│ │        │   │       │   │  D455   │
└────────┘  └───┬────┘   └───────┘   └─────────┘
                │
          ┌─────▼─────┐
          │  MID-360  │
          └───────────┘
```
