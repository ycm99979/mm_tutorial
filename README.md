gz ign과 rviz에서 teleop keyborad로 조작 가능 
조작 명령어는 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

