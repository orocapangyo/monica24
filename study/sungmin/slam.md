# SLAM
## SLAM이란?
- Simultaneous Localization And Mapping 의 약자
  - Simultaneous (동시 수행)
  - Localization (위치 추정)
    - 센서 데이터를 이용해 로봇이 현재 어디에 있는지를 파악합니다.
  - Mapping (지도 작성)
    - 주변 환경을 스캔하여 지도를 만듭니다.
- 동시에 위치 추정과 지도 작성을 수행하는것. 즉, 자신의 위치를 추정하면서 동시에 주변 환경의 지도를 생성하는 기술

## SLAM과 Navigation의 차이
- SLAM은 Navigation의 전제 조건일 수 있음:
  - 로봇이 미지의 환경에서 움직이기 위해선 먼저 지도를 만들어야 하고, 자신의 위치를 알아야 합니다. 이때 SLAM이 필요합니다.
- Navigation은 SLAM의 결과를 사용함:
  - 이미 SLAM으로 생성된 지도와 위치 정보를 바탕으로 로봇이 어떻게 움직일지를 결정하는 게 Navigation입니다.
 
## monicar2로 SLAM 하는 법
### 1. 라즈베리파이에서 ekfPose 실행
```shell
ros2 launch monicar2_localization ekfPose.launch.py initPose:='false'
```
- ekfPose는 확장 칼만 필터 (Extended Kalman Filter, EKF) 를 사용해 로봇의 자세(pose: 위치 + 방향)를 추정하는 것을 의미
- `ekfPose.launch.py` 에는 다음과 같은 작업 수행이 포함됨
  - initPose, use_des, use_joy을 수행하며, 설정값에 따라 다음을 조건적으로 실행:
    - odomPubInit: 항상 실행 (EKF 관련 odometry publish)
    - RViz 클릭 UI: initPose == false일 때 실행
    - 로봇 모델 로딩: use_des == true일 때 실행
    - 조이스틱 제어: use_joy == true일 때 실행
  - 특히 **odomPubInit** 은 아래 작업을 수행
    - monicar2_bringup 패키지의 bringup launch 파일을 실행 
    - odomPublisher 노드를 실행하여 EKF 초기화 및 Odometry 데이터를 퍼블리시
  - bringup
    - bringup은 로봇 기본 노드들(센서, tf, 상태 퍼블리셔 등)을 실행
      - MCU 제어 (`mcu.launch.py`)
      - IMU 센서 변환 (`mpu6050.launch.py`)
      - LiDAR 구동 (`rplidar.launch.py`)

### 2. PC에서 cartographer 실행
 ```shell
 ros2 launch monicar2_cartographer cartographer.launch.py
 ```
   - Cartographer란?
     - LiDAR, IMU, (카메라) 데이터를 사용하여 2D 또는 3D 환경 지도를 생성하고 위치를 추정하는 SLAM 프레임워크
   - `cartographer.launch.py` 에서는 다음과 같은 작업을 수행
     - Cartographer 노드(cartographer_node) 실행
     - Occupancy Grid를 퍼블리시하는 노드 실행 (occupancy_grid.launch.py 포함)
     - 설정 파일 경로, 맵 해상도, 퍼블리시 주기 등 파라미터 정의 및 전달

### 3. PC의 다른 터미널에서 rviz 실행
```shell
ros2 launch monicar2_cartographer cartographer_rviz.launch.py
```
- RViz란?
  - ROS에서 퍼블리시되는 토픽 데이터를 시각적으로 보여주는 툴
- `cartographer_rviz.launch.py`에서 다음과 같은 작업을 수행
  - use_sim_time: Gazebo(시뮬레이터)와 동기화를 위한 시뮬레이션 시간 사용 설정
  - cartographer.rviz: SLAM 시각화를 위한 사전 설정된 RViz 레이아웃
  - **rviz2 노드 실행**: 해당 RViz 구성으로 GUI 실행, 실시간 지도, scan, 위치 확인 가능

![Screenshot from 2025-05-23 22-03-42](https://github.com/user-attachments/assets/04d78ca6-6b1d-4fdd-a7f7-64d45c366c9d)

