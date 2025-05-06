# 라이다 센서 세팅하기

1. 라이다를 라즈베리 파이에 연결 후 몇번 USB에 연결되어있는지 확인
   ```bash
   $ ls -l /dev/ttyUSB*
   ```

2. 만약 0번일 경우, 다음 명령어로 기기 정보 확인 (KERNELS, ATTRS{idProduct}, ATTRS{idVendor}의 값을 확인)
   ```bash
   $ sudo udevadm info -q all -a /dev/ttyUSB0
   ```
   
3. 2에서 확인한 정보를 토대로 rules 생성
   ```bash
   $ sudo nano /etc/udev/rules.d/99-lidar.rules
   ```

   ```bash
   # 아래 내용 기입. 기입 후 ctrl+o 로 저장 후 엔터 그리고 ctrl+x로 나가기
   KERNEL=="ttyUSB*", KERNELS=="4-1", ATTRS{idProduct}=="ea60",
    ATTRS{idVendor}=="10c4", MODE:="0777", SYMLINK+="rplidar"
   ```

   * 주의할점: esp32와 라이더 센서의 product id와 vender id가 동일하여 이 값만으로는 기기를 구분할수 없어서 KERNELS까지 잘 기입해주어야함.
     
4. rules 를 리로드 하고 잘 적용되었는지 확인
  ```bash
  $ sudo udevadm control --reload-rules
  $ sudo udevadm trigger
  $ ls -al /dev/rplidar
  ```

  ```bash
  # 다음과 같이 나오면 성공
  /dev/rplidar -> /dev/ttyUSB0
  ```

5. Slamtec rplidar_ros 설치
   ```bash
   $ cd ~/robot_ws/src
   $ git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   ```
   
6. rplidar_ros 빌드
   ```bash
   $ source /opt/ros/humble/setup.bash
   $ cd ..
   $ colcon build --package-select rplidar_ros
   ```

7. 실제 작동 확인하기
   ```bash
   $ source ~/robot_ws/install/setup.bash
   $ ros2 launch rplidar_ros rplidar_a1_launch.py
   ```

   ```bash
   $ source /opt/ros/humble/setup.bash
   $ source ~/robot_ws/install/setup.bash
   $ ros2 launch rplidar_ros view_rplidar_a1_launch.py
   ```

   ```bash
   $ source /opt/ros/humble/setup.bash
   $ source ~/robot_ws/install/setup.bash
   $ ros2 launch monicar2_bringup rplidar.launch.py
   ```

  ```bash
   $ source /opt/ros/humble/setup.bash
   $ source ~/robot_ws/install/setup.bash
   $ ros2 topic list -t
   $ ros2 topic echo --once /scan
   ```
