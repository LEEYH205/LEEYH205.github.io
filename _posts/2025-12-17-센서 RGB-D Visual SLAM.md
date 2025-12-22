---
title: "센서 RGB-D Visual SLAM"
date: 2025-12-17 14:30:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, SLAM, Sensor]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

실습 환경 구성
- Black Coffee Robotics
- Black Coffee Robotics는 인도의 회사로서 자율 주행 및 로봇 공학 관련 전문 컨설팅 및 개발 서비스를 제공합니다.
    - 연구분야: 자율 주행 모바일 로봇, 지게차, 운송 트럭, 잔디 깎기 기계, 보트 및 드론 분야

## 로봇 시뮬레이션 환경 구성 및 설치

1. 시뮬레이션 환경 구성을 위해 아래 명령어로 패키지를 설치해줍니다.
    
    ```bash
    sudo apt-get install ros-humble-bcr-bot
    ```
    
2. 아래와 같은 설정으로 새 터미널에서 시뮬레이션을 열어줍니다.
    
    ```bash
    ros2 launch bcr_bot gazebo.launch.py \
    	camera_enabled:=True \
    	two_d_lidar_enabled:=True \
    	stereo_camera_enabled:=False \
    	position_x:=0.0 \
    	position_y:=0.0 \
    	orientation_yaw:=0.0 \
    	odometry_source:=odom \
    	world_file:=small_warehouse.sdf
    ```

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2014-32-24.png)


# RGB-D 기반의 SLAM 알고리즘

---

## RTAB-Map (Real-Time Appearance-Based Mapping)

![](/assets/img/ros2nav2/img/rtab-map.png)

RTAB Map SLAM Diagram

- **RTAB-Map**은 RGB-D 카메라, 스테레오 카메라, LIDAR 등 다양한 센서 조합등을 사용하여, 시각적 및 depth 정보를 기반으로 지도를 생성하고 자신의 위치를 동시에 추정하는 SLAM 기법입니다.

### 주요 특징

- **다양한 센서 지원**
    - RGB-D 카메라, 스테레오 카메라, 2D LiDAR, IMU, Wheel Odometry 등의 다양한 센서 지원
- **키프레임 기반의 접근법**
    - RTAB-Map은 전체 맵을 구성하기 위해 모든 프레임을 사용하지 않고, 중요한 프레임(키프레임)만을 사용 (메모리 절약)
- **Loop Closure Detection**
    - https://www.youtube.com/watch?v=joV7VCvGrKM
    - 로봇이 이전에 방문했던 장소를 다시 방문했을 때 이를 인식하고 위치추정을 개선하거나 지도의 연결성을 보장
- **Graph-based Optimazation**
    - 맵을 그래프로 표현하여 각 키프레임을 노드로, 키프레임 간의 이동을 엣지로 나타냄
    - 이 그래프를 최적화하여 로봇의 경로와 맵을 동시에 최적화
- **실시간 처리**
    - 로봇이 움직이는 동안에도 지속적으로 위치를 추정하고 맵을 업데이트

### 활용 예시

- 센서 조합
    - RGB-D Camera + Wheel Odometry + 2D LiDAR
        
        ![](/assets/img/ros2nav2/img/RGB-D%20Camera%20+%20Wheel%20Odometry%20+%202D%20LiDAR.png)
        
    - RGB-D Camera + 2D LiDAR
        
        ![](/assets/img/ros2nav2/img/RGB-D%20Camera%20+%202D%20LiDAR.png)
        
    - 2D LiDAR only
        
        ![](/assets/img/ros2nav2/img/2D%20LiDAR%20only.png)
        
    - 2D LiDAR + Wheel Odometry
        
        ![](/assets/img/ros2nav2/img/2D%20LiDAR%20+%20Wheel%20Odometry.png)
        
    - RGB-D Camera + Wheel Odometry
        
        ![](/assets/img/ros2nav2/img/RGB-D%20Camera%20+%20Wheel%20Odometry.png)
        
    - RGB-D Camera Only
        
        ![](/assets/img/ros2nav2/img/RGB-D%20Camera%20Only.png)
        
    - Stereo Camera
        
        ![](/assets/img/ros2nav2/img/Stereo%20Camera.png)


https://www.youtube.com/watch?v=qpTS7kg9J3A

https://www.youtube.com/watch?v=X885QsH0szo

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2014-50-17.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2014-48-53.png)

3. 위 토픽들을 RTAB-Map에서 사용할 수 있도록 적절한 argument와 함께 런치 파일을 실행시킵니다.
    
    ```bash
    ros2 launch rtabmap_launch rtabmap.launch.py \
       use_sim_time:=true \
       rgb_topic:=/kinect_camera/image_raw \
       depth_topic:=/kinect_camera/depth/image_raw \
       camera_info_topic:=/kinect_camera/camera_info \
       qos:=2 \
       publish_tf_odom:=false \
       frame_id:=base_link \
       args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --delete_db_on_start" \
       approx_sync:=true \
       rviz:=true
    ```
    
4. 로봇을 주행하면서 결과를 확인합니다.
    
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

![](/assets/img/ros2nav2/gif/Peek%202025-12-17%2014-56.gif)