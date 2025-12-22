---
title: "SLAM에서의 Ground Truth"
date: 2025-12-17 15:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, SLAM, Ground Truth]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Ground Truth

• 시스템이나 알고리즘이 얼마나 정확하게 동작하는지를 평가하기 위해 사용되는 참조 데이터

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-03-03.png)


## SLAM에서 Ground Truth는?

• 로봇이나 드론, 차량이 탐색하는 환경의 정확한 위치, 지도, 궤적 등의 정보

• 사용 목적
1. 알고리즘 검증
2. 성능 비교
3. 오류 분석

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-04-04.png)


# Ground Truth를 얻는 방법
## 실외 (Outdoor)

### RTK(Real Time Kinematic) GPS 또는 Network RTK GPS
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-07-44.png)
• 이동형 수신기와 기준국 사이의 상대적인 위치를 매우 높은 정확도로
계산하는 고정밀 위치 측정 기술

• 장점
- 센티미터 수준의 정확도
- 실시간으로 보정 데이터를 수신

• 단점
- 로버가 실시간으로 보정 데이터를 수신하기 위해 인터넷 연결이 필요
- 넓은 지역에 기준국 네트워크를 설치하고 유지하는 데 들어가는 비용
- 도심지에서의 건물 밀집, 산악 지형, 나무 등 장애물로 인해 신호 수신

## 실내 (Indoor)
### Laser Tracker
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-08-17.png)
• 레이저 빔을 사용하여 3차원 공간에서 물체의 위치를 매우 정밀하게
측정하는 장비 (주로 산업용 측정이나 정밀 공학에서 사용)

• 장점
- μm 이하의 정확도와 수십 m 이상의 거리를 측정
- 실시간 측정 및 추적

• 단점
- 높은 가격
- 측정 대상에 반사체 부착 필요
- 반사율이 낮은 표면이나 장애물이 있는 경우 측정이 어려움

### Motion Capture Camera
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-08-43.png)
• 다수의 적외선 카메라를 사용하여 마커의 3차원 위치를 측정하는 장비

• 장점
- 초당 수백에서 수천 프레임을 촬영하여 빠른 움직임도 정확하게 추적
- 다수의 마커 동시 측정 가능

• 단점
- 높은 가격과 넓은 공간에서는 많은 카메라 필요
- 대상에 마커 부착 필요
- 카메라 설치 및 캘리브레이션 필요

### AR Tag
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-09-08.png)
https://berndpfrommer.github.io/tagslam_web/

• 특정 패턴이 인쇄된 마커를 카메라로 인식하여 마커의 3차원 위치 및 자세를
추정하는 기술

• 장점
- 저비용 그리고 쉬운 설치와 사용
- 일반 카메라로 마커 인식 가능

• 단점
- 정확도가 Laser Tracker나 Motion Capture Camera에 비해 낮음
- 카메라의 해상도와 인식 거리 제한
- 태그가 카메라에 명확하게 보여야 함 (시야에 들어와야 함)

# SLAM 성능 평가 방법
## EVO
https://github.com/MichaelGrupp/evo

• SLAM 알고리즘의 평가 및 벤치마킹을 위해 사용되는 Python 기반의 툴

• 지원 포맷
- TUM trajectory files
- KITTI pose files
- EuRoC MAV (.csv groundtruth and TUM trajectory file)
- ROS and ROS2 bagfile
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-14-09.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-14-21.png)

### Absolute Trajectory Error (ATE)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-15-18.png)

### Relative Pose Error (RPE)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-17%2015-15-51.png)