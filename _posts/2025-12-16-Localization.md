---
title: "Localization"
date: 2025-12-16 11:10:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, SLAM, Localization]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# 필수 Frame 개념 총정리

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-12-10.png)

• map
- 로봇이 동작하는 전체 환경에 대한 지도의 기준을 나타내는 프레임
- 고정된 지도 상에서 로봇의 위치를 추정하는 데 사용됨

• odom
- 로봇의 시작 위치를 기준으로 한 로봇의 상대적 위치를 나타내는 프레임
- 오도메트리 정보는 주로 바퀴의 회전이나 기타 이동 센서에서 파생됨
- 이 프레임을 기준으로 publish되는 오도메트리 데이터는 시간이 지남에 따라 오류가 누적될 수 있어 절대적 위치 추정에는 한계가 있음

• base_footprint
- 로봇의 바닥을 나타내는 프레임
- 로봇이 물리적으로 어떻게 지면 위에 위치하고 있는지를 정의

• base_link
- 일반적으로 로봇의 몸체를 나타내며, 모든 센서나 운동 부품은 이 프레임에 상대적으로 위치함
- 이 프레임은 로봇의 현재 물리적 구조(바퀴, 센서 등)와 직접 연결됨



# \[중요!] Frame간의 관계

• odom → base_link
- 오도메트리 정보를 기반으로, 로봇의 상대적인 움직임(예: 전진, 회전 등)을 추정

- 이 변환은 로봇의 물리적 움직임을 바탕으로 계산되므로, 실시간으로 업데이트됨

- 모바일 로봇에서는 보통 Wheel Odometry를 사용하거나 칼만 필터를 이용해
IMU와 결합하여 사용함

• map → odom
- Localization 알고리즘은 지속적으로 map 프레임에서 odom 프레임으로의 변환을 업데이트하여, 지도 상의 **로봇의 추정된 위치와 실제 오도메트리 사이의 불일치를 보정**함

- 즉, 로봇의 odom 프레임은 map 프레임에 대한 상대적 위치를 알고 있고, 결국 base_link 프레임과 odom 프레임에 직접 연결되어 있기 때문에 **로봇은 맵을 기준으로 자신의 위치를 알 수 있음**

## ROS 2 시스템에서 Localization을 위해 필요한 노드

• map_server 노드

• localization 알고리즘을 수행하는 노드

• lifecycle_manager 노드

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-18-16.png)


# 2D LiDAR 기반의 Localization Package

## AMCL (Adaptive Monte Carlo Localization)
https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl

• Monte Carlo Localization (MCL)을 기반으로 하며, 로봇이 지도 상에서 자신의 위치를 찾을 수 있도록 도와주는 확률적 접근 방식
• 예를 들어 로봇의 위치 추정에 필요한 파티클의 수를 동적으로 조정할 수 있으며, 위치의 불확실성이 크면 더 많은 입자를 사용하고, 확실성이 높아지면 입자의 수를 줄여 계산 효율을 높일 수 있음

- 사용자가 파라미터를 직접 구성하고, 수정할 수 있음.
- 로봇이 항상 예상대로 움직이지 않음. 휠사이즈, 미끄러움 등
- 로봇이 어디로 움직일 지 무작위로 추정함.
- 내 초기 위치를 중심으로 파티클을 뿌림
    - 실제의 지도상의 이 위치에 있겠다하는 정보가 들어있음. 가능성이 높은 포즈에 대한 정보가 있음.
- 확률을 어떻게 줄여나가냐면,
    - 라이다와 같은 센서로 주변 환경을 관측할 때, 맵이랑 잘 일치할 수 있도록 하는 파티클들은 유지하고, 낮은 파티클들은 점점 버림.
    - 로봇이 이동하면서 이동량과 센서 측이한 값을 바탕으로 주변 포즈들은 점점 버려지고, 가능성이 높은 포즈들로 수렴하게 된다.
    - 로봇이 계속 움직이면 움직일 수록 내가 지도상에서 어느곳에 위치하고 있을지 확률적으로 파악할 수 있게 된다.

- 파티클도 넓게 많이 뿌리겠다, 적게 뿌려서 컴퓨팅파워를 아끼겠다를 사용자가 설정할 수 있다.


![](/assets/img/ros2nav2/gif/Peek%202025-12-16%2011-42.gif)
AMCL 테스트

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-44-34.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-46-32.png)
처음 파티클을 뿌린걸 보면 위치가 안맞는 것을 볼 수 있다.
localization이 잘 안된 것임.

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-49-17.png)
로봇을 움직여보면, 점점 localization이 되어가는 것을 볼 수 있다.
하지만, 아직 확률이 남아있는 부분이 있다.

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-50-39.png)
로봇을 계속 움직여보면, 파티클이 높은 확률만 남아있는 것을 알 수 있다.

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-44-51.png)
![](/assets/img/ros2nav2/gif/Peek%202025-12-16%2011-54.gif)
map_frame기준으로 odom_frame의 관계가 조정되는 것을 확인할 수 있다.
담당 : localization node

odom_frame 기준으로 로봇의 위치가 잘못 추정될 수 있으니까,
map_frame 기준으로 odem_frame간의 관계를 조절해주면서, 최종적으로 맵을 기준으로 올바르게 그 로봇의 위치를 추정할 수 있도록 도와줌.

## 초기값의 중요성
대략적인 위치랑 방향의 초기값이 중요하다.
그래야, localization이 잘된다.

![](/assets/img/ros2nav2/gif/Peek%202025-12-16%2012-03.gif)
-> 초기 값이 아예 모른다. 랜덤으로 localization을 해야할 때 쓰는 전략이 있다.

-> global localization