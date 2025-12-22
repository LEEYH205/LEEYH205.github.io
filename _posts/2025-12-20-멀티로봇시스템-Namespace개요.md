---
title: "멀티로봇시스템-Namespace개요"
date: 2025-12-20 16:20:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, MultiRobotSystem, Namespace]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---
# namespace란?

• 센서 데이터에 의존하는 모든 ROS 노드가 제대로 작동하고 로봇을 개별적으로
제어하려면, 각 로봇마다 다른 네임스페이스(namespace)를 사용해야 함

• 네임스페이스를 사용하면 서로 다른 토픽, TF 프레임 및 노드 간의 이름에 대해
충돌을 방지할 수 있음

• 멀티 로봇 시스템을 위해서는 각 로봇이 다음에 대해 특정 이름을 가지고 있는지
확인해야 함
- node
- topic
- frame


## namespace (1) - node
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2016-19-45.png)

## namespace (2) - topic
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2016-20-10.png)

## namespace (3) - frame
```DASH
ros2 run tf2_tools view_frames
```
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2016-21-29.png)
