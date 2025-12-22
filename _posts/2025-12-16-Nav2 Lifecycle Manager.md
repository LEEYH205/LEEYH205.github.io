---
title: "Nav2 Lifecycle Manager"
date: 2025-12-16 11:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, SLAM, Nav2 Lifecycle Manager]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Lifecycle이란?

• ROS2 lifecycle은 노드의 상태를 체계적으로 관리하기 위한 시스템

• 기존 ROS1과 달리 ROS2에서는 managed node라는 개념을 도입하여
노드의 상태 전환을 명시적으로 제어할 수 있음

• 관리되는 노드는 재시작, 일시 중지 또는 실행을 쉽게 제어할 수 있음
- unconfigured
- inactive
- active
- finalized

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2010-57-00.png)


## nav2_lifecycle_manager 패키지를 사용하는 이유

• nav2_lifecycle_manager 패키지는 Nav2 관련 노드의 상태를 제어하고 실행 상태를 수정할 수 있음

• Nav2 관련 노드들은 lifecycle_manager 노드에 의해 관리됨

• Nav2의 노드들을 nav2_lifecycle_manager로 관리하는 이유
- 노드 관리 및 조정
- 상태 기반 관리
- 자동 복구 및 오류 처리
- 효율적인 자원 사용
- 시스템의 투명성


실습

```
# localization launch file 실행
ros2 launch neuronbot2_nav localization_launch.py use_sim_time:=true

# nav2_map_server가 lifecycle manager에 의해 관리되는지 확인
ros2 service list | grep lifecycle


# 타입 확인
ros2 service type /lifecycle_manager_localization/manage_nodes

# 구성요소 확인
ros2 interface show nav2_msgs/srv/ManageLifecycleNodes

```

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-01-38.png)


```
# localization을 PAUSE 하기
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes command:\ 1
```
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-05-05.png)

```
# 다시 RESUME 하기
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes command:\ 2
```
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-16%2011-06-29.png)