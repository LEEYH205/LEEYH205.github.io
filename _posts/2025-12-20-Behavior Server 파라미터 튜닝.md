---
title: "Behavior Server 파라미터 튜닝"
date: 2025-12-20 11:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Behavior, Parameter, tuning]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

```BASH
ros2 launch two_wheeled_robot neighborhood_world_v3.launch.py
```

# Behavior Server 파라미터 튜닝

---

- `Behavior Server`는 특정 상황이나 요구 사항에 따라 로봇의 동작(Behavior)을 제어하고 관리하는 기능을 제공
- 다양한 복구 동작과 일반적인 행동을 보다 일관되고 통합된 방식으로 관리하기 위해, 기존 `Recovery`라는 용어가 `Behavior`로 통합

## Behavior Server Parameters

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_costmap_topic: global_costmap/costmap_raw
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_timeout: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
    enable_stamped_cmd_vel: false
```

- **`cycle_frequency`** (double, 기본값: 10.0)
    - Behavior Server의 주기적인 실행 빈도를 설정
- **`behavior_plugins`** (vector<string>, 기본값: [“spin”, “back_up”, “drive_on_heading”, “wait”])
    
    [behavior_plugins 튜닝](#behavior_plugins-튜닝)
    
    - 특정 상황에서 필요한 행동이 있다면 해당 행동을 플러그인 목록에 추가
    
    | Namespace | Plugin | Description |
    | --- | --- | --- |
    | “spin” | “nav2_behaviors::Spin” | 로봇이 제자리에서 회전하는 행동
    (장애물을 감지하고 회피하기 위해 로봇이 회전해야 할 때 사용) |
    | “backup” | “nav2_behaviors::BackUp” | 로봇이 후진하는 행동
    (로봇이 장애물에 막혔을 때 후진하여 장애물에서 벗어나는 데 사용) |
    | “drive_on_heading” | “nav2_behaviors::DriveOnHeading” | 특정 방향으로 직진하는 행동
    (로봇이 특정 방향으로 이동해야 할 때 사용) |
    | “wait” | “nav2_behaviors::Wait” | 일정 시간 동안 대기하는 행동
    (특정 조건이 충족될 때까지 로봇이 대기해야 할 때 사용) |
    | “assisted_teleop" | ”nav2_behaviors::AssistedTeleop” | 원격 조작을 보조하는 행동
    (원격으로 로봇을 제어해야 할 때 사용) |
    
   
    
    - [구미시 1호 ‘로봇 주무관’, 계단서 추락 ‘작동 중단’](https://www.idaegu.com/news/articleView.html?idxno=607093)


    ---
# behavior_plugins 튜닝

## **Spin Behavior**

- **`simulate_ahead_time`** (double, 기본값: 2.0)
    - 로봇의 행동을 예측하고 시뮬레이션하는 시간 범위를 설정
- **`max_rotational_vel`** (double, 기본값: 1.0)
    - 로봇의 최대 회전 속도를 설정
- **`min_rotational_vel`** (double, 기본값: 0.4)
    - 로봇의 최소 회전 속도를 설정
- **`rotational_acc_lim`** (double, 기본값: 3.2)
    - 로봇의 회전 가속도 제한을 설정
    - 로봇이 부드럽게 회전할 수 있도록 설정하며, 로봇의 회전 특성에 맞게 값을 조정
- **`enable_stamped_cmd_vel`** (bool, 기본값: false)
    - cmd_vel 명령에 Timestamp를 포함할지 여부

## **BackUp Behavior**

- **`simulate_ahead_time`** (double, 기본값: 2.0)
    - 로봇의 행동을 예측하고 시뮬레이션하는 시간 범위를 설정
- **`enable_stamped_cmd_vel`** (bool, 기본값: false)
    - cmd_vel 명령에 Timestamp를 포함할지 여부

## **DriveOnHeading Behavior**

- **`simulate_ahead_time`** (double, 기본값: 2.0)
    - 로봇의 행동을 예측하고 시뮬레이션하는 시간 범위를 설정
- **`enable_stamped_cmd_vel`** (bool, 기본값: false)
    - cmd_vel 명령에 Timestamp를 포함할지 여부
- **`bond_heartbeat_period`** (double, 기본값: 0.1)
    - Bond 메커니즘에서 특정 행동이 실행 중임을 나타내기 위해 주기적으로 신호를 보내는 간격을 설정
        - Bond: 두 노드 간의 연결 상태를 모니터링하고, 연결이 끊어지거나 노드가 죽으면 이를 감지하는 메커니즘

## **AssistedTeleop Behavior**

- **`projection_time`** (double, 기본값: 1.0)
    - 로봇이 미래에 어느 지점에 있을지를 예측하기 위해 사용하는 시간
    - 로봇의 현재 속도를 기반으로 로봇의 미래 위치를 예측하여, 원격 조작을 보다 원활하게 만들기 위해 사용
    - 짧은 시간은 더 즉각적인 반응을 제공하지만, 원격 조작의 예측이 덜 정확할 수 있음
- **`simulation_time_step`** (double, 기본값: 0.1)
    - 로봇의 상태를 시뮬레이션할 때 사용되며, 로봇의 움직임을 더 세밀하게 제어할 수 있도록 함
    - 더 작은 간격은 시뮬레이션의 정확성을 높이지만 계산 부하가 증가할 수 있음
- **`cmd_vel_teleop`** (string, 기본값: “cmd_vel_teleop”)
    - 원격 조작 명령을 받을 토픽의 이름을 설정
- **`enable_stamped_cmd_vel`** (bool, 기본값: false)
    - cmd_vel 명령에 Timestamp를 포함할지 여부



---
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2010-59-15.png)
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2010-59-59.png)

 ```bash
    # DriveOnHeading behavior 테스트
    ros2 action send_goal /drive_on_heading nav2_msgs/action/DriveOnHeading "{target: {x: 1.0, y: 0.0, z: 0.0}, speed: 0.2}"
```
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-02.gif)
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-03.gif)


```bash
    # BackUp behavior 테스트
    ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: {x: -2.0, y: 0.0, z: 0.0}, speed: 0.2}"
```
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-06.gif)

```bash
    # Spin behavior 테스트
    ros2 action send_goal /spin nav2_msgs/action/Spin target_yaw:\ 3.14
```
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-08.gif)


---
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-12.gif)
`Running spin`,
`Running backup`,
`Running wait`,
`Running backup`,
등을 수행하다가 갈 일이 없으니, failed 뜨는 것을 볼 수 있다.
`[ERROR]: Goal failed`

