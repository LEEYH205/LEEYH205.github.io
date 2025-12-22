---
title: "Controller Server 파라미터 튜닝"
date: 2025-12-18 11:20:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Controller, Parameter, tuning]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

```BASH
ros2 launch two_wheeled_robot factory_world_v1.launch.py
```
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-18%2011-24-23.png)
![](/assets/img/ros2nav2/gif/Peek%202025-12-18%2011-25.gif)

움직이는 물체, 사람도 시뮬레이션 환경에서 만들 수 있다.
actor plugin
https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin


# Controller 파라미터 튜닝

---

- Nav2의 `Controller`는 ROS1의 `Local Planner`와 동일합니다.
- `Controller`는 현재 위치에서 몇 미터 전방까지(센서 범위까지) 반응형 경로 계획을 수행합니다.
- 또한 동적 장애물(지도에는 나타나지 않지만 센서 데이터의 도움으로 감지할 수 있음)을 피하는 궤적을 만들면서 Global Plan을 따르려고 노력합니다.

## Controller Server Parameters

```yaml
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
    controller_plugins: ["FollowPath"]
    
    ...
```

- **`controller_frequency`** (double, 기본값: 20.0 hz)
    - Controller의 경로 계산 빈도
- **`min_x_velocity_threshold`** (double, 기본값: 0.0001 m/s)
    - 컨트롤러가 고려할 최소 x 속도 (이보다 낮은 값은 0.0m/s로 간주)
- **`min_y_velocity_threshold`** (double, 기본값: 0.0001 m/s)
    - 컨트롤러가 고려할 최소 y 속도 (이보다 낮은 값은 0.0m/s로 간주)
- **`min_theta_velocity_threshold`** (double, 기본값: 0.0001 rad/s)
    - 컨트롤러가 고려할 최소 각속도 (이보다 낮은 값은 0.0rad/s로 간주)
- **`failure_tolerance`** (double, 기본값: 0.0)
    - 호출된 컨트롤러 플러그인이 실패할 수 있는 최대 기간(초)
    - -1.0으로 설정하면 무한대, 0이면 비활성화
- **`progress_checker_plugin`** (string, 기본값: 'progress_checker')
    
    [**`progress_checker_plugin` 튜닝**](#progress_checker_plugin-튜닝)
    - 로봇의 주행 상황을 확인하기 위한 플러그인의 매핑된 이름
    
    | Plugin Name | Description |
    | --- | --- |
    | [SimpleProgressChecker](https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp) | 로봇이 주어진 시간 내에 최소한의 거리를 이동하여 목표를 향해 나아갈 수 있는지 확인 |
    | [PoseProgressChecker](https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/pose_progress_checker.cpp) | 로봇이 주어진 시간 내에 최소한의 거리 또는 각도를 이동하여 목표를 향해 나아갈 수 있는지 확인 |
- **`goal_checker_plugins`** (vector<string>, 기본값: ['general_goal_checker'])
    - 로봇이 목적지에 도달했는지 확인하기 위한 플러그인의 매핑된 이름 목록
    
    [**`goal_checker_plugins` 튜닝**](#goal_checker_plugins-튜닝)
    
    | Plugin Name | Description |
    | --- | --- |
    | [SimpleGoalChecker](https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_goal_checker.cpp) | 로봇이 목표의 병진 거리, 회전 거리 내에 있는지 확인 |
    | [StoppedGoalChecker](https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/stopped_goal_checker.cpp) | 로봇이 목표의 병진 거리, 회전 거리, 속도 임계값 내에 있는지 확인 |
- **`controller_plugins`** (vector<string>, 기본값: ['FollowPath'])
    - 컨트롤러 플러그인의 매핑된 이름 목록
    
    [**`controller_plugins` 튜닝**](#controller_plugins-튜닝)
    
    | Plugin Name | Description | Drivetrain support |
    | --- | --- | --- |
    | [DWB Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller) | 로봇의 동적 속도와 가속도를 고려하여 실시간 장애물 회피에 적합 | Differential, Omnidirectional, Legged |
    | [TEB Controller](https://github.com/rst-tu-dortmund/teb_local_planner/tree/ros2-master) | 시간 최적화 기반 경로 생성, 복잡한 환경에서 효율적이며 좁은 공간에서도 효율적으로 움직임 | **Ackermann**, Legged, Omnidirectional, Differential |
    | [Regulated Pure Pursuit](https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller) | 로봇이 주어진 경로를 추적하기 위해 목표점을 따라가는 단순하지만 효과적이며 경로를 곡선으로 매끄럽게 추적 | **Ackermann**, Legged, Differential |
    | [MPPI Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller) | 샘플 기반 최적화 알고리즘을 사용하여 복잡한 경로 계획과 제어를 수행 | Differential, Omnidirectional, **Ackermann** |
    | [Rotation Shim Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_rotation_shim_controller) | 회전 시 로봇을 미세하게 조정하여 목표 지점에 정확하게 정렬하도록 함 | Differential, Omnidirectional, model rotate in place |
    | [Graceful Controller](https://github.com/ros-planning/navigation2/tree/main/nav2_graceful_controller) | 작은 속도 변화로 안정적인 이동을 제공하며, 부드러운 이동 경로를 생성 | Differential |
    
    ```bash
    # Regulated Pure Pursuit 적용 예시
    ros2 launch two_wheeled_robot hospital_world_connect_to_charging_dock_v6.launch.py
    ```


---
---
# **`progress_checker_plugin` 튜닝**

## [SimpleProgressChecker](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html)

- 로봇이 지정된 시간 동안 일정 거리만큼 이동했는지 확인하면서 경로를 잘 따르고 있는지 평가

```yaml
controller_server:
  ros__parameters:

...

    **progress_checker_plugins: ["progress_checker"]**

...

		**progress_checker:
		      plugin: "nav2_controller::SimpleProgressChecker"
		      required_movement_radius: 0.5
		      movement_time_allowance: 10.0**

...
```

- **`required_movement_radius`** (double, 기본값: 0.5 m)
    - 로봇이 이 반경 내에서 충분히 이동하지 않으면 진행 중임을 간주하지 않음
- **`movement_time_allowance`** (double, 기본값: 10.0 s)
    - 로봇이 이동을 시작하고 진행 상황을 확인하기 위해 허용되는 시간(초)을 설정
    - 이 시간이 지나도 지정된 반경을 벗어나지 않으면 로봇이 경로를 잘 따르고 있지 않다고 판단

## [**PoseProgressChecker**](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/pose_progress_checker.html)

- [SimpleProgressChecker](#simpleprogresschecker)와 다르게 로봇의 위치 변화뿐만 아니라 **회전도 고려**하여 더 정밀한 진행 상황 확인이 가능

---
---
# **`goal_checker_plugins` 튜닝**

## [**SimpleGoalChecker**](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_goal_checker.html)

- 로봇이 목표 포즈에 도달했는지 확인

```yaml
controller_server:
  ros__parameters:

		...

    goal_checker_plugins: ["general_goal_checker"]
****
		...

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
****
		...
```

- **`stateful`** (bool, 기본값: true)
    - true: 로봇의 현재 상태와 이전 상태를 비교하여 목표 도달 여부를 확인
    - false: 로봇의 현재 상태만을 기준으로 목표 도달 여부를 평가
- **`xy_goal_tolerance`** (double, 기본값: 0.25 m)
    - 목표 도달 완료 기준을 충족하기 위한 허용 오차 (m)
- **`yaw_goal_tolerance`** (double, 기본값: 0.25 rad)
    - 목표 도달 완료 기준을 충족하기 위한 허용 오차 (rad)

## [**StoppedGoalChecker**](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/stopped_goal_checker.html)

- 로봇이 목표 포즈에 도달하여 정지했는지 확인


---
---
# **`controller_plugins` 튜닝**

# [DWB Controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)

![dwa.gif](/assets/img/ros2nav2/gif/dwa.gif)

- DWA (Dynamic Window Approach) 알고리즘을 바탕으로 개선되었으며, 로봇의 현재 속도와 가속도를 기반으로, 주어진 시간 내에서 가능한 속도와 방향을 평가하여 최적의 속도 명령을 생성
(ROS 1의 dwa_local_planner)

```yaml
controller_server:
  ros__parameters:

		...

    controller_plugins: ["FollowPath"]

		...

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

- **속도 관련 파라미터**
    - 로봇의 기계적 특성 및 작업 환경에 맞게 조정
    - **`min_vel_x`, `min_vel_y`, `max_vel_x`, `max_vel_y`** (double, 기본값: 0.0)
        - 로봇의 최소 및 최대 선속도를 x 및 y 방향으로 설정 $(m/s)$
    - **`max_vel_theta`, `min_speed_theta`** (double, 기본값: 0.0)
        - 로봇의 최소 및 최대 회전 속도를 설정 $(rad/s)$
    - **`min_speed_xy`, `max_speed_xy`** (double, 기본값: 0.0)
        - 로봇의 최소 및 최대 대각선 이동 속도를 설정 $(m/s)$
- **가속 및 감속 관련 파라미터**
    - **`acc_lim_x`, `acc_lim_y`, `acc_lim_theta`** (double, 기본값: 0.0)
        - x, y 방향 및 회전에 대한 최대 가속도 제한을 설정 $(m/s^2,rad/s^2)$
        - 값이 너무 높으면 움직임이 불규칙해지고, 값이 너무 낮으면 로봇이 반응이 느려짐
    - **`decel_lim_x`, `decel_lim_y`, `decel_lim_theta`** (double, 기본값: 0.0)
        - x, y 방향 및 회전에 대한 최대 감속도 제한을 설정 $(m/s^2,rad/s^2)$
- **`vx_samples`, `vy_samples`, `vtheta_samples`** (int, 기본값: 20, 5, 20)
    - 속도 샘플링에 사용되는 x, y 방향 및 회전 속도의 샘플 수를 설정
    - 경로 계획의 정밀도와 연산량 사이의 균형을 맞춰야 함
- **`sim_time`** (double, 기본값: 1.7)
    - 경로 계획을 위해 시뮬레이션하는 시간을 설정
    - 너무 긴 시간은 계산 부담을 증가시키며, 너무 짧은 시간은 최적의 경로를 찾지 못할 수 있음
- **`linear_granularity`, `angular_granularity`** (double, 기본값: 0.5, 0.025)
    - 로봇이 경로를 따라 이동할 때 계산되는 선형 위치 및 각도의 간격을 설정
    - 환경에 장애물이 많거나 경로가 좁고 꼬불꼬불한 경우 값을 작게하면 되지만 계산 부하가 증가
- **`transform_tolerance`** (double, 기본값: 0.1)
    - 트랜스폼 데이터의 허용 시간 오차를 설정
    - 너무 작은 값은 실시간 데이터 처리에서 동기화 문제를 일으킬 수 있고, 너무 큰 값은 오래된 데이터를 사용하게 만들어 정확성을 떨어뜨릴 수 있음
- **`xy_goal_tolerance`** (double, 기본값: 0.25)
    - 경로 추종(path following) 중에 각 중간 경유점(waypoint)에 대한 허용 오차를 정의
    - 작은 값은 정확한 위치 도달을 요구하지만, 너무 까다로울 수 있고, 큰 값은 더 빠른 목표 달성을 가능하게 하지만 정밀도가 떨어짐
    - 정밀한 조작이 필요한 작업에서는 작은 값으로 설정하고, 넓은 공간에서 빠른 이동이 중요한 경우에는 큰 값을 설정
- **`trans_stopped_velocity`** (double, 기본값: 0.25)
    - 로봇이 정지했다고 판단하는 속도 임계값을 설정
    - 로봇의 정지 동작과 관련된 센서의 반응 속도와 정밀도를 고려하여 설정
    - 너무 높은 값은 로봇이 실제로 정지하기 전에 멈춘 것으로 오인할 수 있으며, 너무 낮은 값은 로봇이 완전히 정지하기를 기다리게 만들어 응답성을 떨어뜨릴 수 있음
- **`short_circuit_trajectory_evaluation`** (bool, 기본값: true)
    - 조건을 만족하는 경로가 발견되면 나머지 경로 평가를 중단

## Trajectory Critics

- DWB 로컬 플래너에서 로봇의 경로를 평가하고 최적의 경로를 결정하기 위해 각각의 특정 측면을 설정 가능
    - [BaseObstacleCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/base_obstacle.html)
        - 장애물 회피에 대한 가중치를 설정
        - 값이 클수록 로봇이 장애물을 피하는 데 더 많은 중요성을 둠
    - [GoalAlignCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/goal_align.html)
        - 로봇이 목표 지점에 도달할 때 올바른 방향을 향하도록 유도하며, 목표 지점에 도착했을 때 로봇의 자세가 정확하게 목표를 향하도록 함
    - [GoalDistCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/goal_dist.html)
        - 로봇이 목표 지점까지의 거리를 얼마나 중요하게 여길지를 결정
        - 목표에 빠르게 도달해야 하는 경우 이 값을 높임
        - 목표 지점 근처에 도달하는 것만으로 충분하다면 값을 낮춤
    - [ObstacleFootprintCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/obstacle_footprint.html)
        - footprint가 장애물과 충돌하는지 여부를 평가하여, 충돌을 방지하고 안전한 경로를 선택하도록 함
    - [OscillationCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/oscillation.html)
        - 경로 추종 중 발생하는 oscillation 현상을 방지하도록 가중치 부여
        - 좁은 공간이나 복잡한 환경에서는 진동이 발생하기 쉬우므로, 이러한 환경에서는 scale 값을 높임
    - [PathAlignCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/path_align.html)
        - 로봇이 글로벌 경로와 정렬하는 데 얼마나 중점을 둘지를 결정
        - 로봇이 정해진 경로를 정확하게 따르도록 해야 하는 경우 이 값을 높임
    - [PathDistCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/path_dist.html)
        - 로봇이 글로벌 경로에 얼마나 근접하게 위치할지를 결정
        - 글로벌 경로를 최대한 따르는 것이 중요하다면 이 값을 높임
        - 로봇이 경로를 벗어날 가능성이 적은 환경에서는 값을 낮춤
    - [PreferForwardCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/prefer_forward.html)
        - 로봇의 경로 계획에서 후진보다 전진을 더 우선시하도록 가중치 부여
        - 로봇이 후진할 때 장애물 감지와 회피가 어려운 경우에 유용
    - [RotateToGoalCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/rotate_to_goal.html)
        - 로봇이 목표 위치에 충분히 가까워졌을 때만 목표 방향으로 회전하도록 가중치 부여
    - [TwirlingCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/twirling.html)
        - 홀로노믹 로봇이 목표를 향해 이동하는 동안 회전하는 것을 방지하도록 가중치 부여