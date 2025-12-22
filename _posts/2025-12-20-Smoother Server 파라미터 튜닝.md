---
title: "Smoother Server 파라미터 튜닝"
date: 2025-12-20 10:20:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Smoother, Parameter, tuning]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

```bash
ros2 launch two_wheeled_robot house_world_v1.launch.py  
```

# **Smoother 파라미터 튜닝**

---

- `Soomther`는 로봇의 경로를 보다 부드럽게 만드는 데 사용되는 모듈입니다.
- 주로 로봇이 목표 지점으로 이동할 때 경로의 급격한 회전이나 불규칙한 움직임을 최소화하여 더욱 자연스럽고 효율적으로 움직이게 하는 역할을 합니다.
    - **경로 평활화(Path Smoothing)**
        - 초기 계획된 경로가 직선 구간과 급격한 회전을 포함하고 있을 수 있기에, 이러한 경로를 매끄럽게 조정하여 로봇의 움직임이 부드럽게 이어지도록 합니다.
    - **경로 최적화(Path Optimization)**
        - 주어진 경로를 따라 이동할 때, 로봇의 운동 역학 및 환경 조건을 고려하여 cost를 줄이거나 이동 시간을 최소화하는 방향으로 경로를 수정합니다.
    - **안전성 확보(Safety Assurance)**
        - 부드러운 경로를 생성함으로써 로봇의 급격한 속도 변화나 방향 전환으로 인한 사고를 방지하고, 더욱 안전하게 목표 지점에 도달할 수 있도록 합니다.


## Smoother Server Parameters

```yaml
smoother_server:
  ros__parameters:
    costmap_topic: global_costmap/costmap_raw
    footprint_topic: global_costmap/published_footprint
    robot_base_frame: base_link
    transform_timeout: 0.1
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      do_refinement: True
      refinement_num: 2
      max_its: 1000
      w_data: 0.2
      w_smooth: 0.3
```

- **`costmap_topic`** (string, 기본값: "global_costmap/costmap_raw")
    - 충돌 검사를 위한 costmap topic
- **`footprint_topic`** (string, 기본값: "global_costmap/published_footprint")
    - costmap내에서 사용할 footprint topic
- **`transform_tolerance`** (double, 기본값: 0.1)
    - TF 정보 허용 시간
- **`smoother_plugins`** (vector<string>, 기본값: ["simple_smoother"])
    - 사용할 smoother 플러그인의 매핑된 이름 목록
    
    [**`smoother_plugins` 튜닝**](#smoother_plugins-튜닝)




---
# **`smoother_plugins` 튜닝**

## [**Simple Smoother**](https://docs.nav2.org/configuration/packages/configuring-simple-smoother.html)

- `Simple Smoother`는 입력 경로를 받아 간단하고 빠른 스무딩 기법을 사용하여 경로를 부드럽게 만듭니다.
- 초기 경로 포인트와 평활화(smoothing)된 경로 포인트에 가중치를 부여하여, 경로가 기존 특성을 유지하면서 진동이나 들쭉날쭉한 부분을 줄이는 균형 잡힌 결과를 생성합니다.

```yaml
smoother_server:
  ros__parameters:

		...

    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      do_refinement: True
      refinement_num: 2
      max_its: 1000
      w_data: 0.2
      w_smooth: 0.3
```

- **`tolerance`** (double, 기본값: 1.0e-10)
    - 경로 평활화 알고리즘이 수렴했다고 간주하는 허용 오차
    - 경로의 변화가 이 값보다 작아지면 알고리즘이 더 이상 경로를 조정하지 않음
    - 작은 값일수록 경로가 더 정확하게 조정되지만, 계산 시간이 길어질 수 있음
- **`do_refinement`** (bool, 기본값: True)
    - 경로 평활화 후 추가적으로 경로를 세밀하게 조정할지 여부를 결정
    - True: 추가적인 경로 세부 조정(refinement)이 수행
- **`refinement_num`** (int, 기본값: 2)
    - 경로 세부 조정(refinement)을 수행할 때 반복 횟수를 설정
    - 이 값이 클수록 경로가 더 부드럽게 조정될 수 있지만, 계산 시간이 늘어날 수 있음
- **`max_its`** (int, 기본값: 1000)
    - 평활화 알고리즘의 최대 반복 횟수를 설정
    - 이 값을 초과하면 알고리즘은 수렴하지 않았더라도 반복을 멈춤
    - 너무 큰 값은 불필요하게 계산 시간을 늘릴 수 있음
- **`w_data`** (double, 기본값: 0.2)
    - data 항의 가중치 (원래 경로를 얼마나 따르게 할지를 결정)
    - 값이 클수록 원래 경로와의 차이를 줄이려는 경향이 강해짐
- **`w_smooth`** (double, 기본값: 0.3)
    - smooth 항의 가중치를 설정(경로를 얼마나 부드럽게 할지를 결정)
    - 값이 클수록 경로가 더 부드럽게 조정


![](/assets/img/Peek%202025-12-20%2010-49.gif)
`/cmd_vel` 평탄화된 걸 볼 수 있다.