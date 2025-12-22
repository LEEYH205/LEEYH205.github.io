---
title: "Planner Server 파라미터 튜닝"
date: 2025-12-18 11:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Planner, Parameter, tuning]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# 로봇 시뮬레이션 환경 구성 및 설치

---

1. 아래 패키지를 `~/Downloads` 경로에 저장합니다.
    
    [two_wheeled_robot.zip](https://prod-files-secure.s3.us-west-2.amazonaws.com/165a18af-9d8a-4762-809c-e48f1e13d66d/3a68038f-74cd-43a3-aad3-199e6807a04d/two_wheeled_robot.zip)
    
2. 압축을 풀어준 뒤 아래 명령어를 통해 `~/nav2_ws/src` 디렉토리로 옮겨줍니다.
    
    ```bash
    cp -r ~/Downloads/two_wheeled_robot ~/nav2_ws/src/two_wheeled_robot
    ```
    
3. 의존성 패키지들을 설치 후 빌드해줍니다.
    
    ```bash
    cd ~/nav2_ws
    rosdep install --from-paths src/two_wheeled_robot -y --ignore-src --os=ubuntu:jammy
    colcon build --symlink-install --packages-select two_wheeled_robot
    source ~/ros2_ws/install/local_setup.bash # 환경에 따라 local_setup.zsh
    ```
    
4. 시뮬레이션 환경이 잘 열리는지 확인해줍니다.
    
    ```bash
    ros2 launch two_wheeled_robot lawn_world_v1.launch.py
    ```


# Planner 파라미터 튜닝
```BASH
ros2 launch two_wheeled_robot car_world_v1.launch.py
```
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-18%2011-22-00.png)

# Planner 파라미터 튜닝

---

- Nav2의 `Planner`는 ROS1의 `Global Planner`와 동일합니다.
- `Planner`는 지도의 알려진 장애물 및 벽 정보를 바탕으로 로봇이 A 지점에서 B 지점까지 이동하는 경로를 찾습니다.


## Planner Server Parameters

- **`expected_planner_frequency`** (double, 기본값: 20)
    - Planner의 경로 계산 빈도
- **`planner_plugins`** (vector<string>, 기본값: [‘GridBased’])
    
    [**`planner_plugins` 튜닝** ](#planner_plugins-튜닝)
    
    - 사용을 위해 가져올 플래너 플러그인의 맵핑된 이름 리스트
    
    | Plugin Name | Description | Drivetrain support |
    | --- | --- | --- |
    | [NavFn Planner](https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner) | A* 또는 Dijkstra 기반의 최단 경로 탐색 | Differential, Omnidirectional, Legged |
    | [SmacPlannerHybrid](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner) | 하이브리드 A* 알고리즘 기반, 복잡한 지형에서 효율적이며 3D 환경을 지원 | **Ackermann**, Differential, Omnidirectional, Legged |
    | [SmacPlanner2D](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner) | 2D 환경용, 빠른 경로 계획을 제공하며 저사양 하드웨어에서도 효율적 | Differential, Omnidirectional, Legged |
    | [SmacPlannerLattice](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner) | Grid 기반 경로 계획, 다양한 경로 탐색 및 분석에 적합 | Differential, Omnidirectional, Ackermann, Legged, Arbitrary / Custom |
    | [ThetaStarPlanner](https://github.com/ros-planning/navigation2/tree/main/nav2_theta_star_planner) | A* 알고리즘을 확장해 직선 경로를 선호, 더 짧고 현실적인 경로를 생성 | Differential, Omnidirectional |
    
    ```yaml
    planner_plugins: ["**GridBased**"]
    **GridBased:**
      plugin: "nav2_navfn_planner/NavfnPlanner"
    ```
    
    - 기타 플러그인 yaml 작성 방법은 각 Plugin Name의 링크를 참고.


    ---


# **`planner_plugins` 튜닝**

# [Navfn Planner](https://docs.nav2.org/configuration/packages/configuring-navfn.html)

- NavfnPlanner는 두 지점 사이의 경로를 찾기 위해 `A*` 또는 `Dijkstra` 알고리즘을 사용합니다.
- NavfnPlanner은 ROS 1 때 부터 약 10년 이상 매우 안정적이었기 때문에, ROS 2에서는 다른 플래너의 포팅 없이 기본 Global Planner로서 사용되고 있습니다.
- Dijkstra 모드(`use_astar` = false)에서는 어떤 조건에서도 최단 경로를 찾을 수 있도록 Dijkstra의 검색 알고리즘이 보장됩니다.
    
    ![Dijkstra Algorithm](/assets/img/ros2nav2/gif/Dijkstra_algorithm.gif)
    
    Dijkstra Algorithm
    
- A* 모드(`use_astar` = true)에서는 A*의 검색 알고리즘이 최단 경로를 찾는다는 보장은 없지만, 목표를 향해 잠재적 필드를 확장하여 가능한 경로를 더 빨리 찾을 수 있도록 합니다.
    
    ![A* Algorithm](/assets/img/ros2nav2/gif/Aalgorithm.gif)
    A* Algorithm
    

```yaml
planner_server:
  ros__parameters:

		...

    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

- **`tolerance`** (double, 기본값: 0.5)
    - 목표 포즈와 경로 끝 사이의 허용 오차(m)
- **`use_astar`** (bool, 기본값: false)
    - A* 사용 여부 (false: Dijkstra)
- **`allow_unknown`** (bool, 기본값: true)
    - 알 수 없는 공간(map의 회색 영역)에서의 계획을 허용할지 여부