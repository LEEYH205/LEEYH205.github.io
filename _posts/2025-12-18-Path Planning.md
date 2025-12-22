---
title: "Path Planning 개요"
date: 2025-12-18 09:50:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Path Planning]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Path Planning

• 로봇이 출발지에서 목적지까지 이동하는 최적 또는 효율적인 경로를 계산하는 과정
• 이 과정에서 로봇은 주어진 환경의 지도를 기반으로 하여 장애물을 피하고, 가장 적합한 경로를 찾아서 목적지에 도달하도록 설계됨

# Nav2 기반의 Path Planning
## Nav2 기반의 Path Planning을 위해 필요한 노드
• Path Planning을 위해 먼저 작동되고 있어야 하는 노드

    • map_server
    • amcl

• Path Planning을 위한 노드

    • controller_server
    • smoother_server
    • planner_server
    • behavior_server
    • bt_navigator
    • waypoint_follower
    • lifecycle_manager

## Nav2 기반의 Path Planning을 위한 설정 파일
• planner_server
- 로봇이 출발 지점에서 목표 지점까지 이동할 경로를 계획하는 서버

• controller_server
- 로봇이 목표 지점으로 이동할 때 경로를 따라가도록 제어하는 서버

• bt_navigator
- 행동 트리(Behavior Tree)를 사용하여 로봇의 탐색 및 네비게이션을 관리

• behavior_server
- 로봇의 동작과 행동을 관리하고 조정하는 서버

• waypoint_follower
- 로봇이 여러 개의 경유 지점을 순차적으로 따라갈 수 있도록 함

• global_costmap
- 로봇의 전체 환경에 대한 전역적인 costmap(비용 지도)를 생성하여 경로 계획에 사용

• local_costmap
- 로봇 주변의 가까운 환경에 대한 지역적인 costmap을 생성하여 충돌 회피와 로컬 경로 추종에 사용