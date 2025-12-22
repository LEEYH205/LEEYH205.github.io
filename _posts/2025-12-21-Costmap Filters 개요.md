---
title: "Costmap Filters 개요"
date: 2025-12-21 12:50:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Costmap Filters]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Costmap Filters란?

• 코스트맵 필터는 일반 코스트맵에 추가되는 추가 레이어(마스크)

• 필터링된 코스트맵을 만들고 그 영역에서 로봇의 동작을 변경하기 위해 필터 플러그인은 필터 마스크에서 가져온 데이터를 읽음

• Nav2에서 제공하는 Costmap Filters 플러그인
- [Keepout Filter](https://docs.nav2.org/configuration/packages/costmap-plugins/keepout_filter.html)
    - 특정 구역을 접근 금지 구역으로 설정하여, 로봇이 이 구역을 피하도록 함
- [Speed Filter](https://docs.nav2.org/configuration/packages/costmap-plugins/speed_filter.html)
    - 로봇의 속도를 조정하기 위해 사용되며, 특정 구역을 지날 때 속도를 줄이거나 높일 수 있음
- [Binary Filter](https://docs.nav2.org/configuration/packages/costmap-plugins/binary_filter.html)
    - 코스트맵의 특정 셀을 이진 값(예: 0 또는 1)으로 설정하는 필터로서, 특정 구역을 명확히 구분하거나 경로 계획에서 특정 영역을 강조하는 데 유용


## Keepout Filter
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-53-43.png)

## Speed Filter
• Speed Filter는 Keepout Filter 유사하지만, 지도의 셀에 해당하는 영역 값이 속도 제한이라는 다른 의미를 갖음

• OccupancyGrid 값은 [0~100] 범위에 속하며, 0 값은 속도 제한이 없음을 의미하며, [1~100] 범위의 값은 다음 공식에 따라 선형적으로 속도 제한 값이 변환
    - speed_limit = filter_mask_data * multiplier + base

- 회색의 비율이 밝을수록 제한 속도가 낮아지고, 회색이 진할수록 제한 속도가 높아짐

![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-21%2012-54-49.png)