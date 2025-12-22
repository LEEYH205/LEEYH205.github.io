---
title: "Costmap (Global Costmap, Local Costmap, Layer)"
date: 2025-12-20 12:10:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Costmap, Global Costmap, Local Costmap, Layer]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

```BASH
# terminal 1
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py

# terminal 2
ros2 launch neuronbot2_nav bringup_launch.py use_sim_time:=true
```

![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-15.gif)

추가된 장애물에 대해 Global & Local Costmap 모두 감지되는 것을 확인할 수 있다.
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-21.gif)

---
# Global Costmap
`Obstacle layer`, `Inflation layer`, `Static layer`

rqt로 Global Costmap에서 obstacle layer의 영향을 확인할 수 있다.
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-26.gif)

obstacle layer enabled를 끄고, 경로를 생성하면,
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-29.gif)
장애물을 무시하고, 뚫어버리게 경로를 생성한다.
local planner도 경로를 따라가고 싶은데, 
local planner 입장에서는 local costmap을 고려하는데, local costmap에서는 막혀있다.
local costmap에서는 막혀있는데, global costmap에서는 안막혀있다보니, 갈팡질팡하는 것을 볼 수 있다.
그래서, global costmap도 obstacle layer를 반영한 것임을 확인할 수 있다.

obstacle layer enabled를 다시 키면,
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-30.gif)


-> Global Costmap도 Obstacle layer, Inflation layer, Static layer를 모두 고려해야한다.

---
# Local Costmap
![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2012-38.gif)
`Inflation Layer`, `Voxel Layer`

Voxel Layer (3D) 가 Obstacle Layer (2D) 역할도 대신함.