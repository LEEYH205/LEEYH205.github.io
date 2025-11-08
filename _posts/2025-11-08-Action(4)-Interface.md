---
title: "Action(4)-Interface"
date: 2025-11-08 17:30:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Action, Interface]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Custom Action 만들기


- 목표
    - 지난 실습에 사용했던 액션 인터페이스인 `turtlesim/action/RotateAbsolute`를 거의 그대로 활용하여 커스텀 액션 작성
    - 액션 타입은 그대로 `RotateAbsolute`라는 이름을 활용
    - 기존 포맷에서 성공 여부를 나타내는 `success`만 result에 추가


RotateAbsolute.action
```
float32 theta
---
float32 delta
bool success # 추가
---
float32 remaining
```

CMakeLists.txt
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  "action/RotateAbsolute.action" # 추가!
)
```

![](/assets/img/Screenshot%20from%202025-11-08%2017-39-43.png)


- `tutorial_action` 패키지의 `rotate_server.py`에서 두 가지 부분을 수정

    - 기존 코드의 `turtlesim/action/RotateAbsolute` 인터페이스 대신 새로 만든 `custom_interfaces/action/RotateAbsolute`로 교체
        ```python
        # from turtlesim.action import RotateAbsolute 대신
        from custom_interfaces.action import RotateAbsolute
        ```
    
    - Result에서 새롭게 추가한 success 처리

        ```python
        # 목표 도달 및 응답
        goal_handle.succeed()
        result = RotateAbsolute.Result()
        result.delta = angle_diff
        result.success = True # 추가
        ```

​
- 지난 실습에서 빌드 시 --symlink-install 옵션으로 빌드했기 때문에, 추가적인 빌드 없이 아래 노드를 실행시킵니다.
    ```bash
    ros2 run tutorial_action rotate_server
    ```

​- 아래 명령어로 액션을 호출하여 결과를 확인합니다.
```bash
ros2 action send_goal -f /rotate_tiago custom_interfaces/action/RotateAbsolute "{theta: 1.5708}"
```

![](/assets/img/Peek%202025-11-08%2017-45.gif)