---
title: "Service(4)-Interface"
date: 2025-11-08 15:35:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Service, Interface]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

서비스를 직접 만들어서 필요한 서비스의 Request, Response를 직접 작성하는 경우가 엄청 많다.

- 목표
    - 로봇의 선속도와 각속도, 그리고 이동 시간을 지정하여 로봇을 제어하기 위한 custom service인 `MoveRobot.srv` 생성
    - 터미널에서 서비스를 호출하여 서비스 서버가 잘 구현되었는지 결과를 확인


MoveRobot.srv
```
float64 linear_velocity  # 선속도 (m/s)
float64 angular_velocity # 각속도 (rad/s)
float64 duration         # 이동 시간 (초)
---
bool success
string message
```

![](/assets/img/Screenshot%20from%202025-11-08%2015-23-26.png)

---


```python

class MovementServer(Node):
    def __init__(self):
        super().__init__('movement_server')
        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        ...

    def move_robot_callback(self, request, response):
        twist = Twist()
        twist.linear.x = request.linear_velocity
        twist.angular.z = request.angular_velocity

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < request.duration:
            self.velocity_publisher.publish(twist)
            time.sleep(0.1)
        # 로봇 정지
        stop_twist = Twist()
        self.velocity_publisher.publish(stop_twist)
        response.success = True

        if request.linear_velocity > 0.0 and request.angular_velocity == 0.0:
            response.message = 'Robot moved forward'
        elif request.linear_velocity < 0.0 and request.angular_velocity == 0.0:
            response.message = 'Robot moved backward'
        elif request.linear_velocity == 0.0 and request.angular_velocity > 0.0:
            response.message = 'Robot turned left'
        elif request.linear_velocity == 0.0 and request.angular_velocity < 0.0:
            response.message = 'Robot turned right'
        else:
            response.message = 'Robot moved in a curve'

        return response

```

```python
#setup.py 추가
entry_points={
    'console_scripts': [
        'robot_status_publisher = custom_interface_example.robot_status_publisher:main',
        'movement_server = custom_interface_example.movement_server:main', # 추가!
    ],
},
```

![](/assets/img/Peek_2025-11-08_15-34.gif)