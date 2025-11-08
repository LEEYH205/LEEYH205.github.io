---
title: "2025-11-08-Node programming(4)-Reentrant Callback Group"
date: 2025-11-08 21:50:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Node Programming, Executor, Callback Group, Reentrant Callback Group]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Reentrant Callback Group

---

- ROS2의 `MutuallyExclusiveCallbackGroup`는 기본 콜백 그룹 유형
- `MutuallyExclusiveCallbackGroup`와 `ReentrantCallbackGroup`의 주요 차이점
    - **`MutuallyExclusiveCallbackGroup`**
        - 이 그룹 내의 모든 콜백은 스레드 수에 관계없이, 해당 그룹에서 하나씩만 실행됩니다.
        - 예를 들어 **하나의 콜백 그룹 내에 콜백이 3개** 있고 **스레드가 3개** 있는 경우, **한 번에 하나의 콜백만 실행**됩니다.
    - **`ReentrantCallbackGroup`**
        - 스레드가 충분하다면 이 그룹 내의 모든 콜백을 병렬로 실행할 수 있습니다.
        - 예를 들어, 동일한 **ReentrantCallbackGroup 내에 3개의 콜백을 추가하고 2개의 스레드**가 있는 경우, **3개의 콜백** 중 **2개의 콜백을 동시에 실행**할 수 있습니다.


## **Callback Group 비교를 위한** 실습

- 실습 스크립트 설명
    - 작성할 스크립트에는 **`하나의 서비스 서버`**와 **`하나의 타이머 콜백`**이 있습니다.
    - 인수를 통해 시간, 스레드 수, 콜백 유형 등 일부 요소를 변경할 수 있습니다.


callback_groups_example.py
```python
import rclpy
from rclpy.node import Node
import time

from std_srvs.srv import SetBool

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import argparse


class DummyServer(Node):

    def __init__(self, args, callback_group_type="reentrant"):

        self.timer_flag = True

        super().__init__('service_start_turn')

        # More info here: https://docs.python.org/3/library/argparse.html
        parser = argparse.ArgumentParser(
            description='Dummy Server to Learn about Callback Groups and Threads')

        # Add an argument for setting the service waiting time
        parser.add_argument('-service_wait_time',
                            type=float,
                            help='Time the service will be waiting',
                            required=True)

        # Add an argument for setting por of the timer callback
        parser.add_argument('-timer_period',
                            type=float,
                            nargs=1,
                            metavar='TIMEOUT',
                            default=1.0,
                            help="Time period of the Callback for the timer")

        # Add an argument for setting the callback group type
        parser.add_argument('-callback_group_type',
                            type=str,
                            default="reentrant",
                            help="Type of Callback Groups REENTRANT of EXCLUSIVE")

        # Add an argument for setting the number of threads
        parser.add_argument('-threads',
                            type=int,
                            default=1,
                            help="Number of threads to use in the executor")

        self.args = parser.parse_args(args[1:])

        parser.print_help()

        # <rclpy.callback_groups.MutuallyExclusiveCallbackGroup object at 0x7ff58fc9e8e0>
        # By default, the Callbacks are mutually exclusive. This means that in each group, only
        # one Callback can be done: https://docs.ros2.org/foxy/api/rclpy/api/node.html
        print("## DEFAULT Node Callback Group=" + str(self.default_callback_group))

        self.get_logger().warning("Setting "+self.args.callback_group_type+" Groups")
        if self.args.callback_group_type == "reentrant":
            # If you set the group reentrant, any Callback inside will be executed in parallel
            # If there are enough threads
            self.group1 = ReentrantCallbackGroup()
            self.get_logger().warning("ReentrantCallbackGroup Set")
            # Both the service and the timer use the same callback group
            self.srv = self.create_service(
                SetBool, '/dummy_server_srv', self.setbool_callback, callback_group=self.group1)
            self.timer = self.create_timer(
                self.args.timer_period[0], self.timer_callback, callback_group=self.group1)

        elif self.args.callback_group_type == "exclusive":
            self.group1 = MutuallyExclusiveCallbackGroup()
            self.group2 = MutuallyExclusiveCallbackGroup()
            self.get_logger().warning("MutuallyExclusiveCallbackGroup Set")
            # Set one group for the service and another one for the timer
            self.srv = self.create_service(
                SetBool, '/dummy_server_srv', self.setbool_callback, callback_group=self.group1)
            self.timer = self.create_timer(
                self.args.timer_period[0], self.timer_callback, callback_group=self.group2)

        else:
            # You do not set groups. Therefore, they will get the default group for the Node
            self.get_logger().error("NO GROUPS SET Set")
            self.srv = self.create_service(
                SetBool, '/dummy_server_srv', self.setbool_callback)
            self.timer = self.create_timer(
                self.args.timer_period[0], self.timer_callback)

    def get_threads(self):
        return self.args.threads

    def setbool_callback(self, request, response):
        self.get_logger().warning("Processing Server Message...")
        self.wait_for_sec(self.args.service_wait_time)
        self.get_logger().warning("Processing Server Message...DONE")
        response.message = 'TURNING'
        # return the response parameters
        return response

    def wait_for_sec(self, wait_sec, delta=1.0):
        i = 0
        while i < wait_sec:
            self.get_logger().info("..."+str(i)+"[WAITING...]")
            time.sleep(delta)
            i += delta

    def timer_callback(self):
        self.print_dummy_msgs()

    def print_dummy_msgs(self):
        if self.timer_flag:
            self.get_logger().info("TICK")
            self.timer_flag = False
        else:
            self.get_logger().info("TACK")
            self.timer_flag = True


def main(args=None):
    # To Use: ros2 service call /dummy_server_srv std_srvs/srv/SetBool data:\ false\
    # ros2 run unit5_pkg callback_groups_examples -service_wait_time 5.0 -timer_period 1.0
    # initialize the ROS communication
    rclpy.init(args=args)
    print("args==="+str(args))
    # Format the arguments given through ROS to use the arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    print("clean ROS args==="+str(args_without_ros))
    start_stop_service_node = DummyServer(args_without_ros)

    num_threads = start_stop_service_node.get_threads()
    start_stop_service_node.get_logger().info(
        'DummyServer Started with threads='+str(num_threads))

    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(start_stop_service_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        start_stop_service_node.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

setup.py
```python
entry_points={
    'console_scripts': [
        'problem_node = node_exercise.problem:main',
        'solution1_node = node_exercise.solution1:main',
        'solution2_node = node_exercise.solution2:main',
        'callback_groups_example = node_exercise.callback_groups_example:main',
    ],
},
```


## [실험 1] 2개의 Threads & 2개의 Mutually Exclusive Callback Group

```bash
ros2 run node_exercise callback_groups_example -service_wait_time 5.0 -timer_period 1.0 -callback_group_type exclusive -threads 2
```
```bash
ros2 service call /dummy_server_srv std_srvs/srv/SetBool data:\ false
```
![](/assets/img/Peek%202025-11-08%2022-02.gif)

- 보시다시피 두 콜백을 동시에 실행합니다. 그 이유는 다음과 같습니다.
    - 두 개의 스레드 설정
    - `Mutually Exclusive Callback Group` 유형을 설정하면 각 콜백이 다른 그룹에 속하게 됩니다.

## [실험 2] 2개의 Threads & 1개의 Reentrant Callback Group
```bash
ros2 run node_exercise callback_groups_example -service_wait_time 5.0 -timer_period 1.0 -callback_group_type reentrant -threads 2
```
```bash
ros2 service call /dummy_server_srv std_srvs/srv/SetBool data:\ false
```
![](/assets/img/Peek%202025-11-08%2022-06.gif)
- 보시다시피 두 콜백을 동시에 실행합니다. 그 이유는 다음과 같습니다.
    - 두 개의 스레드 설정
    - `Reentrant Callback Group` 유형을 설정하면 두 콜백이 모두 같은 콜백 그룹에 속하고, 충분한 스레드가 있다면 병렬 콜백 함수를 실행할 수 있습니다.

## [실험 3] 1개의 Threads & 1개의 Reentrant Callback Group
```bash
ros2 run node_exercise callback_groups_example -service_wait_time 5.0 -timer_period 1.0 -callback_group_type reentrant -threads 1
```
```bash
ros2 service call /dummy_server_srv std_srvs/srv/SetBool data:\ false
```
![](/assets/img/Peek%202025-11-08%2022-11.gif)
- 보시다시피 이제 **한 번에 하나의 콜백만** 실행합니다. 그 이유는 다음과 같습니다.
    - **하나의** 스레드만 설정했습니다.
    - `Reentrant Callback Group` 유형을 설정하면 두 콜백이 모두 같은 콜백 그룹에 속하고, 충분한 스레드가 있다면 병렬 콜백 함수를 실행할 수 있습니다.
    - 그러나 **스레드가 충분하지** 않기 때문에 `Reentrant Callback Group` 을 사용하더라도 다른 콜백은 콜백이 끝날 때까지  기다려야 합니다.


# Callback Group 사용 전략은?
- `그 콜백 그룹 내에서 메인 알고리즘이 돌아간다`면, 다른 그룹으로 따로 묶어 영향이 가지 않게 한다.
- 주기가 많이 잡아먹는 콜백이 있으면 따로 묶어 준다.
- 콜백 그룹 하나 당 하나의 스레드가 있어야 하므로, 자원량을 고려해야 한다.
- `htop`으로 논리 cpu 개수 (스레드 개수)를 파악할 수 있다.
- main, slam, nav2, deep learing, network 등 돌렸을 때, 스레드가 몇 개 남는 지 확인해야 한다.
- `단순하게 최신 값을 업데이트하는 용도다 (단순하게 토픽받아오는 콜백)`면 하나의 콜백 그룹으로 묶어도 된다. (받은 데이터를 가지고 노는 건 main 등에서.)