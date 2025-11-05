---
title: "Service(2)-Client"
date: 2025-11-05 22:20:00 +0900
categories: [ROBOTICS]
tags: [Robotics, Service, Client]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Service  - Client

- 목표
    - 앞서 실행했던 `spin` 또는 `circle` 서비스를 호출하는 **서비스 클라이언트** 노드 생성
    - 비동기 호출 방식(`call_async`)을 사용하여 서비스를 호출
    - 서비스의 사용 가능 유무 확인

## 비동기로 진행


```python
class EmptyServiceClient(Node):
    ...
    def __init__(self):
        super().__init__('empty_service_client')
        self.client = self.create_client(Empty, 'spin')  # 또는 'circle'        
        ...

    def send_request(self):
        self.future = self.client.call_async(self.req)


```

```python
def main(args=None):
    rclpy.init(args=args)
    empty_client = EmptyServiceClient()
    empty_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(empty_client, timeout_sec=0.1)
        if empty_client.future.done():
            try:
                response = empty_client.future.result()
            except Exception as e:
                empty_client.get_logger().info('service call failed %r' % (e,))
            else:
                empty_client.get_logger().info('service call completed')
            break

        # 서비스 서버의 response를 기다리면서 추가적인 동작 수행 가능
        empty_client.get_logger().info('additional work...!')

    empty_client.destroy_node()
    rclpy.shutdown()
    ...
```


우리가 `call_async` 비동기를 했기 때문에, `future.done()`이지 않으면, 즉, response를 기다리면서, 추가적인 동작 수행 가능

```python
try:
    response = empty_client.future.result()
except Exception as e:
    empty_client.get_logger().info('service call failed %r' % (e,))
else:
    empty_client.get_logger().info('service call completed')
```

`response`은 현재 response가 empty인 service이기때문에 해당 데이터를 사용하지 않는데, 추후에 response가 있는 서비스를 만들어서, 이용할 땐, 사용하면 됨.



setup.py에 추가
```python
entry_points={
        'console_scripts': [
            'empty_service_client = tutorial_service.empty_service_client:main',

        ],
    },
```
&nbsp;

![](/assets/img/Peek_2025-11-05_23-40.gif)
&nbsp;

---
## **`rclpy.spin` VS** **`rclpy.spin_once`**

### **rclpy.spin(node)**

- 이 함수는 지정된 노드에 대해 무한 루프를 실행하여 콜백을 계속해서 호출합니다.
- 프로그램이 종료될 때까지 계속 실행되며, 모든 콜백, 타이머, 서비스 등이 처리될 수 있도록 합니다.
- 노드가 계속해서 실행되어야 할 때 주로 사용됩니다.
```python
import rclpy
from std_msgs.msg import String

def callback(msg):
    print('Received message: ', msg.data)

rclpy.init()
node = rclpy.create_node('listener')
subscription = node.create_subscription(String, 'chatter', callback, 10)
rclpy.spin(node)  # 이 부분에서 무한 루프가 실행되며 콜백이 계속 호출됩니다.
```

### **rclpy.spin_once(node, timeout_sec=None)**

- 이 함수는 지정된 노드에 사용 가능한 콜백을 호출합니다.
- 주로 **`while`** 루프 안에서 호출됩니다. 이렇게 하면 프로그램이 계속 실행되면서 필요에 따라 콜백을 실행하고 다른 작업을 수행할 수 있습니다.
- 노드를 더 세밀하게 제어할 필요가 있을 때 유용합니다.

```python
import rclpy
from std_msgs.msg import String

def callback(msg):
    print('Received message: ', msg.data)

rclpy.init()
node = rclpy.create_node('listener')
subscription = node.create_subscription(String, 'chatter', callback, 10)

while rclpy.ok():
    rclpy.spin_once(node)  # 한 번의 반복 실행
    # 필요하다면 여기에서 다른 작업을 수행할 수 있습니다.
```

&nbsp;

&nbsp;

---

## sync vs async (동기식 vs 비동기식 서비스 클라이언트)

- 서비스를 호출(요청 보내기)하면 서버가 응답 메시지를 반환할 때까지 기다리는 것 (sync)을 보셨을 것입니다. 그러나 응답이 완료될 때까지 기다리는 동안 노드에서 다른 작업(예: 로그에 메시지 보내기)을 계속 실행하는 방법(async)도 있습니다.
- 이 두가지 방법이 **비동기(async)** 호출과 **동기(sync)** 호출입니다.


### 예제: ROS2 서비스 클라이언트(동기식 vs 비동기식)

#### 1. **비동기(async) 방식 서비스 클라이언트 예제**
서비스를 호출한 뒤, 응답이 올 때까지 기다리지 않고, 그 동안 다른 작업도 병행할 수 있다.

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class EmptyServiceClientAsync(Node):
    def __init__(self):
        super().__init__('empty_service_client_async')
        self.client = self.create_client(Empty, 'spin')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기중...')
        self.req = Empty.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    client = EmptyServiceClientAsync()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client, timeout_sec=0.1)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(f'서비스 호출 실패: {e}')
            else:
                client.get_logger().info('서비스 비동기 호출 완료')
            break

        client.get_logger().info('응답 기다리면서 다른 일 수행중...')

    client.destroy_node()
    rclpy.shutdown()
```

---

#### 2. **동기(sync) 방식 서비스 클라이언트 예제**
서비스를 호출하면 응답이 올 때까지 **그 자리에서 블로킹**하며 다른 작업을 하지 못한다.

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class EmptyServiceClientSync(Node):
    def __init__(self):
        super().__init__('empty_service_client_sync')
        self.client = self.create_client(Empty, 'spin')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기중...')
        self.req = Empty.Request()

    def call_service(self):
        # call()은 동기 방식이므로, 응답이 올 때까지 기다린다!
        response = self.client.call(self.req)
        return response

def main(args=None):
    rclpy.init(args=args)
    client = EmptyServiceClientSync()
    client.get_logger().info('서비스(동기) 호출...')
    response = client.call_service()
    client.get_logger().info('서비스 동기 호출 완료')
    client.destroy_node()
    rclpy.shutdown()
```

- **비동기(async)** 예제에서는 `call_async`와 `future.done()`을 확인하면서, 서비스 응답을 기다리는 동안에도 다른 작업(예: 로그 출력)을 할 수 있다.
- **동기(sync)** 예제에서는 `call()`을 사용해 응답이 올 때까지 **그 자리에서 멈춘다**.

> 동기 방식은 특별히 이유가 있지 않으면 ROS2에서 권장하지는 않는다. 대부분 상황에서는 비동기 방식을 사용하자!

&nbsp;

&nbsp;

&nbsp;

---
ROS2에서는 작업하는 서비스는 기본적으로 비동기식.
그렇다고 해서 동기식 서비스를 사용할 수 없다는 의미는 아니지만 권장하지는 않는다.

https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html