---
title: "Global Localization 전략"
date: 2025-12-17 09:00:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, SLAM, Global Localization]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---

# Global Localization

---

- `Global Localization`은 사람이나 로봇 모두 로봇이 지도에서 어디에 있는지 모를 때 사용됩니다. 그러면 로봇이 스스로 현지화를 시도해야 합니다. `Global Localization`을 사용하면 로봇이 지도에서 자신의 위치를 파악하려고 시도할 수 있습니다.
- AMCL에서는 `/reinitialize_global_localization`이라는 서비스를 제공하며 필터의 모든 입자를 지도 전체에 배포합니다. 이는 아래와 같은 경우에 사용하면 좋습니다.
    1. **갑작스럽게 위치 추정이 크게 불일치할 때**
        
        로봇의 위치 추정이 실제 위치와 크게 다를 때, 이를 "**Kidnapped robot problem**"이라고 하며, 로봇이 예상치 못한 위치로 이동되었을 때 발생할 수 있습니다. 이 경우, AMCL의 기존 파티클들이 실제 위치를 반영하지 못하고 잘못된 추정을 계속할 수 있습니다. **`reinitialize_global_localization` 서비스를** 통해 파티클들을 전체 지도에 걸쳐 재분배함으로써, 로봇이 자신의 정확한 위치를 다시 찾을 수 있도록 할 수 있습니다.
        
    2. **초기 시작 또는 위치가 크게 변경된 후**
        
        로봇이 새로운 환경에 배치되거나 기존 환경에서의 초기 시작 시, AMCL은 초기 위치에 대한 정보 없이 로봇의 위치를 추정해야 할 수 있습니다. 이 때 전체 지도 상의 임의의 위치에서 파티클들을 뿌려 보다 정확한 위치 추정을 위한 기회를 제공합니다.
        
    3. **장시간의 동작 후 위치 불확실성 증가**
        
        로봇이 장시간 동안 동작하면서 센서의 누적 오류나 환경의 변화 등으로 인해 위치 불확실성이 점점 증가할 수 있습니다. 이런 경우, 파티클 필터의 효율성이 저하될 수 있는데, 이 때 사용하면 위치 불확실성을 초기화하고 새로운 정확한 위치 추정을 시작할 수 있습니다.


```bash
# terminal 1
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py

# terminal 2
ros2 launch neuronbot2_nav localization_launch.py use_sim_time:=true

# terminal 3
cd ~/nav2_ws/src/neuronbot2/neuronbot2_nav/rviz
rviz2 -d nav2_default_view.rviz
```


## [방법 1] 터미널에서 Global Localization 수행하기
```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

![](/assets/img/Peek%202025-12-17%2010-16.gif)
![](/assets/img/Peek%202025-12-17%2010-18.gif)
Global Localization 수행 후 초기 위치 추정에 실패한 것을 확인할 수 있다.

map 전체에 파티클을 뿌리는 건데, 부족하다.

global localization을 할땐, 파티클 개수를 확 늘려줬다가,
끝나면, 파티클 개수를 원래대로 줄여줄거임.

왜냐면, 평상시에도 몇 만개를 쓰면, AMCL 작동하는 데에 컴퓨팅 파워를 다 써서
나머지 로직들이 제대로 안될 수 있음.

## [방법 2] 프로그램 상에서 Global Localization 수행하기

1. localizer_server 패키지의 localizer_server 디렉터리에 global_localization.py를 만들고 아래 코드를 작성합니다.
```PYTHON
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from time import sleep

class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('global_localization_node')
        # create client
        self.relocalization_client = self.create_client(Empty, 'reinitialize_global_localization')
        self.param_client = self.create_client(SetParameters, '/amcl/set_parameters')

        # wait for service
        while not self.relocalization_client.wait_for_service(timeout_sec=1.0) \
                and not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def set_amcl_params(self, max_particles, min_particles):
        params = [
            Parameter(name='max_particles', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=max_particles)),
            Parameter(name='min_particles', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=min_particles))
        ]
        req = SetParameters.Request()

        for param in params:
            req.parameters.append(param)

        future = self.param_client.call_async(req)
        future.add_done_callback(self.set_amcl_params_callback)

        # print current max_particles and min_particles
        self.get_logger().info('max_particles: %d' % max_particles)
        self.get_logger().info('min_particles: %d' % min_particles)
        return future.result()

    def call_reinitialize_global_localization(self):
        req = Empty.Request()
        return self.relocalization_client.call_async(req)

    def set_amcl_params_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
        else:
            self.get_logger().info('Service call succeeded %r' % (response,))

def main(args=None):
    rclpy.init(args=args)
    global_localization = GlobalLocalization()

    # adjust parameters for global localization
    global_localization.set_amcl_params(20000, 1000)
    rclpy.spin_once(global_localization)

    # call service reinitialize_global_localization
    future = global_localization.call_reinitialize_global_localization()
    rclpy.spin_until_future_complete(global_localization, future)
    if future.result() is not None:
        global_localization.get_logger().info('Global localization reinitialized successfully.')
    else:
        global_localization.get_logger().error('Failed to call service reinitialize_global_localization')

    # restore default parameters
    sleep(7)
    global_localization.set_amcl_params(2000, 500)

    rclpy.spin(global_localization)
    global_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

- `Global Localization`을 수행하기 위해서는 지도 상에 많은 파티클을 뿌려야 유리하기 때문에 동적으로 두 개의 파리미터를 수정해주었습니다.
2. `setup.py`를 수정하여 `global_localization.py` 스크립트의 실행 파일을 **entry_points**에 추가합니다.
```PYTHON
entry_points={
    'console_scripts': [
        'init_pose = localizer_server.init_robot:main',
        'global_localization = localizer_server.global_localization:main'
    ],
},
```

![](/assets/img/Peek%202025-12-17%2011-10.gif)
max_particles : 40000
mim_particles : 1000
으로 하고,
로봇을 제자리 회전으로 움직이게 하여,
reinitialize global localization이 제대로 되는 지 확인해보았으나,
map_frame (0,0,0)으로 가는 버그를 확인했다.


## 원인 분석
set_amcl_params(2000,500) 호출 순간 AMCL이 PF(Particle Filter)를 재구성/리셋하면서 파티클이 초기 포즈 기준으로 다시 뿌려지고
그 초기 포즈가 (map 기준) 0,0,0으로 잡혀있어서 로봇의 “지도상 위치 추정”이 원점 근처로 순간 점프

파라미터 변경이 AMCL 내부 상태를 리셋시키고, 초기 포즈가 0,0이라서 파티클이 거기 생기는 것.




## 수정한 코드
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from time import sleep

class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('global_localization_node')

        self.relocalization_client = self.create_client(Empty, 'reinitialize_global_localization')
        self.param_client = self.create_client(SetParameters, '/amcl/set_parameters')

        # 둘 다 준비될 때까지 대기
        while (not self.relocalization_client.wait_for_service(timeout_sec=1.0)
               or not self.param_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('service not available, waiting again...')

        self.last_amcl_pose: PoseWithCovarianceStamped | None = None

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10
        )
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_pose = msg

    def set_amcl_params(self, max_particles: int, min_particles: int):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='max_particles',
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=max_particles)
            ),
            Parameter(
                name='min_particles',
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=min_particles)
            ),
        ]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()

        self.get_logger().info(f"max_particles: {max_particles}")
        self.get_logger().info(f"min_particles: {min_particles}")
        self.get_logger().info(f"set_parameters resp: {resp}")
        return resp

    def call_reinitialize_global_localization(self):
        return self.relocalization_client.call_async(Empty.Request())

    def _build_initialpose_msg(self) -> PoseWithCovarianceStamped | None:
        if self.last_amcl_pose is None:
            return None
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose = self.last_amcl_pose.pose  # pose+covariance 복사
        return m

    def republish_initialpose_n(self, n: int = 3, interval_sec: float = 0.5):
        """
        restore 직후 흔들림을 잡기 위해 /initialpose를 n번 재주입.
        interval_sec 동안 spin_once로 콜백도 계속 처리.
        """
        if self.last_amcl_pose is None:
            self.get_logger().warn("No /amcl_pose received yet -> cannot republish /initialpose")
            return False

        for i in range(n):
            msg = self._build_initialpose_msg()
            if msg is None:
                self.get_logger().warn("Lost /amcl_pose while republishing")
                return False

            self.initpose_pub.publish(msg)
            self.get_logger().info(f"Republished /initialpose ({i+1}/{n})")

            # interval 동안 콜백 처리하며 대기 (sleep만 하면 콜백이 안 돌 수 있어서 spin_once)
            end_t = self.get_clock().now().nanoseconds + int(interval_sec * 1e9)
            while self.get_clock().now().nanoseconds < end_t:
                rclpy.spin_once(self, timeout_sec=0.05)

        return True


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalization()

    # 1) global localization 전: 파티클 늘리기
    node.set_amcl_params(20000, 1000)

    # 2) /amcl_pose 최소 몇 번 받아두기
    node.get_logger().info("Warming up: waiting for /amcl_pose ...")
    for _ in range(40):  # ~2초 (0.05s * 40)
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_amcl_pose is not None:
            break

    # 3) global localization 수행
    fut = node.call_reinitialize_global_localization()
    rclpy.spin_until_future_complete(node, fut)
    if fut.result() is not None:
        node.get_logger().info("Global localization reinitialized successfully.")
    else:
        node.get_logger().error("Failed to call service reinitialize_global_localization")

    # 4) 수렴 대기
    sleep(7)

    # 5) restore + 초기포즈 재주입(3회)
    node.set_amcl_params(2000, 500)
    node.republish_initialpose_n(n=3, interval_sec=0.5)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
![](/assets/img/Peek%202025-12-17%2012-27.gif)

republish_initialpose()는 restore 직후 한 번만 말고, AMCL이 흔들릴 수 있으니 0.5초 간격으로 2~3번 쏴줌.

(AMCL이 파라미터 변경으로 PF 재구성하면서 잠깐 흔들릴 때, 한 번만 쏘면 다시 튈 수 있어서 이 방식이 더 안정적)