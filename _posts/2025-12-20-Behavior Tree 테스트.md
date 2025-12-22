---
title: "Behavior Tree 테스트"
date: 2025-12-20 14:50:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, Behavior Tree]
description: ""
pin: false
math: false
mermaid: false
hidden: true
---
# 테스트 환경
1. `nav2_ws/src` 디렉터리에 simple_bt_example 이라은 이름의 새 패키지를 만든다.
    ```dash
    cd ~/nav2_ws/src
    ros2 pkg create --build-type ament_cmake simple_bt_example
    ```
2. 실습에 필요한 예제
    ```bash
    cd ~/nav2_ws/src/simple_bt_example/src
    touch ex_bt1.cpp ex_bt2.cpp ex_bt3.cpp
    ```

    <details>
    <summary>ex_bt1.cpp</summary>
    <div markdown="1">

    ```C++
    //"header" to BT - our software connection to BT framework
    #include "behaviortree_cpp_v3/bt_factory.h"

    using namespace BT;

    /** Behavior Tree are used to create a logic to decide what
    * to "do" and when.
    * The exercises as it was discussed and depicted in the
    * figure do not include a â€œconnectionâ€ to ROS.
    * The action for the robot, in our case, is defined as a separate C++ class.
    */

    //C++ classes defines robot actions/behaviors

    //----------------Class ApproachBall---------------------------------------
    class ApproachBall : public BT::SyncActionNode // we inherit the SyncActionNode - for synchronous behaviour
    //later we inherit the AsyncActionNode instead to run action in asynchronous mode. Details later
    {
    public:
        ApproachBall(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            //we just print on the terminal
            std::cout << "ApproachBall: " << this->name() << std::endl;
            //the action completes with SUCCESS, you can see below the action can "finish" with FAILURE
            return BT::NodeStatus::SUCCESS;
        }
    };

    //----------------Class FindBall------------------------------------------
    class FindBall : public BT::SyncActionNode
    {
    public:
        FindBall(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "FindBall: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

    //----------------Class PlaceBall------------------------------------------
    class PlaceBall : public BT::SyncActionNode
    {
    public:
        PlaceBall(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "PlaceBall: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

    //----------------Class GripperInterface------------------------------------------
    class GripperInterface
    {
    private:
        bool _opened;

    public:
        GripperInterface() : _opened(true)
        {
        }

        NodeStatus open()
        {
            _opened = true;
            std::cout << "GripperInterface::open" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        NodeStatus close()
        {
            std::cout << "GripperInterface::close" << std::endl;
            _opened = false;
            return BT::NodeStatus::SUCCESS;
        }
    };

    BT::NodeStatus BallClose()
    {
        std::cout << "[ Close to ball: NO ]" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus BallGrasped()
    {
        std::cout << "[ Grasped: NO ]" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    //definition of BT which reflects logical connection between robot actions
    static const char* xml_text = R"(
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <FindBall   name="ball_ok"/>
                    <Sequence>
                        <Fallback>
                            <BallClose   name="no_ball"/>
                            <ApproachBall    name="approach_ball"/>
                        </Fallback>
                        <Fallback>
                            <BallGrasped   name="no_grasp"/>
                            <GraspBall  name="grasp_ball"/>
                        </Fallback>
                    </Sequence>
                <PlaceBall   name="ball_placed"/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";

    int main()
    {
        // We use the BehaviorTreeFactory to register our custom nodes
        BehaviorTreeFactory factory;

        //Node registration process
        factory.registerNodeType<ApproachBall>("ApproachBall");
        factory.registerNodeType<FindBall>("FindBall");
        factory.registerNodeType<PlaceBall>("PlaceBall");

        // Registering a SimpleActionNode using a function pointer.
        // you may also use C++11 lambdas instead of std::bind
        //this method is not recommended so we are not going to use any more
        factory.registerSimpleCondition("BallClose", std::bind(BallClose));
        factory.registerSimpleCondition("BallGrasped", std::bind(BallGrasped));

        //You can also create SimpleActionNodes using methods of a class
        //this method is not recommended so we are not going to use any more
        GripperInterface gripper;
        factory.registerSimpleAction("GraspBall", std::bind(&GripperInterface::close, &gripper));

        // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
        // The currently supported format is XML.
        // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
        auto tree = factory.createTreeFromText(xml_text);

        // To "execute" a Tree you need to "tick" it.
        // The tick is propagated to the children based on the logic of the tree.

        tree.tickRoot();

        return 0;
    }
    ```

    </div>
    </details>

    <details>
    <summary>ex_bt2.cpp</summary>
    <div markdown="1">

    ```c++
    #include "behaviortree_cpp_v3/bt_factory.h"

    using namespace BT;


    class RobotTask1 : public BT::SyncActionNode
    {
    public:
        RobotTask1(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask1: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    };

    class RobotTask2 : public BT::SyncActionNode
    {
    public:
        RobotTask2(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask2: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

    class RobotTask3 : public BT::SyncActionNode
    {
    public:
        RobotTask3(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask3: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

    static const char* xml_text = R"(
    <root main_tree_to_execute = "MainTree" >

        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <RobotTask1   name="task1"/>
                <RobotTask2   name="task2"/>
                <RobotTask3   name="task3"/>
            </Sequence>
        </BehaviorTree>

    </root>
    )";

    // clang-format on

    int main()
    {
        // We use the BehaviorTreeFactory to register our custom nodes
        BehaviorTreeFactory factory;

        factory.registerNodeType<RobotTask1>("RobotTask1");
        factory.registerNodeType<RobotTask2>("RobotTask2");
        factory.registerNodeType<RobotTask3>("RobotTask3");

        // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
        // The currently supported format is XML.
        // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
        auto tree = factory.createTreeFromText(xml_text);

        // To "execute" a Tree you need to "tick" it.
        // The tick is propagated to the children based on the logic of the tree.
        // In this case, the entire sequence is executed, because all the children
        // of the Sequence return SUCCESS.
        tree.tickRoot();

        return 0;
    }
    ```
    </div>
    </details>

    <details>
    <summary>ex_bt3.cpp</summary>
    <div markdown="1">
    
    ```c++
    #include "behaviortree_cpp_v3/bt_factory.h"

    using namespace BT;


    class RobotTask1 : public BT::SyncActionNode
    {
    public:
        RobotTask1(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask1: " << this->name() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    };

    class RobotTask2 : public BT::SyncActionNode
    {
    public:
        RobotTask2(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask2: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

    class RobotTask3 : public BT::SyncActionNode
    {
    public:
        RobotTask3(const std::string& name) : BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        NodeStatus tick() override
        {
            std::cout << "RobotTask3: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };
    static const char* xml_text = R"(

    <root main_tree_to_execute = "MainTree" >

        <BehaviorTree ID="MainTree">
        <Fallback name="root_sequence">
            <RobotTask1   name="task1"/>
            <RobotTask2   name="task2"/>
            <RobotTask3   name="task3"/>
        </Fallback>
        </BehaviorTree>

    </root>
    )";

    int main()
    {
        // We use the BehaviorTreeFactory to register our custom nodes
        BehaviorTreeFactory factory;

        factory.registerNodeType<RobotTask1>("RobotTask1");
        factory.registerNodeType<RobotTask2>("RobotTask2");
        factory.registerNodeType<RobotTask3>("RobotTask3");

        auto tree = factory.createTreeFromText(xml_text);

        tree.tickRoot();

        return 0;
    }

    ```
    </div>
    </details>


3. `CMakeLists.txt`를 아래와 같이 수정합니다.
    
    ```makefile
    cmake_minimum_required(VERSION 3.5)
    project(simple_bt_example)
    
    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()
    
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(behaviortree_cpp_v3 REQUIRED)
    find_package(rclcpp REQUIRED)
    
    include_directories(include)
    
    add_executable(ex_bt1 src/ex_bt1.cpp)
    add_executable(ex_bt2 src/ex_bt2.cpp)
    add_executable(ex_bt3 src/ex_bt3.cpp)
    
    ament_target_dependencies(
      ex_bt1
      behaviortree_cpp_v3
      rclcpp
    )
    
    ament_target_dependencies(
      ex_bt2
      behaviortree_cpp_v3
      rclcpp
    )
    
    ament_target_dependencies(
      ex_bt3
      behaviortree_cpp_v3
      rclcpp
    )
    
    install(TARGETS
      ex_bt1
      ex_bt2
      ex_bt3
      DESTINATION lib/${PROJECT_NAME})
    
    ament_package()
    ```

4. package.xml을 아래와 같이 수정합니다.
    ```xml
    <?xml version="1.0"?>
    <package format="2">
    <name>simple_bt_example</name>
    <version>0.0.0</version>
    <description>ROS2 Behavior Tree Example Package</description>
    <maintainer email="your_email@example.com">Your Name</maintainer>
    <license>Apache-2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>behaviortree_cpp_v3</depend>
    <depend>rclcpp</depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>
    ```



5. 패키지를 빌드합니다.
```DASH
cd ~/nav2_ws
colcon build --symlink-install --packages-select simple_bt_example
```

---
# [예제 1] 특정 시나리오로 보는 BT
```dash
ros2 run simple_bt_example ex_bt1
```
![](/assets/img/Screenshot%20from%202025-12-20%2015-09-19.png)


## 로봇 동작 클래스 정의
```c++
//----------------Class ApproachBall---------------------------------------
class ApproachBall : public BT::SyncActionNode
{
public:
    ApproachBall(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    NodeStatus tick() override
    {
        std::cout << "ApproachBall: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

//----------------Class FindBall------------------------------------------
class FindBall : public BT::SyncActionNode
{
public:
    FindBall(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    NodeStatus tick() override
    {
        std::cout << "FindBall: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

//----------------Class PlaceBall------------------------------------------
class PlaceBall : public BT::SyncActionNode
{
public:
    PlaceBall(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    NodeStatus tick() override
    {
        std::cout << "PlaceBall: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

```
- 위 3개의 클래스들로 로봇의 개별 동작을 정의
    - 각 클래스는 `SyncActionNode`를 상속받아 동기적 행동을 구현
    - `FindBall` 클래스는 로봇이 공을 찾는 동작을 정의
    - `ApproachBall` 클래스는 로봇이 공에 접근하는 동작을 정의
    - `PlaceBall` 클래스는 로봇이 공을 배치하는 동작을 정의
- 본 예제에서는 세 클래스 모두 항상 성공을 반환하도록 함

## 그리퍼 클래스
```c++
//----------------Class GripperInterface------------------------------------------
class GripperInterface
{
private:
    bool _opened;

public:
    GripperInterface() : _opened(true)
    {
    }

    NodeStatus open()
    {
        _opened = true;
        std::cout << "GripperInterface::open" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    NodeStatus close()
    {
        std::cout << "GripperInterface::close" << std::endl;
        _opened = false;
        return BT::NodeStatus::SUCCESS;
    }
};
```
- `GripperInterface` 클래스는 그리퍼 동작을 제어 (열고 닫는 기능을 제공)
- 본 예제에서는 항상 성공을 반환하도록 함

## 조건 함수들
```C++
BT::NodeStatus BallClose()
{
    std::cout << "[ Close to ball: NO ]" << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus BallGrasped()
{
    std::cout << "[ Grasped: NO ]" << std::endl;
    return BT::NodeStatus::FAILURE;
}
```
- 공이 가까이 있는지, 잡혔는지를 확인하는 조건을 내며, 여기서는 항상 실패를 반환

## Behavior Tree XML 정의
```c++
static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <FindBall   name="ball_ok"/>
                <Sequence>
                    <Fallback>
                        <BallClose   name="no_ball"/>
                        <ApproachBall    name="approach_ball"/>
                    </Fallback>
                    <Fallback>
                        <BallGrasped   name="no_grasp"/>
                        <GraspBall  name="grasp_ball"/>
                    </Fallback>
                </Sequence>
            <PlaceBall   name="ball_placed"/>
        </Sequence>
    </BehaviorTree>
</root>
)";
```
![](/assets/img/BTtest.png)
- 로봇 작업의 논리적 연결을 반영하는 Behavior Tree를 정의
- 로봇 동작 순서
    1. **FindBall (공 찾기)**
        - 현재 구현에서는 항상 성공(SUCCESS)을 반환
    2. **BallClose 확인 (공이 가까이 있는지 확인):**
        - 현재 구현에서는 항상 실패(FAILURE)를 반환
    3. **ApproachBall (공에 접근)**
        - BallClose가 실패했기 때문에, 로봇은 공에 접근하는 행동을 수행
        - 성공(SUCCESS)을 반환
    4. **BallGrasped 확인 (공을 잡았는지 확인)**
        - 현재 구현에서는 항상 실패(FAILURE)를 반환
    5. **GraspBall (공 잡기)**
        - BallGrasped가 실패했기 때문에, 로봇은 공을 잡는 행동을 수행
        - 이 행동은 GripperInterface의 close() 메서드를 호출하여 그리퍼를 닫음
        - 현재 구현에서는 항상 성공(SUCCESS)을 반환
    6. **PlaceBall (공 배치)**
        - 이 행동은 항상 성공(SUCCESS)을 반환


## 메인 함수
```C++
int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachBall>("ApproachBall");
    factory.registerNodeType<FindBall>("FindBall");
    factory.registerNodeType<PlaceBall>("PlaceBall");

    factory.registerSimpleCondition("BallClose", std::bind(BallClose));
    factory.registerSimpleCondition("BallGrasped", std::bind(BallGrasped));

    GripperInterface gripper;
    factory.registerSimpleAction("GraspBall", std::bind(&GripperInterface::close, &gripper));
    
    auto tree = factory.createTreeFromText(xml_text);

    tree.tickRoot();

    return 0;
}
```
- Behavior Tree 팩토리를 설정하고, 노드들을 등록한 후, XML에서 트리를 생성하고 실행
- `tree.tickRoot()`를 실행하면 전체 Behavior Tree가 실행되며, 각 노드의 상태에 따라 로봇의 행동이 결정됨


# [예제 2] Sequence (→)

---

- 아래 그림은 위 [예제1](#예제-1-특정-시나리오로-보는-bt)에서 사용했던 BT 블록을 간단하게 나타낸 것.
![](/assets/img/BTtest_2.png)
    - Behavior Tree는 Root 노드로 시작합니다. 노드는 특정 주파수로 틱(ticks)이라고 불리는 신호를 제공하여 자식 노드들의 실행을 허용합니다. 노드는 틱을 받을 때만 실행됩니다. 실행이 진행 중이면 자식 노드는 즉시 부모에게 Running을 반환하고, 목표가 달성되면 Success를, 그렇지 않으면 Failure를 반환합니다.

- **`Sequence(→)`**는 아래 다이어그램과 같이 이루어집니다.
    - `Sequence(→)`이기 때문에 Task1, 2, 3이 모두 성공해야 성공입니다.
    - 두 번째 그림은 Task 2에서 실패했기 때문에 실패입니다.
    ![](/assets/img/BTtest_3.png)
    - 이를 XML로 표현하면 아래와 같습니다.
        ```XML
        <root main_tree_to_execute = "MainTree" >

            <BehaviorTree ID="MainTree">
                <Sequence name="root_sequence">
                    <RobotTask1   name="task1"/>
                    <RobotTask2   name="task2"/>
                    <RobotTask3   name="task3"/>
                </Sequence>
            </BehaviorTree>

        </root>
        ```
    - [핵심!] Sequence 노드는 왼쪽부터 자식 노드들에게 틱을 전달하는 알고리즘을 실행합니다. Failure나 Running을 반환하는 자식 노드를 찾을 때까지 이 과정을 계속하며, 그 결과를 부모 노드에게 반환합니다. 모든 자식 노드가 Success를 반환할 때만 Success를 반환합니다. 자식 노드가 Running이나 Failure를 반환하면 Sequence 노드는 다음 자식 노드(있다면)에게 틱을 전달하지 않습니다. 간단히 말해, Sequence 노드는 논리적 AND 함수로 생각할 수 있습니다.

- 아래 예제에서 왜 task1만 수행되었는지 분석해봅시다.
```DASH
ros2 run simple_bt_example ex_bt2
```
![](/assets/img/Screenshot%20from%202025-12-20%2015-22-06.png)
- 보시다시피, 자식 노드에서 첫 번째 Failure가 발생하면 Sequence가 종료됩니다. Sequence가 성공하려면 모든 노드가 SUCCESS를 반환해야 합니다.
```C++
BT::NodeStatus RobotTask1::tick()
{
    std::cout << "RobotTask1: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTask2::tick()
{
    std::cout << "RobotTask2: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTask3::tick()
{
    std::cout << "RobotTask3: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}
```

# [예제 3] Fallback (?)

---

- **`Fallback(?)`**은 아래 다이어그램과 같이 이루어집니다.
    - `Fallback(?)`이기 때문에 Task1, 2, 3 중 하나만 성공해도 성공입니다.

    ![](/assets/img/BTtest_4.png)

- 아래 그림은 틱과 콜백 흐름
    ![](/assets/img/BTtest_5.png)
        - 이를 XML로 표현하면 아래와 같습니다.
    ```xml
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Fallback name="root_sequence">
                <RobotTask1   name="task1"/>
                <RobotTask2   name="task2"/>
                <RobotTask3   name="task3"/>
            </Fallback>
        </BehaviorTree>
    </root>
    ```
- [핵심!] Fallback 노드는 왼쪽부터 자식 노드들에게 틱을 전달하며, Success 또는 Running을 반환하는 자식 노드를 찾을 때까지 이 과정을 계속합니다. 그리고 나서 그 결과에 따라 Success 또는 Running을 부모 노드에게 전달합니다. 모든 자식 노드가 Failure를 반환할 때만 Failure를 반환합니다. (주의! 자식 노드가 Running이나 Success를 반환하면 Fallback 노드는 다음 자식 노드에게 틱을 전달하지 않음) 간단히 말해, Sequence 노드는 논리적 OR 함수로 생각할 수 있습니다.

```dash
ros2 run simple_bt_example ex_bt3
```
![](/assets/img/Screenshot%20from%202025-12-20%2015-33-28.png)
- 첫 번째 자식 노드에서 FAILURE가 발생하지만, 두 번째 자식 노드가 SUCCESS이므로, RobotTask3은 수행하지 않고 Fallback은 SUCCESS 입니다.
```c++
BT::NodeStatus RobotTask1::tick()
{
    std::cout << "RobotTask1: " << this->name() << std::endl;
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTask2::tick()
{
    std::cout << "RobotTask2: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RobotTask3::tick()
{
    std::cout << "RobotTask3: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}
```


# [예제 4] Reactive Sequence

---

- 일반적인 시퀀스 노드는 자식 노드를 차례로 실행하고, 하나의 노드가 실패하면 시퀀스 전체가 실패로 종료됩니다. 반면에, `ReactiveSequence`는 자식 노드 중 하나라도 실행 상태가 변경되면, 즉시 그 상태를 반영하여 동작을 취합니다. 즉, **자식 노드들의 상태 변화를 지속적으로 모니터링하여 즉각적인 반응**을 합니다
- 이러한 특성으로 인해 `ReactiveSequence`는 비동기 이벤트나 지속적으로 변하는 상태를 다루는 데 매우 유용합니다. 예를 들어, 로봇이 장애물을 피하거나, 다양한 센서 입력을 실시간으로 처리해야 하는 상황에서 효과적으로 사용할 수 있습니다.

## 상황 예시
![](/assets/img/bt-demo1.gif)
장애물에 가로막힌 상황

![](/assets/img/BTtest_6.png)
- Root가 `Reactive Sequence`에 연결되어 있음을 볼 수 있습니다. 그 다음, 첫 번째 `Reactive Sequence`는 다음과 연결됩니다.
    - BlackBoard
    - 두 번째 `Reactive Sequence`
    - 로봇 이동
- BlackBoard는 건너뛰고 두 번째 `Reactive Sequence` 블록(노드)을 고려해봅시다. `Reactive Sequence`는 논리적 AND 게이트처럼 작동합니다. 모든 입력이 TRUE이면 AND 게이트의 출력도 TRUE입니다.
- 로봇은 회전하다가 (TRUE), 그 후 로봇은 레이저를 사용하여 장애물이 없는 경로를 스캔합니다. 아래와 같이 수동으로 장애물을 제거했으므로 TRUE가 됩니다.
    ![](/assets/img/bt-demo2.gif)
- 장애물 제거 후, 두 번째 Reactive Sequence가 TRUE가 되므로 로봇은 MoveRobot을 수행해서 탈출합니다.
    ![](/assets/img/bt-demo3.gif)

# Groot를 통한 BT 시각화

---

## 설치 방법

1. 아래 링크로 접속하고, 맨 아래 Download에서 `Linux installer`를 다운로드합니다.

https://www.behaviortree.dev/groot/

2. 내려받은 경로에서 버전에 맞는 파일명에 대해 명령어 입력 후 installer를 실행합니다.
```dash
chmod +x Groot2-v1.7.0-linux-installer.run
./Groot2-v1.7.0-linux-installer.run
```

3. `~/.zshrc`에 groot2를 alias 등록
```BASH
...

# groot2
alias groot2='cd ~/Groot2/bin && ./groot2'
```

![](/assets/img/Screenshot%20from%202025-12-20%2015-46-16.png)