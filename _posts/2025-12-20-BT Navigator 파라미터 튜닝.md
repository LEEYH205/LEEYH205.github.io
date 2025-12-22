---
title: "BT Navigator 파라미터 튜닝"
date: 2025-12-20 11:20:00 +0900
categories: [ROBOTICS, ROS2Nav2]
tags: [Robotics, ROS2, Nav2, BT Navigator, Parameter, tuning]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# BT Navigator 파라미터 튜닝

---

- 로봇의 네비게이션 행동을 정의하고 관리하기 위해 사용되는 **`Behavior Tree(BT)`**를 실행하는 컴포넌트
    - **Behavior Tree**: 로봇의 행동을 트리 형태로 구조화하여, 로봇이 수행해야 할 작업을 유연하고 모듈화된 방식으로 표현
    - 트리는 여러 노드로 구성되며, 각 노드는 로봇의 특정 행동이나 조건을 나타냄
    (Action Node, Condition Node, Control Node 등)
- `bt_navigator`의 동작은 해당 동작이 포함된 XML 파일에 의해 정의
    
    ```xml
    <!--
      This Behavior Tree replans the global path periodically at 1 Hz, and has
      recovery actions. Obtained from the official Nav2 package
    -->
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="6" name="NavigateRecovery">
          <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
              <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
              </RecoveryNode>
            </RateController>
            <RecoveryNode number_of_retries="1" name="FollowPath">
              <FollowPath path="{path}" controller_id="FollowPath"/>
              <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
            </RecoveryNode>
          </PipelineSequence>
          <SequenceStar name="RecoveryActions">
            <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5"/>
          </SequenceStar>
        </RecoveryNode>
      </BehaviorTree>
    </root>
    ```

## BT Navigator **Parameters**

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml
    default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_through_poses_bt.xml
    always_reload_bt_xml: false
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator" # In Iron and older versions, "/" was used instead of "::"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator" # In Iron and older versions, "/" was used instead of "::"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
      # - smoother_error_code, navigate_to_pose_error_code, navigate_through_poses_error_code, etc
```

- `default_nav_to_pose_bt_xml`, `default_nav_through_poses_bt_xml` 파라미터에서 경로가 제대로 성정되지 않았다면 기본 xml 파일이 자동으로 설정됩니다.
![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2011-38-17.png)

- `default_nav_to_pose_bt_xml` 파라미터 처리
    
    ```cpp
    ...
    
    std::string
    NavigateToPoseNavigator::getDefaultBTFilepath(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
      std::string default_bt_xml_filename;
      auto node = parent_node.lock();
    
      if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
        std::string pkg_share_dir =
          ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
        node->declare_parameter<std::string>(
          "default_nav_to_pose_bt_xml",
          pkg_share_dir +
          **"/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");**
      }
    
      node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
    
      return default_bt_xml_filename;
    }
    
    ...
    ```
    nav2_bt_navigator/src/navigators/navigate_to_pose.cpp

- `default_nav_through_poses_bt_xml` 파라미터 처리
    
    ```cpp
    ...
    
    std::string
    NavigateThroughPosesNavigator::getDefaultBTFilepath(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
      std::string default_bt_xml_filename;
      auto node = parent_node.lock();
    
      if (!node->has_parameter("default_nav_through_poses_bt_xml")) {
        std::string pkg_share_dir =
          ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
        node->declare_parameter<std::string>(
          "default_nav_through_poses_bt_xml",
          pkg_share_dir +
          **"/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml");**
      }
    
      node->get_parameter("default_nav_through_poses_bt_xml", default_bt_xml_filename);
    
      return default_bt_xml_filename;
    }
    
    ...
    ```
    nav2_bt_navigator/src/navigators/navigate_through_poses.cpp

    ![](/assets/img/ros2nav2/img/Screenshot%20from%202025-12-20%2011-43-48.png)
    ![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-46.gif)
    ![](/assets/img/ros2nav2/gif/Peek%202025-12-20%2011-48.gif)

    `Running spin`, `Running wait`, `Running backup` 을 진행하는 것을 볼 수 있다.