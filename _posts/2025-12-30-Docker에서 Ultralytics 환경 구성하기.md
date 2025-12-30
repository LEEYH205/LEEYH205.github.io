---
title: "Docker에서 Ultralytics 환경 구성하기"
date: 2025-12-30 11:45:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, Docker, Ultralytics, YOLO]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# YOLO with ROS 2

---

## yolo_ros 레포지토리

https://github.com/mgonzs13/yolo_ros

https://drive.google.com/file/d/1wVZgi5GLkAYxv3GmTxX5z-vB8RQdwqLP/view?usp=sharing

- Ultralytics의 다양한 YOLO 모델(v3 ~ 12)을 ROS 2 환경에서 사용할 수 있도록 래핑(wrapping)한 패키지
- 객체 인식뿐만 아니라 추적(Tracking), 세그멘테이션, 포즈 추정 및 Depth 정보를 활용한 3D 인식 기능을 제공

### 주요 특징

- **Lifecycle Node 지원**
    - 관리형 노드(Managed Node)를 구현하여, **`inactive`** 상태일 때 CPU 및 VRAM 사용량을 최소화
- **Docker 지원**
    - NVIDIA Container Toolkit을 활용한 GPU 가속 환경(CUDA)을 Docker로 쉽게 배포함
- **ROS 2 호환성**
    - Humble, Iron, Jazzy, Rolling 등 주요 배포판을 모두 지원
- **YOLO-World & YOLOE 사용 가능**
    - **YOLO-World**
        
        ![](/assets/img/HiWonderPi/img/download/YOLO-World.png)
        
        - **재학습 없이 새로운 물체를 찾을 수 있는 모델**
        - **YOLO:** 학습된 80개 클래스(사람, 차, 고양이 등)만 찾을 수 있으며 "스테이플러"를 찾으려면 데이터를 모아서 다시 학습시켜야 함
        - **YOLO-World:** 텍스트(언어)와 이미지를 연결하는 비전-언어 모델(Vision-Language Model)을 사용하며, 사용자가 blue cup(파란 컵)이라고 텍스트를 입력하면, 재학습 없이 즉시 해당 물체를 인식함 (Zero-shot)
    - **YOLOE**
        - 기존(YOLOv5 이전)에는 물체 크기를 미리 가정한 틀(Anchor Box)을 썼지만, YOLOE는 이를 없애고 픽셀 단위로 물체 중심을 예측 (참고: YOLOv8 이후 모델들도 이 방식을 채택)
        - 표준 YOLO 모델보다 특정 상황에서 더 빠르고 정교한 검출 성능을 보여주기 위해 설계된 아키텍쳐

### 설치

```bash
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
pip3 install -r yolo_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src/yolo_ros --ignore-src -r -y
colcon build --symlink-install --packages-up-to yolo_bringup yolo_msgs yolo_ros
```

## YOLO 관련 ROS msg 살펴보기

### **기본 2D/3D 좌표·크기 타입**

> 이 세 개는 **이미지 위에서 위치/크기/방향을 표현하기 위한 가장 기본 타입**
> 
- **Point2D.msg**
    - 2D 점 (픽셀 좌표)
    - **`x`**, **`y`** (float64)
- **Pose2D.msg**
    - 2D 위치 + 회전 (이미지 좌표)
    - **`position`**: `Point2D`
    - **`theta`**: 회전(라디안)
- **Vector2.msg**
    - 2D 크기 (폭/높이, 픽셀 단위)
    - **`x`**, **`y`** (float64)

### **바운딩 박스 (Bounding Box)**

- **BoundingBox2D.msg**
    - 이미지(2D) 상의 박스
    - **`center`**: `Pose2D`→ 바운딩 박스 중심 위치(x, y) + 회전(theta)
    - **`size`**: `Vector2`→ 박스의 가로/세로 크기 (픽셀 단위)
- **BoundingBox3D.msg**
    - 3D 공간에서의 박스
    - **`center`**: `geometry_msgs/Pose`→ 3D 위치 + 쿼터니언 회전
    - **`size`**: `geometry_msgs/Vector3`→ 폭/높이/깊이 (미터 단위)
    - **`frame_id`**: 어떤 좌표계(예: `base_link`) 기준인지

### **키포인트(포즈 추정) 관련**

> **사람 포즈 추정(YOLO pose)** 결과를 2D/3D로 표현하기 위한 메시지 세트
> 
- **KeyPoint2D.msg**
    - 한 개의 2D 키포인트
    - **`id`**: 키포인트 번호 (예: 1 = 코, 2 = 왼눈, …)
    - **`point`**: `Point2D` (픽셀 위치)
    - **`score`**: 신뢰도
- **KeyPoint2DArray.msg**
    - 여러 개의 2D 키포인트 모음
    - **`data`**: `KeyPoint2D[]`
- **KeyPoint3D.msg**
    - 한 개의 3D 키포인트
    - **`id`**: 번호
    - **`point`**: `geometry_msgs/Point` (미터 단위 3D 좌표)
    - **`score`**: 신뢰도
- **KeyPoint3DArray.msg**
    - 여러 개의 3D 키포인트 모음
    - **`data`**: `KeyPoint3D[]`
    - **`frame_id`**: 어느 좌표계 기준인지


### **마스크 및 전체 Detection 구조**

- **Mask.msg**
    - 한 개 객체의 세그멘테이션 마스크
    - **`height`**, **`width`**: 원본 이미지 크기
    - **`data`**: `Point2D[]`→ 마스크의 **외곽 경계선 좌표 리스트** (폴리곤 형태 느낌)
- **Detection.msg**
    - **YOLO 한 개 검출 결과 전체**를 담는 메시지
    - 클래스 정보
        - **`class_id`**, **`class_name`**, **`score`**
    - 추적용 ID
        - **`id`**: 트래킹 ID
    - 위치 정보
        - **`bbox`**: `BoundingBox2D` (이미지 상 2D 박스)
        - **`bbox3d`**: `BoundingBox3D` (3D 박스)
    - 형태 정보
        - **`mask`**: `Mask` (세그멘테이션 경계)
    - 포즈 정보
        - **`keypoints`**: `KeyPoint2DArray` (2D 포즈)
        - **`keypoints3d`**: `KeyPoint3DArray` (3D 포즈)
- **DetectionArray.msg**
    - 한 프레임에 검출된 모든 객체
    - **`header`**: `std_msgs/Header` (타임스탬프, frame_id 등)
    - **`detections`**: `Detection[]`