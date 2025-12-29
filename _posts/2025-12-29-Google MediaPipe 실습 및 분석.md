---
title: "Google MediaPipe 실습 및 분석"
date: 2025-12-29 14:40:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, MediaPipe]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# Google MediaPipe
MediaPipe는 Google에서 개발한 멀티모달 머신러닝 파이프라인 프레임워크로, 실시간 컴퓨터 비전 및 오디오 처리에 특화된 솔루션을 제공합니다. 복잡한 ML 모델을 단일 파이프라인으로 구성하고, 모바일·웹·데스크톱·임베디드 환경에서 최적화된 성능을 제공합니다.

## MediaPipe 핵심 개념

![image.png](/assets/img/HiWonderPi/img/download/MediaPipe.png)

### [Graph](https://developers.google.com/mediapipe/framework/framework_concepts/graphs.md)

- Calculator 들이 연결된 처리 파이프라인
- `.pbtxt` 형태로 구성

### [Calculator](https://developers.google.com/mediapipe/framework/framework_concepts/calculators.md)

- MediaPipe의 기본 단위 연산 노드
- 예: 이미지를 입력 → 전처리 → 결과 계산 → 다음 노드 전달

### [Packet](https://developers.google.com/mediapipe/framework/framework_concepts/packets.md)

- 개별 데이터 단위를 의미 (예: 이미지 프레임, 포인트 정보)



# Google MediaPipe 실습

---

## 개요

- MentorPi의 `~/ros2_ws/src/example/example/mediapipe_example` 내 구현된 간단한 MediaPipe 예제들은 아래와 같습니다.
    - **Hand Landmarker**: 손의 21개 랜드마크 검출
    - **Hand Gesture**: 손가락 각도 기반 제스처 인식
    - **Face Detection**: 얼굴 위치 검출
    - **Face Mesh**: 얼굴 468개 랜드마크 및 Blendshapes 검출
    - **Pose Landmarker**: 전신 33개 포즈 랜드마크 검출
    - **Holistic**: 얼굴, 손, 포즈 통합 검출
    - **Objectron**: 3D 객체 바운딩 박스 검출
    - **Selfie Segmentation**: 사람과 배경 분리


## 공통 사전 준비

1. MentorPi를 시작하고 VNC Viewer에 연결합니다.
2. **Terminator**에서 아래 명령어를 입력해 ROS 2관련 자동 실행 서비스를 중지합니다.
    
    ```bash
    ~/.stop_ros.sh
    ```
    
3. 카메라 드라이버를 아래 명령어로 켜줍니다.
    
    ```bash
    ros2 launch peripherals depth_camera.launch.py
    ```
    
4. 실습을 위해 예제 디렉토리로 이동합니다.
    
    ```bash
    cd ros2_ws/src/example/example/mediapipe_example
    ```
    

## 공통 프로그램 실행 흐름

1. **ROS2 노드 초기화**: `rclpy.init()` 및 노드 생성
2. **이미지 구독 설정**: 카메라 토픽 구독 시작
3. **MediaPipe 모델 로드**: 각 예제에 맞는 모델 초기화
4. **이미지 처리 루프**: 이미지 수신 → 이미지 처리 → 시각화
5. **종료**: `q` 또는 `ESC` 키 입력 시 종료

## 예제별 실습 및 코드 분석

### Hand Landmarker (손 랜드마크 검출)

![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2014-56.gif)

- 실행
    
    ```bash
    python3 hand.py
    ```
    
- 동작 원리
    - Hand Landmarker는 MediaPipe Tasks API를 사용하여 손의 21개 랜드마크를 검출
    - 사전 학습된 `.task` 모델 파일을 로드하여 실시간으로 손의 위치와 형태를 추적
- 핵심 코드
    - **모델 초기화 (`HandNode.__init__`)**
        
        ```python
        # 모델 파일 경로 설정
        model_path = os.path.join(..., 'model/hand_landmarker.task')
        
        # MediaPipe Tasks 옵션 설정
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=2  # 최대 2개의 손 검출
        )
        
        # detector 생성
        self.detector = vision.HandLandmarker.create_from_options(options)
        ```
        
    - **검출 수행 (`HandNode.main`)**
        
        ```python
        # 이미지 좌우 반전 (거울 모드)
        image = cv2.flip(image, 1)
        
        # MediaPipe Image 형식으로 변환
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        
        # 손 랜드마크 검출
        detection_result = self.detector.detect(mp_image)
        
        # 결과 시각화
        annotated_image = draw_hand_landmarks_on_image(image, detection_result)
        ```

### Hand Gesture (손 제스처 인식)

![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2014-59.gif)
- 실행
    
    ```bash
    python3 hand_gesture.py
    ```
- 동작 원리
    - Hand Gesture는 MediaPipe Hands를 사용하여 손 랜드마크를 검출한 후, 각 손가락의 굽힘 각도를 계산하여 제스처를 판별
    - 2D 벡터 각도 계산을 통해 손가락이 펴졌는지 굽혀졌는지를 판단
    - 사용방법
        - 손으로 `1`을 표시하면, 그림 그리기
        - 손으로 `5`를 표시하면, 그림 초기화
- 핵심 코드
    - **MediaPipe Hands 초기화 (`HandGestureNode.__init__`)**
        
        ```python
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,      # 비디오 스트림 모드
            max_num_hands=1,              # 최대 1개의 손 검출
            min_tracking_confidence=0.05, # 추적 신뢰도 임계값
            min_detection_confidence=0.6  # 검출 신뢰도 임계값
        )
        ```
        
    - **랜드마크 좌표 변환 (`get_hand_landmarks`)**
        
        ```python
        def get_hand_landmarks(img, landmarks):
            """
            MediaPipe의 정규화된 좌표를 픽셀 좌표로 변환
            """
            h, w, _ = img.shape
            # 정규화 좌표(0~1) → 픽셀 좌표로 변환
            landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
            return np.array(landmarks)
        ```
        
    - **손가락 각도 계산 (`hand_angle`)**
        
        ```python
        def hand_angle(landmarks):
            """
            각 손가락의 굽힘 각도 계산
            """
            angle_list = []
            
            # 엄지: 랜드마크 3-4 벡터와 0-2 벡터 사이 각도
            angle_ = vector_2d_angle(
                landmarks[3] - landmarks[4],  # 엄지 끝 방향
                landmarks[0] - landmarks[2]   # 손목-엄지 기준 방향
            )
            angle_list.append(angle_)
            
            # 검지: 랜드마크 0-6 벡터와 7-8 벡터 사이 각도
            angle_ = vector_2d_angle(
                landmarks[0] - landmarks[6],
                landmarks[7] - landmarks[8]
            )
            angle_list.append(angle_)
            
            # 중지, 약지, 소지도 동일한 방식으로 계산
            # ...
            
            return [abs(a) for a in angle_list]
        ```
        
        - 손 랜드마크들 인덱스
        ![](/assets/img/HiWonderPi/img/download/hand_landmark_index.png)
    - **제스처 판별 (`h_gesture`)**
    
    ```python
    def h_gesture(angle_list):
        """
        손가락 각도를 기반으로 제스처 판별
        """
        thr_angle = 65.       # 손가락 굽힘 임계값
        thr_angle_thumb = 53. # 엄지 굽힘 임계값
        thr_angle_s = 49.     # 손가락 펴짐 임계값
        
        # 모든 손가락이 굽혀짐 → 주먹
        if (angle_list[0] > thr_angle_thumb) and \
           (angle_list[1] > thr_angle) and \
           (angle_list[2] > thr_angle) and \
           (angle_list[3] > thr_angle) and \
           (angle_list[4] > thr_angle):
            return "fist"
        
        # 검지만 펴짐 → one
        elif (angle_list[0] > 5) and \
             (angle_list[1] < thr_angle_s) and \
             (angle_list[2] > thr_angle) and \
             (angle_list[3] > thr_angle) and \
             (angle_list[4] > thr_angle):
            return "one"
        
        # 모든 손가락이 펴짐 → five
        elif all(angle < thr_angle_s for angle in angle_list):
            return "five"
        
        # ... 기타 제스처
        return "none"
    ```
### Face Detection (얼굴 검출)

![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2015-04.gif)

- 실행
    
    ```bash
    python3 face_detect.py
    ```

- 동작 원리
    - Face Detection은 MediaPipe Tasks API의 FaceDetector를 사용하여 이미지에서 얼굴의 위치(바운딩 박스)와 주요 특징점(눈, 코, 입 등)을 검출합니다.
- 핵심 코드
    - **모델 초기화 (`FaceDetectionNode.__init__`)**
        
        ```python
        # TFLite 모델 파일 로드
        model_path = os.path.join(..., 'model/detector.tflite')
        
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceDetectorOptions(base_options=base_options)
        
        # Face Detector 생성
        self.detector = vision.FaceDetector.create_from_options(options)
        ```
        
    - **검출 수행 (`FaceDetectionNode.main`)**
        
        ```python
        image = cv2.flip(image, 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        
        # 얼굴 검출
        detection_result = self.detector.detect(mp_image)
        
        # 결과 시각화 (바운딩 박스, 특징점)
        annotated_image = visualize(image, detection_result)
        ```

### Face Mesh (얼굴 메시)

![](https://camo.githubusercontent.com/9e4ea63d67f5029f6c0aee4af2c0a0d3d3422c75693d71fad82f12a7db0c7bf5/68747470733a2f2f6d65646961706970652e6465762f696d616765732f666163655f6d6573685f61725f656666656374732e676966)
![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2015-25.gif)
- 실행
    
    ```bash
    python3 face_mesh.py
    ```
- 동작 원리
    - Face Mesh는 얼굴의 468개 3D 랜드마크를 검출하고, Blendshapes(표정 파라미터)와 얼굴 변환 행렬을 출력
    - 이를 통해 얼굴 표정 분석, AR 필터 적용 등이 가능
- 핵심 코드
    - **모델 초기화 (`FaceMeshNode.__init__`)**
        
        ```python
        model_path = os.path.join(..., 'model/face_landmarker_v2_with_blendshapes.task')
        
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.FaceLandmarkerOptions(
            base_options=base_options,
            output_face_blendshapes=True,                  # 표정 파라미터 출력
            output_facial_transformation_matrixes=True,   # 변환 행렬 출력
            num_faces=1                                    # 최대 1개 얼굴 검출
        )
        
        self.detector = vision.FaceLandmarker.create_from_options(options)
        ```
        
    - **검출 수행 (`FaceMeshNode.main`)**
        
        ```python
        image = cv2.flip(image, 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        
        # 얼굴 메시 검출
        detection_result = self.detector.detect(mp_image)
        
        # 결과 시각화 (468개 랜드마크 연결)
        annotated_image = draw_face_landmarks_on_image(image, detection_result)
        ```

### Pose Landmarker (포즈 검출)
![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2015-28.gif)

- 실행
    
    ```bash
    python3 pose.py
    ```
    
- 동작 원리
    - Pose Landmarker는 전신의 33개 포즈 랜드마크를 검출
    - 어깨, 팔꿈치, 손목, 엉덩이, 무릎, 발목 등의 관절 위치를 추적하며, 선택적으로 세그멘테이션 마스크도 출력할 수 있음

- 핵심 코드
    - **모델 초기화 (`PoseNode.__init__`)**
        
        ```python
        model_path = os.path.join(..., 'model/pose_landmarker.task')
        
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True  # 세그멘테이션 마스크 출력
        )
        
        self.detector = vision.PoseLandmarker.create_from_options(options)
        ```
        
    - **검출 수행 (`PoseNode.main`)**
        
        ```python
        image = cv2.flip(image, 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        
        # 포즈 검출
        detection_result = self.detector.detect(mp_image)
        
        # 결과 시각화 (스켈레톤)
        annotated_image = draw_pose_landmarks_on_image(image, detection_result)
        ```

### Holistic (통합 검출)

![](/assets/img/HiWonderPi/gif/Peek%202025-12-29%2015-32.gif)
- 실행
    
    ```bash
    python3 holistic.py
    ```
- 동작 원리
    - Holistic은 얼굴, 손, 포즈를 동시에 검출하는 통합 솔루션
    - 단일 모델로 전신의 모든 랜드마크를 추적할 수 있어 효율적
- 핵심 코드
    - **Holistic 초기화 (`HolisticNode.__init__`)**
        
        ```python
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        ```
        
    - **검출 수행 (`HolisticNode.main`)**
        
        ```python
        with self.mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        ) as holistic:
            
            # 성능 향상을 위해 이미지를 읽기 전용으로 설정
            image.flags.writeable = False
            results = holistic.process(image)
            
            # 결과 시각화
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # 얼굴 메시 그리기
            self.mp_drawing.draw_landmarks(
                image,
                results.face_landmarks,
                self.mp_holistic.FACEMESH_CONTOURS,
                landmark_drawing_spec=None,
                connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
            )
            
            # 포즈 스켈레톤 그리기
            self.mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
        ```

### Objectron (3D 객체 검출)



- 실행
    
    ```bash
    python3 objectron.py
    ```

- 동작 원리
    - Objectron은 특정 객체(컵, 의자, 신발, 카메라)의 3D 바운딩 박스를 검출합니다. 2D 이미지에서 객체의 3D 위치, 회전, 크기를 추정하여 AR 응용에 활용할 수 있습니다.
    - **지원 객체**
        - `Cup`: 컵
        - `Chair`: 의자
        - `Shoe`: 신발
        - `Camera`: 카메라
- 핵심 코드
    - **Objectron 초기화 (`ObjectronNode.__init__`)**
        
        ```python
        self.mp_objectron = mp.solutions.objectron
        self.mp_drawing = mp.solutions.drawing_utils
        ```
        
    - **검출 수행 (`ObjectronNode.main`)**
        
        ```python
        with self.mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,             # 최대 1개 객체 검출
            min_detection_confidence=0.4,
            min_tracking_confidence=0.5,
            model_name='Cup'               # 검출할 객체 종류
        ) as objectron:
            
            # 성능 향상을 위해 이미지를 읽기 전용으로 설정
            image.flags.writeable = False
            results = objectron.process(image)
            
            # 결과 시각화
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if results.detected_objects:
                for detected_object in results.detected_objects:
                    # 2D 바운딩 박스 그리기
                    self.mp_drawing.draw_landmarks(
                        image,
                        detected_object.landmarks_2d,
                        self.mp_objectron.BOX_CONNECTIONS
                    )
                    
                    # 3D 축 그리기 (회전, 이동 정보 사용)
                    self.mp_drawing.draw_axis(
                        image,
                        detected_object.rotation,
                        detected_object.translation
                    )
        ```

    ### Selfie Segmentation (배경 분리)


- 실행
    
    ```bash
    python3 self_segmentation.py
    ```
- 동작 원리
    - Selfie Segmentation은 이미지에서 사람 영역을 분리하여 세그멘테이션 마스크를 생성
    - 이 마스크를 사용하여 배경을 다른 이미지나 색상으로 대체할 수 있음
- 핵심 코드
    - **Selfie Segmentation 초기화 (`SegmentationNode.__init__`)**
        
        ```python
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.BG_COLOR = (192, 192, 192)  # 회색 배경
        ```
        
    - **검출 및 배경 합성 (`SegmentationNode.main`)**
        
        ```python
        with self.mp_selfie_segmentation.SelfieSegmentation(
            model_selection=1  # 0: 일반 모델, 1: landscape 모델
        ) as selfie_segmentation:
            
            # 성능 향상을 위해 이미지를 읽기 전용으로 설정
            image.flags.writeable = False
            results = selfie_segmentation.process(image)
            
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # 마스크 조건 생성 (임계값 0.1 이상이면 사람 영역)
            condition = np.stack(
                (results.segmentation_mask,) * 3,  # 3채널로 확장
                axis=-1
            ) > 0.1
            
            # 배경 이미지 생성
            if bg_image is None:
                bg_image = np.zeros(image.shape, dtype=np.uint8)
                bg_image[:] = self.BG_COLOR
            
            # 조건에 따라 합성 (사람: 원본, 배경: 회색)
            output_image = np.where(condition, image, bg_image)
        ```
        
