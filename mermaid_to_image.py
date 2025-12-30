#!/usr/bin/env python3
"""
Mermaid 다이어그램을 이미지로 변환하는 스크립트
mermaid.ink API를 사용합니다.
"""

import urllib.parse
import urllib.request
import base64
import json
import sys

def mermaid_to_image(mermaid_code, output_path, format='png'):
    """
    Mermaid 코드를 이미지로 변환
    
    Args:
        mermaid_code: Mermaid 다이어그램 코드
        output_path: 출력 이미지 파일 경로
        format: 이미지 형식 ('png' 또는 'svg')
    """
    # Mermaid 코드를 base64로 인코딩
    encoded = base64.urlsafe_b64encode(mermaid_code.encode('utf-8')).decode('utf-8')
    
    # mermaid.ink API URL
    url = f"https://mermaid.ink/img/{encoded}"
    
    if format == 'svg':
        url = f"https://mermaid.ink/svg/{encoded}"
    
    try:
        # 이미지 다운로드
        print(f"다이어그램을 이미지로 변환 중...")
        urllib.request.urlretrieve(url, output_path)
        print(f"✅ 이미지 저장 완료: {output_path}")
        return True
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        return False

if __name__ == "__main__":
    mermaid_code = """graph LR
    Start[Launch 파일 실행]
    
    subgraph Launch[" "]
        direction TB
        L1[depth_camera.launch.py<br/>카메라 노드]
        L2[controller.launch.py<br/>컨트롤러 노드]
        L3[hand_detect<br/>손 검출 노드]
        L4[hand_track<br/>손 추적 노드]
    end
    
    Start --> Launch
    
    subgraph DetectNode[hand_detect 노드]
        direction TB
        D1[이미지 구독]
        D2[MediaPipe 손 검출]
        D3[손 중심 좌표 계산]
        D4[Point2D 메시지 발행]
        
        D1 --> D2 --> D3 --> D4
    end
    
    subgraph TrackNode[hand_track 노드]
        direction TB
        T1[Point2D 메시지 구독]
        T2[PID 제어<br/>X축, Y축]
        T3[PWM 서보 제어]
        
        T1 --> T2 --> T3
    end
    
    Launch --> DetectNode
    DetectNode --> TrackNode
    
    style Start fill:#e1f5ff
    style DetectNode fill:#ffe1f5
    style TrackNode fill:#e1ffe1"""
    
    output_path = "assets/img/ros2nav2/img/hand_follower_structure.png"
    
    mermaid_to_image(mermaid_code, output_path, format='png')


