---
title: "LMM, VLM, VLA 개요 및 실습"
date: 2025-12-30 16:00:00 +0900
categories: [ROBOTICS, PiRobot]
tags: [Robotics, ROS2, LMM, VLM, VLA]
description: ""
pin: false
math: true
mermaid: true
hidden: false
---

# LMM / VLM 개요

---

## LMM(Large Multimodal Model)이란?

![](/assets/img/HiWonderPi/img/download/LMM.png)

> 텍스트뿐만 아니라 이미지, 오디오, 비디오 등 여러 모달리티(Modality)를 함께 이해하고 처리할 수 있는 대규모 AI 모델
> 

| 구분 | LLM | LMM |
| --- | --- | --- |
| **입력** | 텍스트만 | 텍스트 + 이미지 + 오디오 등 |
| **출력** | 텍스트 | 텍스트 (일부는 이미지 생성도 가능) |
| **핵심 능력** | 언어 이해/생성 | 멀티모달 이해/추론 |

### 모달리티(Modality)란?

> 정보를 표현하는 형태 또는 채널
> 
> - **텍스트**: 자연어 문장
> - **이미지**: 사진, 그림, 스크린샷
> - **오디오**: 음성, 음악, 환경음
> - **비디오**: 연속된 이미지 + 오디오
> - **센서 데이터**: LiDAR, IMU, 깊이 센서 등

## VLM(Vision-Language Model)이란?

![](/assets/img/HiWonderPi/img/download/VLM.png)
> LMM의 한 종류로, 이미지(Vision)와 텍스트(Language)를 함께 이해하는 모델
> 

| 구분 | 설명 |
| --- | --- |
| **입력** | 이미지 + 텍스트(질문/명령) |
| **출력** | 텍스트(설명, 분석, JSON 등) |
| **핵심 구조** | Vision Encoder + LLM |
| **대표 모델** | GPT-4V, Gemini Pro Vision, LLaVA, Qwen-VL |

### VLM의 동작 원리
![](/assets/img/HiWonderPi/img/download/VLM_2.png)
1. 입력 (Inputs)
    - **이미지 입력 (Image Input):** 원본 이미지(Raw Image) 형태로 모델에 제공됨
    - **텍스트 프롬프트 (Text Prompt):** 자연어 질문 또는 명령(Natural Language Question/Command) 형태로 제공됨
2. 인코더 (Encoders)
    - **비전 인코더 (Vision Encoder, 예: ViT)**
        - **Patchify:** 원본 이미지를 여러 개의 작은 조각(Patch)으로 분할함
        - **Visual Embeddings:** 각 이미지 조각을 컴퓨터가 이해하는 숫자 벡터(Visual Embeddings)로 변환함
    - **텍스트 인코더 (Text Encoder)**
        - **Tokenization:** 텍스트를 개별 단어 또는 서브워드(토큰)로 분할함
        - **Text Embeddings:** 분할된 각 토큰을 숫자 벡터(Text Embeddings)로 변환함
3. 연결 및 정렬 (Bridge & Alignment)
    - **Alignment Projector** (예: Linear Layer/MLP)
        - **Cross-Modal Projection:** 시각 임베딩(Visual Embeddings)을 거대 언어 모델(LLM)이 이해할 수 있는 차원으로 투영하고 변환함
        - **Alignment:** 이미지와 텍스트 임베딩을 동일한 공간(Unified Space)으로 정렬하여 통합된 토큰(Unified Tokens)으로 만듦
4. LLM 백본 (LLM Backbone)
    - **Large Language Model** (예: Transformer Decoder)
        - **Contextual Understanding:** 통합된 토큰을 입력받아 이미지와 텍스트 간의 관계 및 문맥을 파악
        - **Next-Token Prediction:** 문맥을 바탕으로 다음에 올 가장 적절한 단어(토큰)를 순차적으로 예측하고 생성
5. 출력 (Output)
    - **텍스트 생성 (Text Generation):** LLM이 예측한 토큰들을 조합하여 자연어 문장으로 변환
    - **생성된 응답 (Generated Response):** 사용자의 질문에 대한 최종적인 설명이나 답변을 제공

## 로보틱스에서 VLM이 중요한 이유

> VLM을 활용하면 로봇이 본 것을 이해하고 자연어로 소통할 수 있음
> 

| 기존 컴퓨터 비전 | VLM 활용 |
| --- | --- |
| 객체 인식: “사람 detected” | “파란 셔츠를 입은 사람이 왼쪽에서 걸어오고 있음” |
| 장면 분류: “실내” | “회의실로 보이며, 테이블 위에 노트북 3대가 있음” |
| 별도 모델 필요 (객체/장면/OCR…) | 하나의 VLM으로 통합 처리 가능 |
| 고정된 클래스만 인식 | 자연어로 유연한 질의 가능 |

### 로봇에서의 VLM 활용 사례

| 활용 | 설명 | 예시 질문 |
| --- | --- | --- |
| **장면 분석** | 전방 카메라 영상 분석 | “이 환경에서 주행 가능한가?” |
| **객체 인식** | 특정 물체 위치 파악 | “빨간 공이 어디 있어?” |
| **텍스트 인식(OCR)** | 표지판/안내문 읽기 | “저 표지판에 뭐라고 써있어?” |
| **상황 판단** | 위험/안전 평가 | “이 길로 가도 안전할까?” |
| **조작 가이드** | 물체 잡는 방법 제안 | “이 컵을 어떻게 잡아야 해?” |


# VLA 개요

---

## VLA(Vision-Language-Action)란?

> 이미지(Vision)를 보고, 언어(Language)로 태스크를 이해하며, 행동(Action)을 직접 출력하는 모델
> 

| 구분 | VLM | VLA |
| --- | --- | --- |
| **입력** | 이미지 + 텍스트 | 이미지 + 텍스트(태스크) |
| **출력** | 텍스트 | **로봇 행동 (관절 각도, 속도 등)** |
| **목적** | 이미지 이해/설명 | 로봇 제어 |
| **학습 데이터** | 이미지-텍스트 쌍 | 이미지-텍스트-행동 데이터셋 |

### VLA의 동작 원리

![](/assets/img/HiWonderPi/img/download/VLA.png)

1. 멀티모달 입력 (Multimodal Inputs)
    - **시각 관측 (Visual Observation):** 로봇에 장착된 카메라를 통해 입력되는 현재 환경의 이미지 (예: 물건이 놓인 책상)
    - **언어 지시 (Language Instruction):** 로봇에게 작업을 요청하는 자연어 명령 (예: "서랍을 열고 캔을 넣어줘")
2. VLA Backbone Model
    - VLM이 가진 일반적인 세상에 대한 상식과 시각적 추론
    - "이 상황(이미지)에서 이 명령(텍스트)을 수행하려면 어떻게 움직여야 하는가?"를 판단
3. 액션 토큰화 (Action Tokenization)
    - **통합 출력 스트림 (Unified Output Stream):** 텍스트 답변과 로봇의 물리적 제어 신호를 구분하지 않고 하나의 연속된 데이터 흐름으로 취급함
    - **액션 토큰 생성 (Action Tokens)**
        - 로봇의 팔 위치(x, y, z 좌표), 회전 각도, 그리퍼 개폐 여부 등을 단어(Token)처럼 변환하여 출력
        - 예시 출력: `[I]` `[will]` `[pick]` `[<x=102>]` `[<y=45>]` `[<gripper=close>]`
4. 물리적 실행 (Physical Execution)
    - **De-tokenizer:** 모델이 내뱉은 추상적인 '액션 토큰(숫자)'을 실제 모터의 전압이나 속도 명령으로 변환
    - **행동 수행 (Robot Action):** 변환된 신호에 따라 로봇 팔(Manipulator)이 실제로 움직이며 동작 수행

### 대표 VLA 모델

| 모델 | 개발사 | 특징 |
| --- | --- | --- |
| **RT-1** | Google | 로봇 조작 전용, 130K 에피소드 학습 |
| **RT-2** | Google | PaLM-E 기반, 언어 이해 + 로봇 제어 |
| **OpenVLA** | Stanford | 오픈소스 VLA, 7B 파라미터 |
| **π₀ (pi-zero)** | Physical Intelligence | Flow Matching 기반, 범용 조작 |
| **Octo** | UC Berkeley | 다양한 로봇 형태 지원 |

## End-to-End VLA vs 소프트웨어 파이프라인

### End-to-End VLA

> 이미지와 태스크를 하나의 신경망에 넣으면, 로봇 제어 신호가 바로 출력됨
> 

| 특징 | 설명 |
| --- | --- |
| **단일 모델** | 인식 → 판단 → 행동이 하나의 네트워크에서 수행 |
| **End-to-End 학습** | 로봇 데모 데이터로 전체 파이프라인을 한 번에 학습 |
| **직접 행동 출력** | 관절 각도, 속도 등 Low-level 제어 신호 출력 |
| **대표 모델** | RT-2 (Google), OpenVLA, π₀ (Physical Intelligence) |

```
[이미지 + "컵을 집어"] ──▶ [End-to-End VLA] ──▶ [joint: [0.1, -0.3, 0.5, ...]]
```

### 소프트웨어 VLA 파이프라인

> 기존 VLM + LLM을 소프트웨어로 연결하여 VLA와 유사한 동작을 구현
> 

| 특징 | 설명 |
| --- | --- |
| **다단계 처리** | Vision → Language → Action을 별도 모듈로 구성 |
| **기존 모델 활용** | GPT-4V, Gemini 등 범용 VLM/LLM 사용 |
| **규칙 기반 매핑** | LLM 출력(JSON)을 코드로 로봇 명령에 매핑 |
| **유연하지만 느림** | API 호출 지연, 실시간 제어에는 부적합 |

```
[이미지] ──▶ [VLM: 장면 분석] ──▶ [LLM: 행동 결정] ──▶ [코드: 명령 변환] ──▶ [cmd_vel]
```

### 비교 요약

| 구분 | End-to-End VLA | 소프트웨어 파이프라인 |
| --- | --- | --- |
| **속도** | 빠름 (수십 ms) | 느림 (수 초) |
| **정확도** | 전용 학습 시 높음 | 범용 모델이라 제한적 |
| **유연성** | 학습된 태스크만 | 프롬프트로 다양한 태스크 가능 |
| **구현 난이도** | 높음 (전용 모델/데이터 필요) | 낮음 (API 호출로 가능) |
| **실시간 제어** | 가능 | 어려움 |
| **학습 비용** | 높음 (로봇 데모 수집 필요) | 없음 (기존 모델 사용) |


# VLM/VLA 실습

---

## 실습 환경 구성

> 실습에 사용할 파일들을 아래 명령어로 만들어줍니다.
> 

```bash
cd ~/ros2_ws/src/large_models/large_models/openrouter
touch vlm_basic.py vlm_bbox.py vlm_robot_action.py vla_style_pipeline.py
```

### VLM 모델 선택 시 주의사항

> VLM은 이미지 입력을 지원하는 모델이어야 하며, 모델 선택후 config.json에 넣어줍니다.
>

```json
{
  "provider": "openrouter",
  "api_key_env": "OPENROUTER_API_KEY",
  "api_key": "",
  "base_url": "https://openrouter.ai/api/v1",
  "llm_model": "nvidia/nemotron-nano-9b-v2:free",
  **"vlm_model": "nvidia/nemotron-nano-12b-v2-vl:free", # VLM 모델**
  ...
}
```
2025.12.30 현재 `nvidia/nemotron-nano-12b-v2-vl:free`가 사용 가능했다.

## 실습 1: VLM 기본 이미지 분석 (vlm_basic.py)

> 이미지 URL을 입력받아 로봇 주행 관점에서 장면을 분석
> 

```python
import argparse

from common import (
    chat_complete,
    default_generation_params,
    get_client,
    load_config,
)

DEFAULT_IMAGE_URL = "https://www.cbiz.kr/news/photo/201712/12478_15145_1549.jpg"
DEFAULT_QUESTION = "이 환경에 대해 분석해줘. 어떻게 주행해야 할까?"
# DEFAULT_IMAGE_URL = "https://csossihettpx2597658.cdn.gov-ntruss.com/data2/content/image/2020/12/08/.cache/512/20201208254670.jpg"
# DEFAULT_QUESTION = "보이는 텍스트 분석해줘."

def analyze_image_url(image_url: str, question: str) -> str:
    """URL 이미지를 VLM으로 분석하여 장면 설명을 반환"""
    cfg = load_config()
    client = get_client(cfg)

    system_rule = """
너는 모바일 로봇의 비전 분석 AI다.
- 로봇 전방 카메라로 촬영된 이미지를 분석한다.
- 로봇 주행에 중요한 정보를 우선적으로 보고한다.
- 출력 형식:
  1) 장면 요약 (1줄)
  2) 감지된 객체 목록
  3) 주행 관련 주의사항
  4) 권장 행동
""".strip()

    user_content = [
        {
            "type": "text",
            "text": question,
        },
        {
            "type": "image_url",
            "image_url": {"url": image_url},
        },
    ]

    messages = [
        {"role": "system", "content": system_rule},
        {"role": "user", "content": user_content},
    ]

    params = default_generation_params(cfg)
    params["max_tokens"] = 1024  # 이미지 분석은 더 긴 응답 허용

    return chat_complete(
        client=client,
        model=cfg["vlm_model"],
        messages=messages,
        **params,
    )

def main() -> None:
    parser = argparse.ArgumentParser(description="VLM basic example (URL image)")
    parser.add_argument(
        "--url",
        default=DEFAULT_IMAGE_URL,
        help="Image URL to analyze",
    )
    parser.add_argument(
        "--question",
        default=DEFAULT_QUESTION,
        help="Question to ask about the image",
    )
    args = parser.parse_args()

    print("=== VLM Basic (URL) ===")
    print(f"URL: {args.url}")
    print(f"Question: {args.question}\n")

    result = analyze_image_url(args.url, args.question)
    print(result)

if __name__ == "__main__":
    main()
```

- **실행 방법**
    
    ```bash
    # 기본 실행 (기본 URL + 기본 질문 사용)
    python3 vlm_basic.py
    
    # URL 지정
    python3 vlm_basic.py --url "https://example.com/image.jpg"
    
    # 질문 지정
    python3 vlm_basic.py --question "이 장면에서 위험 요소가 있나요?"
    ```

![](https://www.cbiz.kr/news/photo/201712/12478_15145_1549.jpg)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-28-06.png)

![](https://postfiles.pstatic.net/MjAyNDEwMjVfNjYg/MDAxNzI5ODM2NTg1MDgy.Ttrxk_0dHE1GODrMwy4rVlIBZ0KOkTvNyTF-7fSnsWsg.MZqC1XTsQ-lZ0AqifslIXPrdlhanhoqMG5bwTSWK_lAg.PNG/SE-3248e975-81eb-47f4-b1e2-79a80527c782.png?type=w966)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-35-56.png)

![](https://postfiles.pstatic.net/MjAyNDEwMjVfMTUw/MDAxNzI5ODM2MTM0MDEz.4De5Lmy8bJM3vfopip2ZK3G894BN4vY-9nTMm_EcIE4g.NuKJRUXbVjjySc5xE3WaOkdSTbZVhGvoRp9PfJnEMFEg.PNG/%ED%99%94%EB%A9%B4_%EC%BA%A1%EC%B2%98_2024-10-25_150045.png?type=w966)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-37-04.png)
무료모델이라 한계가 있다.

## 실습 2: 객체 위치 감지 + 바운딩 박스 (vlm_bbox.py)

> VLM에게 객체 위치를 JSON(bbox)으로 요청하고, 이미지에 바운딩 박스를 그려 저장
> 

```
import argparse
import json
import re
import urllib.request
from io import BytesIO
from typing import Any, Dict, List, Tuple

from common import chat_complete, default_generation_params, get_client, load_config

DEFAULT_IMAGE_URL = "https://upload.wikimedia.org/wikipedia/commons/thumb/1/14/Seals%40melb_zoo.jpg/960px-Seals%40melb_zoo.jpg"
DEFAULT_OUT_PATH = "vlm_bbox_result.png"

def _require_pillow():
    try:
        from PIL import Image, ImageDraw, ImageFont  # noqa: F401

        return True
    except Exception:
        return False

def download_image_bytes(url: str) -> bytes:
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) MentorPi/1.0",
        },
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read()

def get_image_size(image_bytes: bytes) -> Tuple[int, int]:
    from PIL import Image

    im = Image.open(BytesIO(image_bytes))
    return int(im.size[0]), int(im.size[1])

def strip_code_fences(text: str) -> str:
    t = text.strip()
    if t.startswith("```"):
        lines = t.splitlines()
        if len(lines) >= 3:
            return "\n".join(lines[1:-1]).strip()
    return t

def extract_first_json_object(text: str) -> str:
    """LLM이 주변에 설명을 섞어도 첫 JSON 객체/배열을 최대한 추출"""

    t = strip_code_fences(text)

    # 먼저 완전한 JSON만 있는 경우
    if t.startswith("{") or t.startswith("["):
        return t

    # 중간에 JSON이 있는 경우(간단 히ュー리스틱)
    m = re.search(r"(\{.*\}|\[.*\])", t, re.DOTALL)
    if m:
        return m.group(1).strip()

    return t

def parse_bbox_response(text: str) -> Dict[str, Any]:
    raw = extract_first_json_object(text)
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {"parse_error": True, "raw_response": text}

def normalize_bbox(bbox: List[float], w: int, h: int) -> List[int]:
    """bbox가 정규화(0~1)로 오면 픽셀로 변환, 아니면 클램핑"""
    if len(bbox) != 4:
        return [0, 0, 0, 0]

    x1, y1, x2, y2 = bbox

    # normalized heuristic
    if 0.0 <= x1 <= 1.0 and 0.0 <= x2 <= 1.0 and 0.0 <= y1 <= 1.0 and 0.0 <= y2 <= 1.0:
        x1, x2 = x1 * w, x2 * w
        y1, y2 = y1 * h, y2 * h

    x1i = max(0, min(int(round(x1)), w - 1))
    y1i = max(0, min(int(round(y1)), h - 1))
    x2i = max(0, min(int(round(x2)), w - 1))
    y2i = max(0, min(int(round(y2)), h - 1))

    # ensure proper ordering
    if x2i < x1i:
        x1i, x2i = x2i, x1i
    if y2i < y1i:
        y1i, y2i = y2i, y1i

    return [x1i, y1i, x2i, y2i]

def request_bboxes(image_url: str, width: int, height: int, max_objects: int) -> Dict[str, Any]:
    cfg = load_config()
    client = get_client(cfg)

    system_rule = f"""
너는 모바일 로봇의 비전 인식 AI다.
이미지를 보고 로봇 주행/조작에 의미있는 객체들의 위치를 바운딩 박스로 추정한다.

- 이미지 크기: width={width}, height={height}
- 객체 개수는 최대 {max_objects}개까지만 출력
- bbox 좌표는 픽셀 좌표계를 사용한다:
  - bbox = [x1, y1, x2, y2]
  - (0,0)은 이미지의 좌상단
  - x는 오른쪽으로 증가, y는 아래로 증가
  - 0 <= x1 < x2 < width, 0 <= y1 < y2 < height

반드시 아래 JSON 스키마만 출력하라(다른 텍스트 금지):
{{
  "image": {{"width": {width}, "height": {height}}},
  "objects": [
    {{"label": "object_name", "confidence": 0.0, "bbox": [0, 0, 10, 10]}}
  ]
}}
""".strip()

    user_content = [
        {"type": "text", "text": "이미지에서 중요한 객체들의 위치를 bbox로 추정해줘."},
        {"type": "image_url", "image_url": {"url": image_url}},
    ]

    messages = [
        {"role": "system", "content": system_rule},
        {"role": "user", "content": user_content},
    ]

    params = default_generation_params(cfg)
    params["temperature"] = 0.1
    params["max_tokens"] = 1200

    resp_text = chat_complete(
        client=client,
        model=cfg["vlm_model"],
        messages=messages,
        **params,
    )

    return parse_bbox_response(resp_text)

def draw_bboxes(image_bytes: bytes, parsed: Dict[str, Any], out_path: str) -> None:
    from PIL import Image, ImageDraw, ImageFont

    im = Image.open(BytesIO(image_bytes)).convert("RGB")
    draw = ImageDraw.Draw(im)

    w, h = im.size

    try:
        font = ImageFont.load_default()
    except Exception:
        font = None

    objects = parsed.get("objects")
    if not isinstance(objects, list):
        objects = []

    for obj in objects:
        if not isinstance(obj, dict):
            continue
        label = str(obj.get("label") or "unknown")
        conf = obj.get("confidence")
        bbox = obj.get("bbox")
        if not isinstance(bbox, list):
            continue

        bbox_px = normalize_bbox([float(x) for x in bbox], w, h)
        x1, y1, x2, y2 = bbox_px

        draw.rectangle([x1, y1, x2, y2], outline=(255, 0, 0), width=3)

        caption = label
        if isinstance(conf, (int, float)):
            caption = f"{label} ({conf:.2f})"

        # label background
        text_xy = (x1 + 2, max(0, y1 - 14))
        if font is not None:
            tw, th = draw.textbbox((0, 0), caption, font=font)[2:]
            draw.rectangle(
                [text_xy[0] - 2, text_xy[1] - 2, text_xy[0] + tw + 2, text_xy[1] + th + 2],
                fill=(255, 0, 0),
            )
            draw.text(text_xy, caption, fill=(255, 255, 255), font=font)
        else:
            draw.text(text_xy, caption, fill=(255, 255, 255))

    im.save(out_path)

def main() -> None:
    parser = argparse.ArgumentParser(description="VLM bbox detection + visualization (URL image)")
    parser.add_argument("--url", default=DEFAULT_IMAGE_URL, help="Image URL")
    parser.add_argument("--out", default=DEFAULT_OUT_PATH, help="Output image path")
    parser.add_argument("--max-objects", type=int, default=8, help="Max objects to request")
    args = parser.parse_args()

    if not _require_pillow():
        print("Missing dependency: pillow")
        print("Install with: pip install pillow")
        return

    print("=== VLM BBox Detection (URL) ===")
    print(f"URL: {args.url}")
    print(f"Output: {args.out}")

    image_bytes = download_image_bytes(args.url)
    w, h = get_image_size(image_bytes)

    parsed = request_bboxes(args.url, w, h, args.max_objects)
    print("=== Model JSON ===")
    print(json.dumps(parsed, indent=2, ensure_ascii=False))

    if parsed.get("parse_error"):
        print("Failed to parse JSON. Skip drawing.")
        return

    draw_bboxes(image_bytes, parsed, args.out)
    print(f"Saved: {args.out}")

if __name__ == "__main__":
    main()
```

- **동작 흐름**
    1. 이미지 URL 다운로드 → (width, height) 파악
    2. VLM에게 "객체 목록 + bbox 좌표" JSON 요청
    3. 응답 JSON 파싱
    4. Pillow로 이미지에 bbox 그리기
    5. 결과 이미지 저장

- **실행 방법**
    
    ```bash
    # 의존성 설치
    pip install pillow
    
    # 기본 실행
    python3 vlm_bbox.py
    
    # URL 및 출력 파일 지정
    python3 vlm_bbox.py --url "https://example.com/image.jpg" --out result.png
    
    # 최대 객체 수 제한
    python3 vlm_bbox.py --max-objects 5
    ```
    
- **참고 사항**
    - VLM의 bbox 추정은 정확도가 제한적
    - VLM은 전용 객체 탐지 모델(YOLO, Faster R-CNN)이 아님
    - 대략적인 위치 추정용으로만 사용
    - 정밀한 객체 탐지가 필요하면 전용 모델 사용 권장

![](https://upload.wikimedia.org/wikipedia/commons/thumb/1/14/Seals%40melb_zoo.jpg/960px-Seals%40melb_zoo.jpg)

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-50-45.png)

![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-51-12.png)

## 실습 3: VLM → 로봇 행동 추출 (vlm_robot_action.py)

> 이미지를 분석하여 구조화된 JSON으로 로봇 행동을 추출
> 

```python
import json
import argparse
from typing import Any, Dict

from common import (
    chat_complete,
    default_generation_params,
    get_client,
    load_config,
)

DEFAULT_IMAGE_URL = "https://images.pexels.com/photos/2103828/pexels-photo-2103828.jpeg"

def extract_robot_action_from_url(image_url: str) -> Dict[str, Any]:
    """URL 이미지를 분석하여 로봇이 취해야 할 행동을 JSON으로 반환"""
    cfg = load_config()
    client = get_client(cfg)

    system_rule = """
너는 모바일 로봇의 행동 결정 AI다.
이미지를 분석하여 로봇이 취해야 할 행동을 JSON 형식으로 출력한다.

반드시 아래 JSON 스키마를 따라 출력하라:
{
  "scene_summary": "장면 요약 (1문장)",
  "detected_objects": [
    {"name": "객체명", "position": "left|center|right", "distance": "near|medium|far", "risk_level": "low|medium|high"}
  ],
  "recommended_action": {
    "type": "move_forward|turn_left|turn_right|stop|slow_down|avoid_obstacle",
    "speed": 0.0~1.0,
    "reason": "행동 이유"
  },
  "safety_warning": "안전 경고 메시지 (없으면 null)"
}

JSON만 출력하고 다른 텍스트는 출력하지 마라.
""".strip()

    user_content = [
        {
            "type": "text",
            "text": "이 이미지를 분석하여 로봇 행동을 JSON으로 출력해줘.",
        },
        {
            "type": "image_url",
            "image_url": {"url": image_url},
        },
    ]

    messages = [
        {"role": "system", "content": system_rule},
        {"role": "user", "content": user_content},
    ]

    params = default_generation_params(cfg)
    params["temperature"] = 0.1  # JSON 출력은 낮은 temperature
    params["max_tokens"] = 1024

    response = chat_complete(
        client=client,
        model=cfg["vlm_model"],
        messages=messages,
        **params,
    )

    # JSON 파싱 시도
    try:
        # 마크다운 코드블록 제거
        clean_response = response.strip()
        if clean_response.startswith("```"):
            lines = clean_response.split("\n")
            clean_response = "\n".join(lines[1:-1])
        return json.loads(clean_response)
    except json.JSONDecodeError:
        return {"raw_response": response, "parse_error": "Failed to parse JSON"}

def execute_action(action: Dict[str, Any]) -> None:
    """추출된 행동을 실행 (시뮬레이션)"""
    if "parse_error" in action:
        print(f"[ERROR] {action['parse_error']}")
        print(f"[RAW] {action.get('raw_response', 'N/A')}")
        return

    print(f"[SCENE] {action.get('scene_summary', 'N/A')}")
    print()

    print("[DETECTED OBJECTS]")
    for obj in action.get("detected_objects", []):
        print(f"  - {obj.get('name')}: {obj.get('position')}, {obj.get('distance')}, risk={obj.get('risk_level')}")
    print()

    rec = action.get("recommended_action", {})
    print("[RECOMMENDED ACTION]")
    print(f"  Type: {rec.get('type', 'N/A')}")
    print(f"  Speed: {rec.get('speed', 'N/A')}")
    print(f"  Reason: {rec.get('reason', 'N/A')}")
    print()

    warning = action.get("safety_warning")
    if warning:
        print(f"[SAFETY WARNING] {warning}")

def main() -> None:
    parser = argparse.ArgumentParser(description="VLM robot action extraction (URL image)")
    parser.add_argument(
        "--url",
        default=DEFAULT_IMAGE_URL,
        help="Image URL to analyze",
    )
    args = parser.parse_args()

    print("=== VLM Robot Action Extraction (URL) ===")
    print(f"URL: {args.url}\n")

    action = extract_robot_action_from_url(args.url)

    print("=== Extracted Action (JSON) ===")
    print(json.dumps(action, indent=2, ensure_ascii=False))
    print()

    print("=== Action Execution (Simulation) ===")
    execute_action(action)

if __name__ == "__main__":
    main()
```

- **실행 방법**
    
    ```bash
    # 기본 실행
    python3 vlm_robot_action.py
    
    # URL 지정
    python3 vlm_robot_action.py --url "https://example.com/corridor.jpg"
    ```

![](https://images.pexels.com/photos/2103828/pexels-photo-2103828.jpeg)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-54-14.png)

## 실습 4: VLA 스타일 파이프라인 (vla_style_pipeline.py)

> Vision → Language → Action 3단계 파이프라인을 소프트웨어로 구현
> 

<aside>
💡

⚠️ **이 예제는 End-to-End VLA가 아닙니다!**
실제 VLA 모델(RT-2, OpenVLA)은 하나의 신경망에서 이미지를 보고 바로 로봇 제어 신호를 출력합니다. 본 실습은 **기존 VLM + LLM을 연결**하여 VLA의 동작 원리를 이해하기 위한 간단한 실습 입니다.

</aside>

### 3단계 파이프라인

| 단계 | 모듈 | 입력 | 출력 | 사용 모델 |
| --- | --- | --- | --- | --- |
| **1. Perceive** | Vision | 이미지 URL | 장면 JSON | VLM |
| **2. Reason** | Language | 장면 JSON + 태스크 | 행동 계획 JSON | LLM |
| **3. Act** | Action | 행동 계획 | RobotCommand | 규칙 기반 코드 |

```python
import json
import argparse
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from common import (
    chat_complete,
    default_generation_params,
    get_client,
    load_config,
)

DEFAULT_IMAGE_URL = "https://csossihettpx2597658.cdn.gov-ntruss.com/data2/content/image/2020/12/08/.cache/512/20201208254670.jpg"
DEFAULT_TASK = "전방의 장애물을 피해 안전하게 주행"

@dataclass
class RobotCommand:
    """로봇 제어 명령 구조체"""
    linear_x: float = 0.0   # 전진/후진 속도 (m/s)
    linear_y: float = 0.0   # 좌우 이동 속도 (m/s) - 메카넘휠용
    angular_z: float = 0.0  # 회전 속도 (rad/s)
    duration: float = 1.0   # 명령 지속 시간 (초)
    gripper: Optional[str] = None  # 그리퍼 명령: "open", "close", None

class VLASimulator:
    """VLA 파이프라인 시뮬레이터"""

    def __init__(self):
        self.cfg = load_config()
        self.client = get_client(self.cfg)
        self.action_history: List[Dict[str, Any]] = []

    def perceive(self, image_url: str) -> Dict[str, Any]:
        """1단계: 이미지 인식 (Vision)"""
        system_rule = """
너는 모바일 로봇의 비전 시스템이다.
이미지를 분석하여 로봇 행동에 필요한 정보를 JSON으로 출력한다.

출력 JSON 스키마:
{
  "scene_type": "indoor_corridor|indoor_room|outdoor|elevator|stairs|unknown",
  "objects": [
    {"label": "객체명", "bbox_center": "left|center|right", "depth": "near|medium|far"}
  ],
  "floor_condition": "clear|obstacle|uneven|wet",
  "lighting": "bright|normal|dark",
  "navigation_hint": "주행 힌트 1문장"
}

JSON만 출력하라.
""".strip()

        user_content = [
            {"type": "text", "text": "이미지를 분석하여 JSON으로 출력해줘."},
            {"type": "image_url", "image_url": {"url": image_url}},
        ]

        params = default_generation_params(self.cfg)
        params["temperature"] = 0.1
        params["max_tokens"] = 1024

        response = chat_complete(
            client=self.client,
            model=self.cfg["vlm_model"],
            messages=[
                {"role": "system", "content": system_rule},
                {"role": "user", "content": user_content},
            ],
            **params,
        )

        return self._parse_json(response)

    def reason(self, perception: Dict[str, Any], task: str) -> Dict[str, Any]:
        """2단계: 상황 판단 및 행동 결정 (Language)"""
        system_rule = """
너는 모바일 로봇의 행동 계획 AI다.
인식 결과와 사용자 태스크를 받아 행동을 결정한다.

출력 JSON 스키마:
{
  "task_understanding": "태스크 해석",
  "situation_assessment": "현재 상황 평가",
  "action_plan": [
    {"step": 1, "action": "행동 설명", "type": "move|turn|stop|wait|interact"}
  ],
  "selected_action": {
    "type": "forward|backward|left|right|rotate_left|rotate_right|stop",
    "speed_level": "slow|normal|fast",
    "confidence": 0.0~1.0
  },
  "abort_reason": null 또는 "중단 이유"
}

JSON만 출력하라.
""".strip()

        user_prompt = f"""
[인식 결과]
{json.dumps(perception, indent=2, ensure_ascii=False)}

[태스크]
{task}

위 정보를 바탕으로 다음 행동을 결정해줘.
""".strip()

        params = default_generation_params(self.cfg)
        params["temperature"] = 0.2
        params["max_tokens"] = 1024

        response = chat_complete(
            client=self.client,
            model=self.cfg["llm_model"],  # 텍스트 전용 모델 사용
            messages=[
                {"role": "system", "content": system_rule},
                {"role": "user", "content": user_prompt},
            ],
            **params,
        )

        return self._parse_json(response)

    def act(self, reasoning: Dict[str, Any]) -> RobotCommand:
        """3단계: 로봇 제어 명령 생성 (Action)"""
        selected = reasoning.get("selected_action", {})
        action_type = selected.get("type", "stop")
        speed_level = selected.get("speed_level", "slow")

        # 속도 매핑
        speed_map = {"slow": 0.1, "normal": 0.3, "fast": 0.5}
        base_speed = speed_map.get(speed_level, 0.1)

        # 행동 타입에 따른 명령 생성
        cmd = RobotCommand()

        if action_type == "forward":
            cmd.linear_x = base_speed
        elif action_type == "backward":
            cmd.linear_x = -base_speed
        elif action_type == "left":
            cmd.linear_y = base_speed  # 메카넘휠 좌측 이동
        elif action_type == "right":
            cmd.linear_y = -base_speed
        elif action_type == "rotate_left":
            cmd.angular_z = 0.5
        elif action_type == "rotate_right":
            cmd.angular_z = -0.5
        elif action_type == "stop":
            pass  # 모든 값 0

        return cmd

    def run_pipeline(self, image_url: str, task: str) -> Dict[str, Any]:
        """VLA 파이프라인 전체 실행"""
        result = {
            "image": image_url,
            "task": task,
            "stages": {},
        }

        # 1. Vision
        print("[Stage 1/3] Perceiving...")
        perception = self.perceive(image_url)
        result["stages"]["perception"] = perception
        print(f"  Scene: {perception.get('scene_type', 'unknown')}")
        print(f"  Objects: {len(perception.get('objects', []))} detected")

        # 2. Language (Reasoning)
        print("[Stage 2/3] Reasoning...")
        reasoning = self.reason(perception, task)
        result["stages"]["reasoning"] = reasoning
        print(f"  Assessment: {reasoning.get('situation_assessment', 'N/A')[:50]}...")

        # 중단 조건 확인
        if reasoning.get("abort_reason"):
            print(f"  [ABORT] {reasoning['abort_reason']}")
            result["aborted"] = True
            result["command"] = None
            return result

        # 3. Action
        print("[Stage 3/3] Generating command...")
        command = self.act(reasoning)
        result["stages"]["command"] = {
            "linear_x": command.linear_x,
            "linear_y": command.linear_y,
            "angular_z": command.angular_z,
            "duration": command.duration,
        }
        print(f"  Command: lin=({command.linear_x:.2f}, {command.linear_y:.2f}), ang={command.angular_z:.2f}")

        self.action_history.append(result)
        return result

    def _parse_json(self, response: str) -> Dict[str, Any]:
        """응답에서 JSON 파싱"""
        try:
            clean = response.strip()
            if clean.startswith("```"):
                lines = clean.split("\n")
                clean = "\n".join(lines[1:-1])
            return json.loads(clean)
        except json.JSONDecodeError:
            return {"raw_response": response, "parse_error": True}

def main() -> None:
    parser = argparse.ArgumentParser(description="VLA simulation (URL image)")
    parser.add_argument(
        "--url",
        default=DEFAULT_IMAGE_URL,
        help="Image URL to use as observation",
    )
    parser.add_argument(
        "--task",
        default=DEFAULT_TASK,
        help="High-level navigation task",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("VLA (Vision-Language-Action) Simulation")
    print("=" * 60)
    print(f"Image URL: {args.url}")
    print(f"Task: {args.task}")
    print("=" * 60)
    print()

    simulator = VLASimulator()
    result = simulator.run_pipeline(args.url, args.task)

    print()
    print("=" * 60)
    print("Full Result (JSON)")
    print("=" * 60)
    print(json.dumps(result, indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
```

- 실행 방법
    
    ```bash
    # 기본 실행
    python3 vla_style_pipeline.py
    # URL 및 태스크 지정
    python3 vla_style_pipeline.py --url "https://example.com/image.jpg" --task "문 앞까지 이동"
    ```
![](https://csossihettpx2597658.cdn.gov-ntruss.com/data2/content/image/2020/12/08/.cache/512/20201208254670.jpg)
![](/assets/img/HiWonderPi/img/Screenshot%20from%202025-12-30%2016-57-38.png)