---
title: "LangChain으로 AI 에이전트 만들기: 실습"
date: 2025-09-05 19:30:00 +0900
categories: [AI, LangChain, 에이전트]
tags: [LangChain, AI에이전트, 오케스트레이션, 자동화]
---

# LangChain으로 AI 에이전트 만들기: 완전 실습

LangChain은 AI 애플리케이션 개발을 위한 강력한 프레임워크입니다. 이 글에서는 LangChain을 활용하여 지능적인 AI 에이전트를 만드는 방법을 단계별로 알아보겠습니다.

## 🤔 LangChain이란?

**LangChain**은 대규모 언어 모델(LLM)을 활용한 애플리케이션 개발을 위한 오픈소스 프레임워크입니다.

### 핵심 특징

- **모듈화된 구성 요소**: 체인, 에이전트, 메모리 등 재사용 가능한 컴포넌트
- **다양한 LLM 지원**: OpenAI, Anthropic, Hugging Face 등
- **외부 도구 연동**: 웹 검색, 데이터베이스, API 등
- **메모리 관리**: 대화 기록 및 컨텍스트 유지

## 🏗️ LangChain 핵심 구성 요소

### 1. LLM (Large Language Model)

```python
from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI

# 기본 LLM
llm = OpenAI(temperature=0.7)

# 채팅 모델
chat_model = ChatOpenAI(
    model_name="gpt-3.5-turbo",
    temperature=0.7
)
```

### 2. 프롬프트 템플릿

```python
from langchain.prompts import PromptTemplate

# 프롬프트 템플릿 생성
template = """
당신은 {role}입니다.
다음 질문에 대해 {tone}으로 답변해주세요:

질문: {question}
답변:
"""

prompt = PromptTemplate(
    input_variables=["role", "tone", "question"],
    template=template
)

# 프롬프트 사용
formatted_prompt = prompt.format(
    role="AI 전문가",
    tone="친근하게",
    question="인공지능의 미래는 어떻게 될까요?"
)
```

### 3. 체인 (Chain)

```python
from langchain.chains import LLMChain

# LLM 체인 생성
chain = LLMChain(llm=llm, prompt=prompt)

# 체인 실행
result = chain.run(
    role="AI 전문가",
    tone="친근하게",
    question="인공지능의 미래는 어떻게 될까요?"
)
```

## 🤖 AI 에이전트 만들기

### 1. 기본 에이전트

```python
from langchain.agents import initialize_agent, Tool
from langchain.agents import AgentType
from langchain.llms import OpenAI

# 도구 정의
def search_tool(query: str) -> str:
    """웹 검색 도구"""
    # 실제 검색 로직 구현
    return f"'{query}'에 대한 검색 결과입니다."

def calculator_tool(expression: str) -> str:
    """계산기 도구"""
    try:
        result = eval(expression)
        return f"계산 결과: {result}"
    except:
        return "계산할 수 없는 식입니다."

# 도구 리스트
tools = [
    Tool(
        name="Search",
        func=search_tool,
        description="웹에서 정보를 검색할 때 사용합니다."
    ),
    Tool(
        name="Calculator",
        func=calculator_tool,
        description="수학 계산을 할 때 사용합니다."
    )
]

# 에이전트 초기화
agent = initialize_agent(
    tools=tools,
    llm=OpenAI(temperature=0),
    agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
    verbose=True
)

# 에이전트 실행
result = agent.run("2024년 인공지능 트렌드를 검색하고, 그 중에서 가장 중요한 3가지를 선정해주세요.")
```

### 2. 고급 에이전트: 메모리 기능

```python
from langchain.memory import ConversationBufferMemory
from langchain.agents import initialize_agent

# 메모리 설정
memory = ConversationBufferMemory(
    memory_key="chat_history",
    return_messages=True
)

# 메모리가 있는 에이전트
agent_with_memory = initialize_agent(
    tools=tools,
    llm=OpenAI(temperature=0),
    agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
    memory=memory,
    verbose=True
)

# 대화형 에이전트 사용
result1 = agent_with_memory.run("안녕하세요! 저는 AI에 관심이 있습니다.")
result2 = agent_with_memory.run("앞서 말씀하신 AI에 대해 더 자세히 알려주세요.")
```

## 🛠️ 실전 프로젝트: 개인 비서 에이전트

### 1. 프로젝트 구조

```
personal_assistant/
├── main.py
├── tools/
│   ├── weather.py
│   ├── calendar.py
│   └── email.py
├── agents/
│   └── assistant_agent.py
└── config.py
```

### 2. 도구 구현

```python
# tools/weather.py
import requests

def get_weather(city: str) -> str:
    """날씨 정보를 가져옵니다."""
    # 실제 API 호출 (예시)
    api_key = "your_api_key"
    url = f"http://api.openweathermap.org/data/2.5/weather?q={city}&appid={api_key}"
    
    try:
        response = requests.get(url)
        data = response.json()
        return f"{city}의 현재 날씨: {data['weather'][0]['description']}, 온도: {data['main']['temp']}°C"
    except:
        return f"{city}의 날씨 정보를 가져올 수 없습니다."

# tools/calendar.py
from datetime import datetime, timedelta

def get_schedule(date: str = None) -> str:
    """일정을 조회합니다."""
    if date is None:
        date = datetime.now().strftime("%Y-%m-%d")
    
    # 실제 일정 데이터베이스 조회 (예시)
    schedule = [
        {"time": "09:00", "title": "팀 미팅"},
        {"time": "14:00", "title": "프로젝트 리뷰"},
        {"time": "16:00", "title": "고객 상담"}
    ]
    
    result = f"{date} 일정:\n"
    for item in schedule:
        result += f"- {item['time']}: {item['title']}\n"
    
    return result

def add_schedule(time: str, title: str, date: str = None) -> str:
    """일정을 추가합니다."""
    if date is None:
        date = datetime.now().strftime("%Y-%m-%d")
    
    # 실제 일정 추가 로직
    return f"{date} {time}에 '{title}' 일정이 추가되었습니다."

# tools/email.py
def send_email(to: str, subject: str, body: str) -> str:
    """이메일을 발송합니다."""
    # 실제 이메일 발송 로직
    return f"{to}에게 '{subject}' 제목으로 이메일을 발송했습니다."
```

### 3. 에이전트 구현

```python
# agents/assistant_agent.py
from langchain.agents import initialize_agent, Tool
from langchain.agents import AgentType
from langchain.llms import OpenAI
from langchain.memory import ConversationBufferMemory
from tools.weather import get_weather
from tools.calendar import get_schedule, add_schedule
from tools.email import send_email

class PersonalAssistantAgent:
    def __init__(self):
        self.setup_tools()
        self.setup_memory()
        self.setup_agent()
    
    def setup_tools(self):
        """도구 설정"""
        self.tools = [
            Tool(
                name="Weather",
                func=get_weather,
                description="특정 도시의 날씨 정보를 조회합니다. 입력: 도시명"
            ),
            Tool(
                name="Schedule",
                func=get_schedule,
                description="일정을 조회합니다. 입력: 날짜 (YYYY-MM-DD 형식, 선택사항)"
            ),
            Tool(
                name="AddSchedule",
                func=add_schedule,
                description="일정을 추가합니다. 입력: 시간, 제목, 날짜 (선택사항)"
            ),
            Tool(
                name="Email",
                func=send_email,
                description="이메일을 발송합니다. 입력: 수신자, 제목, 내용"
            )
        ]
    
    def setup_memory(self):
        """메모리 설정"""
        self.memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True
        )
    
    def setup_agent(self):
        """에이전트 설정"""
        self.agent = initialize_agent(
            tools=self.tools,
            llm=OpenAI(temperature=0.7),
            agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
            memory=self.memory,
            verbose=True
        )
    
    def chat(self, message: str) -> str:
        """사용자와 대화"""
        return self.agent.run(message)

# 사용 예시
if __name__ == "__main__":
    assistant = PersonalAssistantAgent()
    
    # 대화 예시
    print(assistant.chat("안녕하세요! 오늘 서울 날씨가 어때요?"))
    print(assistant.chat("오늘 일정이 뭐가 있나요?"))
    print(assistant.chat("내일 오후 2시에 '프로젝트 미팅' 일정을 추가해주세요."))
```

## 🔧 고급 기능

### 1. 커스텀 도구 클래스

```python
from langchain.tools import BaseTool
from typing import Optional, Type
from pydantic import BaseModel, Field

class WeatherInput(BaseModel):
    city: str = Field(description="날씨를 조회할 도시명")

class WeatherTool(BaseTool):
    name = "weather_tool"
    description = "특정 도시의 날씨 정보를 조회합니다."
    args_schema: Type[BaseModel] = WeatherInput
    
    def _run(self, city: str) -> str:
        return get_weather(city)
    
    def _arun(self, city: str) -> str:
        raise NotImplementedError("비동기 실행은 지원하지 않습니다.")

# 커스텀 도구 사용
custom_tools = [WeatherTool()]
```

### 2. 에이전트 체인 연결

```python
from langchain.chains import SimpleSequentialChain
from langchain.chains import LLMChain

# 첫 번째 체인: 질문 분석
analysis_prompt = PromptTemplate(
    input_variables=["question"],
    template="다음 질문을 분석하고 필요한 도구를 결정하세요: {question}"
)
analysis_chain = LLMChain(llm=llm, prompt=analysis_prompt)

# 두 번째 체인: 답변 생성
answer_prompt = PromptTemplate(
    input_variables=["analysis", "question"],
    template="분석 결과: {analysis}\n원래 질문: {question}\n답변:"
)
answer_chain = LLMChain(llm=llm, prompt=answer_prompt)

# 체인 연결
overall_chain = SimpleSequentialChain(
    chains=[analysis_chain, answer_chain],
    verbose=True
)

result = overall_chain.run("오늘 날씨가 어떤가요?")
```

### 3. 스트리밍 응답

```python
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler

# 스트리밍 LLM
streaming_llm = OpenAI(
    streaming=True,
    callbacks=[StreamingStdOutCallbackHandler()],
    temperature=0.7
)

# 스트리밍 에이전트
streaming_agent = initialize_agent(
    tools=tools,
    llm=streaming_llm,
    agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
    verbose=True
)

# 실시간 응답
result = streaming_agent.run("복잡한 수학 문제를 풀어주세요.")
```

## 📊 성능 최적화

### 1. 캐싱 활용

```python
from langchain.cache import InMemoryCache
from langchain.globals import set_llm_cache

# 메모리 캐시 설정
set_llm_cache(InMemoryCache())

# 동일한 질문에 대해 캐시된 결과 반환
result1 = agent.run("인공지능이란 무엇인가요?")
result2 = agent.run("인공지능이란 무엇인가요?")  # 캐시에서 반환
```

### 2. 비동기 처리

```python
import asyncio
from langchain.agents import initialize_agent

async def async_agent_run(agent, query):
    """비동기 에이전트 실행"""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, agent.run, query)

# 여러 질문 동시 처리
queries = [
    "오늘 날씨는?",
    "내일 일정은?",
    "이번 주 계획은?"
]

async def process_queries():
    tasks = [async_agent_run(agent, query) for query in queries]
    results = await asyncio.gather(*tasks)
    return results

# 실행
results = asyncio.run(process_queries())
```

## 🎯 실제 활용 사례

### 1. 고객 서비스 봇

```python
class CustomerServiceBot:
    def __init__(self):
        self.tools = [
            Tool(
                name="OrderLookup",
                func=self.lookup_order,
                description="주문 번호로 주문 정보를 조회합니다."
            ),
            Tool(
                name="RefundProcess",
                func=self.process_refund,
                description="환불 처리를 진행합니다."
            ),
            Tool(
                name="ProductInfo",
                func=self.get_product_info,
                description="제품 정보를 조회합니다."
            )
        ]
        
        self.agent = initialize_agent(
            tools=self.tools,
            llm=OpenAI(temperature=0.3),
            agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
            verbose=True
        )
    
    def handle_customer_query(self, query: str) -> str:
        return self.agent.run(query)
```

### 2. 데이터 분석 에이전트

```python
class DataAnalysisAgent:
    def __init__(self):
        self.tools = [
            Tool(
                name="LoadData",
                func=self.load_data,
                description="데이터를 로드합니다."
            ),
            Tool(
                name="AnalyzeData",
                func=self.analyze_data,
                description="데이터를 분석합니다."
            ),
            Tool(
                name="CreateVisualization",
                func=self.create_visualization,
                description="시각화를 생성합니다."
            )
        ]
        
        self.agent = initialize_agent(
            tools=self.tools,
            llm=OpenAI(temperature=0.1),
            agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
            verbose=True
        )
    
    def analyze(self, request: str) -> str:
        return self.agent.run(request)
```

## 🚀 배포 및 운영

### 1. FastAPI와 연동

```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

class ChatRequest(BaseModel):
    message: str

class ChatResponse(BaseModel):
    response: str

# 에이전트 인스턴스
assistant = PersonalAssistantAgent()

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    response = assistant.chat(request.message)
    return ChatResponse(response=response)

@app.get("/health")
async def health_check():
    return {"status": "healthy"}
```

### 2. 모니터링 및 로깅

```python
import logging
from langchain.callbacks import BaseCallbackHandler

class LoggingCallbackHandler(BaseCallbackHandler):
    def on_agent_action(self, action, **kwargs):
        logging.info(f"Agent Action: {action}")
    
    def on_agent_finish(self, finish, **kwargs):
        logging.info(f"Agent Finish: {finish}")

# 로깅 설정
logging.basicConfig(level=logging.INFO)

# 에이전트에 콜백 추가
agent = initialize_agent(
    tools=tools,
    llm=OpenAI(temperature=0),
    agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
    callbacks=[LoggingCallbackHandler()],
    verbose=True
)
```

## 🎯 마무리

LangChain을 활용하면 복잡한 AI 에이전트를 쉽게 구축할 수 있습니다. 

### 핵심 포인트

1. **모듈화**: 도구, 체인, 메모리 등을 독립적으로 구성
2. **확장성**: 새로운 도구와 기능을 쉽게 추가
3. **유연성**: 다양한 LLM과 도구 조합 가능
4. **실용성**: 실제 비즈니스 문제 해결에 활용 가능

### 다음 단계

- **LangGraph**: 복잡한 워크플로우 관리
- **LangServe**: 에이전트 API 서버 구축
- **LangSmith**: 에이전트 성능 모니터링

LangChain으로 더욱 지능적이고 유용한 AI 에이전트를 만들어보세요!

---

**참고 자료:**
- [LangChain 공식 문서](https://python.langchain.com/)
- [LangChain GitHub](https://github.com/langchain-ai/langchain)
- [LangChain 예제 모음](https://github.com/langchain-ai/langchain/tree/master/templates)

*이 글이 도움이 되셨다면 댓글로 피드백을 남겨주세요! 🚀*
