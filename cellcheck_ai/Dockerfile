# ✅ Python 3.9.13 slim 이미지 사용
FROM python:3.9.13-slim

# 필수 시스템 라이브러리 설치 (libGL 포함)
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ✅ 작업 디렉토리 생성
WORKDIR /app

# ✅ 의존성 파일 먼저 복사
COPY requirements.txt .

# ✅ pip 업그레이드 및 의존성 설치
RUN pip install --upgrade pip \
    && pip install --no-cache-dir -r requirements.txt

# ✅ 전체 코드 복사
COPY . .

# ✅ 외부 포트 오픈 (WebSocket용 8000)
EXPOSE 8000

# ✅ WebSocket 서버 실행
CMD ["python", "run_server.py"]
