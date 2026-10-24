FROM python:3.12-slim
WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY aegis ./aegis
COPY web ./web
COPY server.py .
RUN useradd --create-home appuser
USER appuser
EXPOSE 8000
# PORT is honoured for hosts such as Render / Railway / Fly.io that inject it.
CMD ["sh", "-c", "uvicorn server:app --host 0.0.0.0 --port ${PORT:-8000}"]
