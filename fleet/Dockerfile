FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
RUN pip install --no-cache-dir -e .

# Verify installation
RUN python debug/iogita_diagnostics.py

# Default: run fleet debug node in REST polling mode
ENV FMS_HOST=127.0.0.1
ENV FMS_REST_PORT=7012
ENV FMS_TCP_PORT=65123

CMD ["python", "debug/fleet_debug_node.py", "--mode", "rest", "--host", "${FMS_HOST}", "--port", "${FMS_REST_PORT}"]
