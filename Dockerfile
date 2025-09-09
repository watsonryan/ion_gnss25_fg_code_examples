# syntax=docker/dockerfile:1.6
ARG PYTHON_VERSION=3.10
FROM --platform=$TARGETPLATFORM python:${PYTHON_VERSION}-slim AS base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends     graphviz     tini     build-essential     git     ca-certificates  && rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE=1     PYTHONUNBUFFERED=1     MPLBACKEND=Agg     PIP_NO_CACHE_DIR=1     LC_ALL=C.UTF-8     LANG=C.UTF-8     PYTHONPATH=/app

WORKDIR /app
COPY requirements.txt /app/requirements.txt
RUN python -m pip install --upgrade pip &&     pip install -r /app/requirements.txt &&     pip install graphviz

RUN python - <<'PY'
import sys
print("Python:", sys.version)
try:
    import gtsam
    print("GTSAM import OK")
except Exception as e:
    print("GTSAM import failed:", e)
    raise
PY

COPY . /app

ARG APP_UID=1000
ARG APP_GID=1000
RUN groupadd -g $APP_GID app || true && useradd -m -u $APP_UID -g $APP_GID app || true &&     chown -R app:app /app
USER app

ENTRYPOINT ["tini","--"]
CMD ["bash"]
