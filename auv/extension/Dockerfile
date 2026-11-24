# Build from the repo root:  docker buildx build --platform linux/arm/v7 -f extension/Dockerfile .
# Base choice is an assumption: python:3.9-slim-bullseye ships armv7 images; numpy wheels for
# armv7 come from piwheels (extra index below), otherwise it compiles (slow, needs gcc).
FROM python:3.9-slim-bullseye

WORKDIR /app
RUN apt-get update && apt-get install -y --no-install-recommends gcc libc6-dev libatlas3-base \
    && apt-get clean && rm -r /var/lib/apt/lists/*
COPY extension/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --extra-index-url https://www.piwheels.org/simple -r /tmp/requirements.txt
COPY auv/ /app/auv/
COPY gui/ /app/gui/
COPY extension/app/ /app/app/
COPY extension/register_service /app/register_service
RUN mkdir -p /data/logs

ENV AUV_LOG_DIR=/data/logs AUV_MAVLINK=udpin:0.0.0.0:14550
EXPOSE 8080

# Labels follow the BlueOS extension convention as remembered. UNVERIFIED: check the BlueOS
# docs (Extensions -> creating an extension) for current label names and the permissions JSON.
LABEL version="0.1.0"
LABEL permissions='{"ExposedPorts":{"8080/tcp":{}},"HostConfig":{"Binds":["/root/.config/auv:/data:rw"],"NetworkMode":"host"}}'
LABEL authors='[{"name":"AUV team","email":"unset@example.com"}]'
LABEL company='{"about":"AUV outer-loop navigation","name":"AUV","email":"unset@example.com"}'
LABEL type="other"
LABEL readme="https://raw.githubusercontent.com/OWNER/REPO/main/extension/README.md"
LABEL links='{"website":"https://example.com","support":"https://example.com"}'
LABEL requirements="core >= 1.1"

# One worker: AuvCore owns the link and the control thread; more workers would duplicate it.
CMD ["python", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080", "--workers", "1"]
