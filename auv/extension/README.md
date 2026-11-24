# AUV BlueOS extension

Runs `auv.core.AuvCore` (20 Hz outer loops) plus a FastAPI server on port 8080 inside a BlueOS
extension container. Target: Raspberry Pi 4B, 32-bit BlueOS (armv7, Debian bullseye).

Items marked **ASSUMED** come from memory and were not verified against current BlueOS docs.
Check them before first use.

## Routes
| Route | Auth | Purpose |
|---|---|---|
| GET /status | none | state, failsafe level, overruns |
| GET /telemetry | none | full snapshot from the control thread |
| POST /mission | `X-Auth-Token` | JSON mission (n/e or lat/lon+origin, depth in (0, 12] m, <=50 waypoints) |
| POST /abort | `X-Auth-Token` | request SURFACE and abort the mission |
| GET /logs, GET /logs/{name} | none | CSV logs |
| GET /register_service | none | sidebar registration JSON (**ASSUMED** BlueOS convention) |
| / | none | static GUI (`gui/`) if present |

Set `AUV_TOKEN` (container env). Unset means every POST is refused.

## Config (env)
- `AUV_MAVLINK`: pymavlink endpoint, default `udpin:0.0.0.0:14550`. **ASSUMED**: BlueOS
  mavlink-router can forward to a UDP endpoint on 14550 and, with host networking, the container
  can bind it. If not, add a UDP endpoint in BlueOS (MAVLink endpoints page) pointing at the
  container, or use `tcp:127.0.0.1:<port>` if your router exposes TCP. mavlink2rest
  (`http://host.docker.internal/mavlink2rest`) is HTTP/JSON, a different transport, not implemented.
- `AUV_LINK=fake`: run without hardware.
- `AUV_THRUST_MODE`: `direct` or `centered`; see `auv/mavlink_io.py` assumption A1. ArduSub thrust
  semantics are unverified: bench-test with props off.
- `AUV_LOG_DIR`: default `/data/logs`, bound to `/root/.config/auv` on the Pi (**ASSUMED** persistent path).

## Local test
    .venv/bin/python -m pytest tests/test_web.py
    AUV_LINK=fake AUV_TOKEN=x AUV_LOG_DIR=/tmp/auvlogs PYTHONPATH=extension:. \
        .venv/bin/python -m uvicorn app.main:app --port 8080

## Build and push (run from the repo root, not extension/)
    docker buildx create --use --name auvbuilder     # once
    docker login
    docker buildx build --platform linux/arm/v7 -f extension/Dockerfile \
        -t DOCKERUSER/auv-outer-loop:0.1.0 --push .

## Install in BlueOS
1. BlueOS web UI -> Extensions -> Installed -> `+` (create).
2. Name `AUV Outer Loop`, image `DOCKERUSER/auv-outer-loop`, tag `0.1.0`.
3. Original settings (**ASSUMED** format):
   `{"ExposedPorts":{"8080/tcp":{}},"HostConfig":{"Binds":["/root/.config/auv:/data:rw"],"NetworkMode":"host"}}`
   Add env `AUV_TOKEN=<secret>`.
4. Save, wait for the image pull, open from the sidebar or `http://BLUEOS_IP:8080`.
5. Confirm `/status` shows `running: true` and `/telemetry` has live `att`.
   Do the first arm/GUIDED test props off.
