"""BlueOS extension web API. The control loop runs in AuvCore's own thread; handlers here only
read AuvCore.snapshot() or enqueue commands, so a slow client cannot stall control.

Env config:
  AUV_TOKEN          shared secret for POST routes (header X-Auth-Token). Unset => POSTs refused.
  AUV_MAVLINK        pymavlink endpoint, default "udpin:0.0.0.0:14550"  (ASSUMPTION, see README)
  AUV_LINK           "mavlink" (default) or "fake" (no hardware, for smoke tests)
  AUV_LOG_DIR        default /data/logs   (bind-mounted from the Pi, see README)
  AUV_GUI_DIR        static GUI dir; default: first existing of ./gui, ../../gui
  AUV_THRUST_MODE    "direct" | "centered" (see auv/mavlink_io.py assumption A1)
"""
from __future__ import annotations
import hmac
from contextlib import asynccontextmanager
import json
import os
from typing import Optional

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from auv.core import AuvCore

HERE = os.path.dirname(os.path.abspath(__file__))


def _gui_dir() -> Optional[str]:
    cands = [os.environ.get("AUV_GUI_DIR"),
             os.path.join(os.getcwd(), "gui"),
             os.path.join(HERE, "..", "..", "gui")]
    for c in cands:
        if c and os.path.isdir(c):
            return os.path.abspath(c)
    return None


def create_app(core: AuvCore, token: Optional[str] = None, gui_dir: Optional[str] = None,
               start_core: bool = False) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_app):
        if start_core:
            core.start()
        yield
        if start_core:
            core.stop()

    app = FastAPI(title="AUV outer loop", lifespan=lifespan)
    secret = token if token is not None else os.environ.get("AUV_TOKEN", "")

    def require_token(x_auth_token: str = Header(default="")):
        if not secret or not hmac.compare_digest(x_auth_token.encode(), secret.encode()):
            raise HTTPException(status_code=401, detail="bad or missing token")

    @app.get("/register_service")
    def register_service():
        # ASSUMED BlueOS convention: sidebar entry discovered via this JSON route.
        with open(os.path.join(HERE, "..", "register_service")) as fh:
            return json.load(fh)

    @app.get("/status")
    def status():
        s = core.snapshot()
        return {"running": s.get("running", False), "mission": s.get("mission"),
                "failsafe": s.get("failsafe"), "tick": s.get("tick"),
                "overruns": s.get("overruns"), "error": s.get("error"),
                "battery_v": s.get("battery_v"), "leak": s.get("leak")}

    @app.get("/telemetry")
    def telemetry():
        return core.snapshot()

    @app.post("/mission", dependencies=[Depends(require_token)])
    def mission(body: dict):
        try:
            clean = core.submit_mission(body)
        except ValueError as ex:
            raise HTTPException(status_code=422, detail=str(ex))
        return {"accepted": True, "name": clean["name"], "n_waypoints": len(clean["waypoints"])}

    @app.post("/abort", dependencies=[Depends(require_token)])
    def abort():
        core.abort()
        return {"abort": "queued"}

    @app.get("/logs")
    def logs():
        return {"files": [{"name": f, "bytes": os.path.getsize(os.path.join(core.log_dir, f))}
                          for f in core.log_files()]}

    @app.get("/logs/{name}")
    def log_file(name: str):
        if name not in core.log_files():          # whitelist: blocks any path traversal
            raise HTTPException(status_code=404, detail="no such log")
        return FileResponse(os.path.join(core.log_dir, name), media_type="text/csv",
                            filename=name)

    g = gui_dir if gui_dir is not None else _gui_dir()
    if g and os.path.isdir(g):
        app.mount("/", StaticFiles(directory=g, html=True), name="gui")   # mounted last
    return app


def build_default_app() -> FastAPI:
    from auv.mavlink_io import Link, FakeLink
    log_dir = os.environ.get("AUV_LOG_DIR", "/data/logs")
    if os.environ.get("AUV_LINK", "mavlink") == "fake":
        link, source = FakeLink(), "fake"
    else:
        link = Link(os.environ.get("AUV_MAVLINK", "udpin:0.0.0.0:14550"),
                    thrust_mode=os.environ.get("AUV_THRUST_MODE", "direct"))
        source = "real"
    return create_app(AuvCore(link, log_dir=log_dir, source=source), start_core=True)


def __getattr__(name):   # `uvicorn app.main:app` builds lazily; importing the module stays cheap
    if name == "app":
        a = build_default_app()
        globals()["app"] = a
        return a
    raise AttributeError(name)
