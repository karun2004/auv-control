import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "extension"))

import pytest
from fastapi.testclient import TestClient

from app.main import create_app
from auv.core import AuvCore, CsvLogger
from auv.mavlink_io import FakeLink

TOKEN = "s3cret"
MISSION = {"name": "w", "waypoints": [{"n": 10, "e": 5, "depth": 2}]}


@pytest.fixture
def ctx(tmp_path):
    link = FakeLink()
    core = AuvCore(link, log_dir=str(tmp_path), source="fake")
    link.connect()
    core.step(0.0)
    (tmp_path / "auv_test.csv").write_text("# x\nt_ns\n1\n")
    return TestClient(create_app(core, token=TOKEN, gui_dir=str(tmp_path / "nogui"))), core, link


def test_status_and_telemetry(ctx):
    c, core, _ = ctx
    r = c.get("/status"); assert r.status_code == 200 and r.json()["running"] is True
    t = c.get("/telemetry").json()
    assert "nav" in t and t["battery_v"] is None          # NaN -> null, not 0


def test_post_requires_token(ctx):
    c, core, link = ctx
    assert c.post("/mission", json=MISSION).status_code == 401
    assert c.post("/mission", json=MISSION, headers={"X-Auth-Token": "bad"}).status_code == 401
    assert c.post("/abort").status_code == 401
    assert c.post("/abort", headers={"X-Auth-Token": "bad"}).status_code == 401
    assert core.mm.waypoints == [] and not core._cmds


def test_unset_token_refuses_all(tmp_path):
    core = AuvCore(FakeLink(), log_dir=str(tmp_path))
    c = TestClient(create_app(core, token="", gui_dir=str(tmp_path / "x")))
    assert c.post("/abort", headers={"X-Auth-Token": ""}).status_code == 401


def test_mission_upload_and_validation(ctx):
    c, core, link = ctx
    h = {"X-Auth-Token": TOKEN}
    r = c.post("/mission", json=MISSION, headers=h)
    assert r.status_code == 200 and r.json()["n_waypoints"] == 1
    core.step(0.05)
    assert len(core.mm.waypoints) == 1
    bad = c.post("/mission", json={"waypoints": [{"n": 1, "e": 1, "depth": 500}]}, headers=h)
    assert bad.status_code == 422 and "depth" in bad.json()["detail"]
    assert c.post("/mission", json={}, headers=h).status_code == 422


def test_abort_queues_and_core_surfaces(ctx):
    c, core, link = ctx
    assert c.post("/abort", headers={"X-Auth-Token": TOKEN}).status_code == 200
    core.step(0.1)
    assert "surface" in link.kinds()


def test_logs_list_download_and_traversal(ctx):
    c, core, _ = ctx
    names = [f["name"] for f in c.get("/logs").json()["files"]]
    assert "auv_test.csv" in names
    assert c.get("/logs/auv_test.csv").text.startswith("# x")
    assert c.get("/logs/..%2Fsecret").status_code == 404
    assert c.get("/logs/nonexistent.csv").status_code == 404


def test_static_gui_mounted(tmp_path):
    g = tmp_path / "gui"; g.mkdir(); (g / "index.html").write_text("<h1>hi</h1>")
    core = AuvCore(FakeLink(), log_dir=str(tmp_path))
    c = TestClient(create_app(core, token=TOKEN, gui_dir=str(g)))
    assert "hi" in c.get("/").text
    assert c.get("/status").status_code == 200       # api routes win over the mount
