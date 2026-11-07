"""End-to-end tests of the MCP server against a fake agent bridge (no robot needed)."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import httpx
import pytest
from mcp.client import Client

from robot_mcp import server as srv
from robot_mcp.analysis import summarize_csv
from robot_mcp.client import RobotClient
from robot_mcp.schema import ConfigSchema

REPO = Path(__file__).resolve().parents[3]


class FakeBridge:
    """Just enough of AgentBridge's HTTP behaviour to drive the server's tools."""

    def __init__(self) -> None:
        self.state = "none"
        self.name = ""
        self.recording = False
        self.seq = 0
        self.calls: list[str] = []
        self.config: dict[str, object] = {"shooter.pidf.p": 0.001}
        self.gamepad: dict | None = None
        self.flight_recording = False
        self.flight_files: dict[str, bytes] = {}
        self.grow_on_read: list[bytes] = []  # appended to hardware.csv on each read, live-tail test
        self.statics: dict[str, object] = {"Tuner.targetRPM": 870.0, "Casablanca.frictionX": 0.1}
        # Like Casablanca's constructor: init copies config over this static.
        self.init_overwrites = {"Casablanca.frictionX": 0.1}

    def handle(self, request: httpx.Request) -> httpx.Response:
        path = request.url.path
        q = dict(request.url.params)
        self.calls.append(f"{request.method} {path}")

        def ok(body: object) -> httpx.Response:
            return httpx.Response(200, json=body)

        def conflict(msg: str) -> httpx.Response:
            return httpx.Response(409, json={"error": msg})

        if path == "/agent/status":
            return ok(
                {
                    "opmode": {"name": self.name, "state": self.state},
                    "telemetry_seq": self.seq,
                    "recorder": {"recording": self.recording},
                }
            )
        if path == "/agent/opmode/init":
            if self.state != "none":
                return conflict("active")
            self.state, self.name = "init", q["name"]
            self.statics.update(self.init_overwrites)
            return ok({})
        if path == "/agent/opmode/start":
            if self.state != "init":
                return conflict("no OpMode is in INIT")
            self.state = "running"
            return ok({"ok": True})
        if path == "/agent/opmode/stop":
            self.state = "none"
            return ok({"ok": True})
        if path == "/agent/gamepad":
            self.gamepad = json.loads(request.content)
            return ok({"active": True})
        if path == "/agent/telemetry":
            # Each poll while running produces one new update.
            if self.state == "running":
                self.seq += 1
            since = int(q.get("since", self.seq))
            entries = [
                {"seq": s, "tMs": s, "opMode": self.name, "lines": [{"caption": "vel", "value": str(s)}]}
                for s in range(since + 1, self.seq + 1)
            ]
            return ok({"latest_seq": self.seq, "entries": entries})
        if path == "/agent/record/start":
            self.recording = True
            return ok({"columns": ["shooter.vel"], "devices": [{"name": "shooter"}], "skipped": []})
        if path == "/agent/record/stop":
            self.recording = False
            return ok({"recording": False, "rows": 3})
        if path == "/agent/record/csv":
            return httpx.Response(
                200, text="t_ms,shooter.vel,shooter.power\n0,0,0.5\n20,800,0.5\n40,1400,0.5\n"
            )
        if path == "/agent/configurables":
            return ok([{"key": k, "value": v} for k, v in self.statics.items()])
        if path == "/agent/configurable":
            key = q["key"]
            if key not in self.statics:
                return httpx.Response(400, json={"error": f"no configurable '{key}'"})
            if request.method == "POST":
                before = self.statics[key]
                self.statics[key] = json.loads(q["value"])
                return ok({"key": key, "before": before, "after": self.statics[key]})
            return ok({"key": key, "value": self.statics[key]})
        if path == "/agent/flight/start":
            if self.flight_recording:
                return conflict("flight is recording")
            self.flight_recording = True
            self.flight_files = {
                "hardware.csv": b"t_ms,loop_seq,shooter.vel\n0,0,1000\n5,1,1100\n",
                "events.jsonl": b'{"event":"opmode_init"}\n',
                "meta.json": b'{"id":"F1","label":"x","rows":2,"columns":["shooter.vel"]}',
            }
            return ok({"id": "F1", "label": q.get("label", ""), "columns": ["shooter.vel"]})
        if path == "/agent/flight/stop":
            self.flight_recording = False
            return ok({"id": "F1", "label": "x", "rows": 2, "recording": False})
        if path == "/agent/flights":
            return ok([{"id": "F1", "meta": {"rows": 2}}])
        if path == "/agent/flight/file":
            name, offset = q["name"], int(q.get("offset", 0))
            if name not in self.flight_files:
                return httpx.Response(400, json={"error": f"no {name} in flight F1"})
            if name == "hardware.csv" and offset > 0 and self.grow_on_read:
                self.flight_files[name] += self.grow_on_read.pop(0)
            data = self.flight_files[name]
            chunk = data[offset : offset + 16]  # small chunks: exercise the offset loop
            return httpx.Response(
                200,
                content=chunk,
                headers={"X-Next-Offset": str(offset + len(chunk)), "X-File-Size": str(len(data))},
            )
        if path == "/agent/config":
            if request.method == "POST":
                self.config[q["key"]] = json.loads(q["value"])
                return ok({"path": q["key"], "after": self.config[q["key"]]})
            return ok({"key": q["key"], "value": self.config.get(q["key"])})
        return httpx.Response(404, json={"error": "not found"})


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    fake = FakeBridge()
    monkeypatch.setattr(
        srv, "_client", RobotClient("http://robot", transport=httpx.MockTransport(fake.handle))
    )
    monkeypatch.setattr(srv, "RUNS_DIR", tmp_path / "runs")
    monkeypatch.setattr(srv, "_sleep_polling", _fast_sleep)
    return fake


async def _fast_sleep(r, seconds, pull):
    await pull()


class ToolFailed(Exception):
    """The tool returned an error result; the message is the text the agent sees."""


def call(tool: str, args: dict) -> dict:
    async def go():
        async with Client(srv.server) as client:
            return await client.call_tool(tool, args)

    result = asyncio.run(go())
    if result.is_error:
        raise ToolFailed(result.content[0].text)
    sc = result.structured_content
    if sc is not None:
        return sc.get("result", sc) if isinstance(sc, dict) else sc
    return json.loads(result.content[0].text)


def test_every_tool_is_listed():
    async def go():
        async with Client(srv.server) as client:
            return {t.name for t in (await client.list_tools()).tools}

    names = asyncio.run(go())
    assert {
        "robot_status",
        "run_experiment",
        "play_gamepad",
        "set_config",
        "hardware_snapshot",
        "start_recording",
        "stop_recording",
        "read_telemetry",
    } <= names


def test_run_experiment_orders_the_run_and_saves_it(bridge):
    out = call(
        "run_experiment",
        {
            "opmode": "Shooter Characterization",
            "duration_s": 0.2,
            "gamepad": {"steps": [{"at_ms": 0, "state": {"a": True}}], "end_ms": 200},
            "config_overrides": {"shooter.pidf.p": 0.002},
            "init_wait_s": 0.0,
            "coast_s": 0.0,
        },
    )
    run = Path(out["dir"])
    assert (run / "hardware.csv").read_text().startswith("t_ms,shooter.vel")
    assert (run / "telemetry.jsonl").exists()
    meta = json.loads((run / "meta.json").read_text())
    assert meta["config_overrides"]["shooter.pidf.p"]["after"] == 0.002
    events = [e["event"] for e in out["events"]]
    assert events[:4] == ["recording_started", "init", "config_overrides", "start"]
    assert events[-2:] == ["stop", "recording_stopped"]
    # Config is set after init (init reloads config) and before START.
    posts = [c for c in bridge.calls if c.startswith("POST")]
    assert posts.index("POST /agent/opmode/init") < posts.index("POST /agent/config")
    assert posts.index("POST /agent/config") < posts.index("POST /agent/opmode/start")
    assert bridge.gamepad == {"steps": [{"at_ms": 0, "state": {"a": True}}], "end_ms": 200, "pad": 1}
    assert out["summary"]["varying"]["shooter.vel"]["max"] == 1400
    assert bridge.state == "none" and not bridge.recording


def test_run_experiment_refuses_a_documented_out_of_range_override(bridge):
    with pytest.raises(ToolFailed, match="minimum"):
        call(
            "run_experiment",
            {"opmode": "X", "duration_s": 0.1, "config_overrides": {"shooter.pidf.p": -1.0}},
        )
    assert not any(c.startswith("POST /agent/opmode/init") for c in bridge.calls)


def test_run_experiment_stops_everything_when_a_step_fails(bridge):
    bridge.handle_orig = bridge.handle

    def failing(request):
        if request.url.path == "/agent/opmode/start":
            return httpx.Response(500, json={"error": "boom"})
        return bridge.handle_orig(request)

    srv._client = RobotClient("http://robot", transport=httpx.MockTransport(failing))
    with pytest.raises(ToolFailed, match="boom"):
        call("run_experiment", {"opmode": "X", "duration_s": 0.1, "init_wait_s": 0, "coast_s": 0})
    assert bridge.state == "none", "the OpMode was stopped"
    assert not bridge.recording, "the recorder was stopped"


def test_set_config_checks_the_documented_range(bridge):
    with pytest.raises(ToolFailed, match="minimum"):
        call("set_config", {"key": "agent_bridge.record_rate_hz", "value": 0})
    call("set_config", {"key": "agent_bridge.record_rate_hz", "value": 100})
    assert bridge.config["agent_bridge.record_rate_hz"] == 100


def test_schema_types_and_ranges():
    s = ConfigSchema.load(REPO)
    leaf = s.leaf("agent_bridge.record_rate_hz")
    assert (leaf.minimum, leaf.maximum) == (1.0, 200.0)
    assert s.leaf("agent_bridge.bridge_enabled").type == "boolean"
    s.check("agent_bridge.bridge_enabled", False)
    with pytest.raises(ValueError):
        s.check("agent_bridge.bridge_enabled", 1)
    with pytest.raises(ValueError):
        s.check("shooter", 1)
    with pytest.raises(ValueError):
        s.leaf("shooter.nope")


def test_summary_separates_varying_constant_and_empty_columns():
    text = "t_ms,a,b,c\n0,1,5,\n20,3,5,\n40,2,5,\n"
    s = summarize_csv(text)
    assert s["rows"] == 3
    assert s["timing"]["mean_dt_ms"] == 20
    assert s["varying"]["a"]["max"] == 3 and s["varying"]["a"]["last"] == 2
    assert s["constant"] == {"b": 5}
    assert s["never_read"] == ["c"]


def test_flight_only_run_saves_the_flight_and_skips_the_independent_recorder(bridge):
    out = call(
        "run_experiment",
        {
            "opmode": "TeleOp",
            "duration_s": 0.1,
            "init_wait_s": 0,
            "coast_s": 0,
            "flight_record": True,
            "independent_recording": False,
        },
    )
    run = Path(out["dir"])
    assert (run / "flight" / "hardware.csv").read_text().endswith("5,1,1100\n"), "offset loop joined chunks"
    assert "flight/events.jsonl" in out["files"] and "hardware.csv" not in out["files"]
    assert out["flight"]["summary"]["varying"]["shooter.vel"]["max"] == 1100
    assert "columns" not in out["flight"]["meta"], "layout is in meta.json, not the reply"
    assert not any("/agent/record/" in c for c in bridge.calls)
    events = [e["event"] for e in out["events"]]
    assert events[0] == "flight_started" and events[-1] == "flight_stopped"
    assert not bridge.flight_recording


def test_run_experiment_needs_some_recording(bridge):
    with pytest.raises(ToolFailed, match="enable"):
        call(
            "run_experiment",
            {"opmode": "X", "duration_s": 0.1, "independent_recording": False},
        )


def test_fetch_flight_follows_a_growing_file(bridge, monkeypatch):
    call("start_flight", {"label": "x"})
    bridge.grow_on_read = [b"9,2,1200\n", b"14,3,1250\n"]
    monkeypatch.setattr(srv.asyncio, "sleep", _no_sleep)
    out = call("fetch_flight", {"flight_id": "F1", "follow_s": 0.05})
    text = (Path(out["dir"]) / "hardware.csv").read_text()
    assert text.startswith("t_ms,loop_seq,shooter.vel\n0,0,1000\n")
    assert "9,2,1200\n" in text, "rows written after the first fetch were appended"
    assert out["summary"]["rows"] >= 3


async def _no_sleep(_seconds):
    return None


def test_configurables_are_set_before_init_checked_after_and_restored(bridge):
    out = call(
        "run_experiment",
        {
            "opmode": "Shooter Velocity Tuner",
            "duration_s": 0.1,
            "init_wait_s": 0,
            "coast_s": 0,
            "configurables": {"Tuner.targetRPM": 1300, "Casablanca.frictionX": 0.5},
        },
    )
    posts = [c for c in bridge.calls if c.startswith("POST")]
    assert posts.index("POST /agent/configurable") < posts.index("POST /agent/opmode/init")
    assert out["configurables"] == {"Tuner.targetRPM": 1300, "Casablanca.frictionX": 0.5}
    assert out["configurables_overwritten"] == {
        "Casablanca.frictionX": {"set": 0.5, "after_init": 0.1}
    }, "the agent is told init replaced its value"
    assert "Tuner.targetRPM" not in out["configurables_overwritten"]
    assert bridge.statics["Tuner.targetRPM"] == 870.0, "restored after the run"
    assert out["configurables_restored"] is True


def test_configurables_can_be_left_set(bridge):
    call(
        "run_experiment",
        {
            "opmode": "X",
            "duration_s": 0.1,
            "init_wait_s": 0,
            "coast_s": 0,
            "configurables": {"Tuner.targetRPM": 1300},
            "restore_configurables": False,
        },
    )
    assert bridge.statics["Tuner.targetRPM"] == 1300


def test_a_bad_configurable_key_undoes_the_ones_already_set(bridge):
    with pytest.raises(ToolFailed, match="no configurable"):
        call(
            "run_experiment",
            {
                "opmode": "X",
                "duration_s": 0.1,
                "configurables": {"Tuner.targetRPM": 1300, "Nope.x": 1},
            },
        )
    assert bridge.statics["Tuner.targetRPM"] == 870.0
    assert not any(c == "POST /agent/opmode/init" for c in bridge.calls)


def test_configurables_restored_when_the_run_fails(bridge):
    orig = bridge.handle

    def failing(request):
        if request.url.path == "/agent/opmode/start":
            return httpx.Response(500, json={"error": "boom"})
        return orig(request)

    srv._client = RobotClient("http://robot", transport=httpx.MockTransport(failing))
    with pytest.raises(ToolFailed, match="boom"):
        call(
            "run_experiment",
            {
                "opmode": "X",
                "duration_s": 0.1,
                "init_wait_s": 0,
                "coast_s": 0,
                "configurables": {"Tuner.targetRPM": 1300},
            },
        )
    assert bridge.statics["Tuner.targetRPM"] == 870.0
    assert bridge.state == "none"


def test_set_configurable_tool(bridge):
    out = call("set_configurable", {"key": "Tuner.targetRPM", "value": 1700})
    assert out["after"] == 1700
    assert call("get_configurable", {"key": "Tuner.targetRPM"})["value"] == 1700
