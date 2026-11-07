"""MCP server for the biobuzz robot's agent bridge.

Run by an MCP client (Claude Code reads the repo's .mcp.json). Talks HTTP to the bridge on the
Robot Controller (ROBOT_HOST, default 192.168.43.1, port ROBOT_PORT 8080).
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from mcp.server.mcpserver import Context, MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .analysis import summarize_csv
from .client import RobotClient, RobotError
from .schema import ConfigSchema

REPO_ROOT = Path(os.environ.get("ROBOT_MCP_REPO", Path(__file__).resolve().parents[3]))
RUNS_DIR = Path(os.environ.get("ROBOT_MCP_RUNS", REPO_ROOT / "calibration-data" / "agent-runs"))

INSTRUCTIONS = """\
Drives the biobuzz FTC robot through the agent bridge running inside the robot app.

What you can see: every hub-connected device sampled at up to 200 Hz independently of the OpMode
(encoder position/velocity, the power the OpMode last commanded, motor current, servo commands,
analog/digital inputs, hub voltage, bulk-read latency, the OpMode's gamepad input), plus every
telemetry update the OpMode sends to the Driver Station.

Two recordings: the independent recorder samples the hardware itself (ground truth, currents
included, one extra hub read per sample); the flight recorder (run_experiment flight_record,
start_flight/stop_flight) logs one row per read the OpMode itself made, from the SDK's caches,
at the OpMode's loop rate and no hub cost: what the code actually saw. Use both to check whether
an OpMode's outputs follow from its inputs and whether it reads fresh data every loop. With
agent_bridge.flight_auto the robot flight-records every OpMode run, matches included;
list_flights and fetch_flight retrieve them afterwards (fetch_flight follow_s tails a live one).

What you can do: init/start/stop OpModes, play timed gamepad scripts into the running OpMode
(it cannot tell them from a real gamepad), set config values in memory.

Workflow for tuning: read the OpMode's source and the config keys it uses (config.yaml, with
descriptions and ranges in config-docs.yaml) first. Use run_experiment for anything that moves:
it records hardware and telemetry around one OpMode run and saves them under
calibration-data/agent-runs/. Analyze the saved hardware.csv yourself (python, pandas) instead of
trusting the OpMode's printed conclusions; that is the point of the raw data. Compare against the
repo's simulations (FlywheelSim, TurretSim) where they exist. Persist a value you are confident in
by editing config.yaml and running ./gradlew pushConfig, which runs the push-config safety tests.

Panels configurables (the public static fields the Panels dashboard edits, e.g. a test
OpMode's target velocity or power) can be listed and set too: list_configurables,
set_configurable, or run_experiment's `configurables`, which also restores them afterwards.
They are not reset by OpMode init, unlike config.

Units and conventions: every *_rpm config value is encoder ticks per second. Gamepad stick _y is
negative when pushed forward. Config sets are discarded when an OpMode initializes (its init calls
config.reload()), so set values after init: run_experiment's config_overrides does this.

Safety: a person must be at the robot with the Driver Station; its stop button always works. An
OpMode you started is stopped by the robot if no request reaches it for agent_bridge.deadman_ms
(30 s): do not leave a mechanism running between tool calls. Leave the Driver Station's gamepads
unassigned while a script plays, or real input will override it.
"""

server = MCPServer("robot", instructions=INSTRUCTIONS)
_client: RobotClient | None = None
_schema: ConfigSchema | None = None


def robot() -> RobotClient:
    global _client
    if _client is None:
        _client = RobotClient()
    return _client


def schema() -> ConfigSchema:
    global _schema
    if _schema is None:
        _schema = ConfigSchema.load(REPO_ROOT)
    return _schema


def check_config(key: str, value: Any) -> None:
    """Documented type and range check; a refusal is a ToolError so the agent sees why."""
    try:
        schema().check(key, value)
    except ValueError as e:
        raise ToolError(str(e)) from e


READ = ToolAnnotations(readOnlyHint=True, openWorldHint=False)
ACT = ToolAnnotations(readOnlyHint=False, destructiveHint=False, openWorldHint=False)


class GamepadStep(BaseModel):
    at_ms: int = Field(ge=0, description="When this state starts, ms after the script starts.")
    state: dict[str, float | bool] = Field(
        default_factory=dict,
        description=(
            "Inputs held from at_ms until the next step. Axes: left_stick_x/y, right_stick_x/y "
            "(-1..1, y negative = forward), left_trigger/right_trigger (0..1). Buttons (true): a, b, "
            "x, y, dpad_up/down/left/right, left_bumper, right_bumper, left_stick_button, "
            "right_stick_button, back, start, guide, touchpad (PlayStation names also accepted). "
            "Omitted inputs are neutral."
        ),
    )


class GamepadScript(BaseModel):
    steps: list[GamepadStep] = Field(min_length=1)
    end_ms: int = Field(gt=0, description="When the pad returns to neutral and the script ends.")
    pad: int = Field(default=1, ge=1, le=2)

    def payload(self) -> dict:
        return self.model_dump()


# --- status and OpModes ------------------------------------------------------------------------


@server.tool(annotations=READ)
async def robot_status() -> dict:
    """Bridge state: active OpMode and its state, gamepad script, recorder, telemetry sequence."""
    return await robot().status()


@server.tool(annotations=READ)
async def list_opmodes() -> list[dict]:
    """Every registered OpMode: name, group, flavor (TELEOP/AUTONOMOUS)."""
    return await robot().opmodes()


@server.tool(annotations=ACT)
async def init_opmode(name: str) -> dict:
    """Init an OpMode by its exact name (as list_opmodes gives it). Fails if one is already active."""
    return await robot().init_opmode(name)


@server.tool(annotations=ACT)
async def start_opmode() -> dict:
    """Press START on the OpMode in INIT."""
    return await robot().start_opmode()


@server.tool(annotations=ACT)
async def stop_opmode() -> dict:
    """Stop the active OpMode (and any gamepad script). Always safe to call."""
    return await robot().stop_opmode()


# --- gamepad -------------------------------------------------------------------------------------


@server.tool(annotations=ACT)
async def play_gamepad(script: GamepadScript) -> dict:
    """Play a timed gamepad script into the active OpMode (INIT or running).

    Each step's state is held until the next step; nothing is interpolated, so write a ramp as
    several steps. The pad returns to neutral at end_ms. Returns at once; the robot plays it.
    """
    return await robot().play_gamepad(script.payload())


@server.tool(annotations=ACT)
async def press_button(button: str, hold_ms: int = 150, pad: int = 1) -> dict:
    """Press and release one button (e.g. 'a', 'dpad_up', 'right_bumper') on the active OpMode."""
    return await robot().play_gamepad(
        {"pad": pad, "end_ms": max(1, hold_ms), "steps": [{"at_ms": 0, "state": {button: True}}]}
    )


@server.tool(annotations=ACT)
async def stop_gamepad() -> dict:
    """Abort a running gamepad script; its pad returns to neutral."""
    return await robot().stop_gamepad()


# --- data ----------------------------------------------------------------------------------------


@server.tool(annotations=READ)
async def read_telemetry(since: int | None = None, max_entries: int = 50) -> dict:
    """Telemetry the OpMode sent to the Driver Station.

    Without `since`, only the latest update. With `since` (a telemetry seq), every update after it,
    oldest first, up to max_entries. Telemetry is throttled to ~4 Hz by the SDK: use recordings
    for anything faster.
    """
    return await robot().telemetry(since, max_entries)


@server.tool(annotations=READ)
async def hardware_snapshot() -> dict:
    """One reading of every hub-connected device, plus the device map (names, hubs, ports)."""
    return await robot().hardware()


@server.tool(annotations=READ)
async def start_recording(rate_hz: float | None = None) -> dict:
    """Start sampling all hardware (default agent_bridge.record_rate_hz). Returns the columns."""
    return await robot().start_recording(rate_hz)


@server.tool(annotations=READ)
async def stop_recording(label: str = "manual") -> dict:
    """Stop the recorder, save hardware.csv under calibration-data/agent-runs/, return a summary."""
    status = await robot().stop_recording()
    csv_text = await robot().recording_csv()
    run = _run_dir(label)
    (run / "hardware.csv").write_text(csv_text)
    (run / "meta.json").write_text(json.dumps({"recorder": status}, indent=2))
    return {"dir": str(run), "recorder": status, "summary": summarize_csv(csv_text)}


@server.tool(annotations=READ)
async def logcat(lines: int = 300, pattern: str = "AgentBridge|RobotCore|Exception|FATAL") -> str:
    """Recent robot log lines over ADB that match a regex (crashes, stack traces, bridge events).

    Needs adb connected to the Control Hub (./gradlew connectAdb); ROBOT_ADB picks the device.
    """
    adb = shutil.which(os.environ.get("ADB", "adb"))
    if adb is None:
        return "adb not found on PATH (set ADB to its full path)"
    cmd = [adb]
    if os.environ.get("ROBOT_ADB"):
        cmd += ["-s", os.environ["ROBOT_ADB"]]
    cmd += ["logcat", "-d", "-t", str(lines)]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired:
        return "adb logcat timed out"
    if out.returncode != 0:
        return f"adb failed: {out.stderr.strip()}"
    rx = re.compile(pattern)
    return "\n".join(line for line in out.stdout.splitlines() if rx.search(line)) or "(no matches)"


# --- config --------------------------------------------------------------------------------------


@server.tool(annotations=READ)
async def get_config(key: str) -> dict:
    """The live value of a config key on the robot (dotted path, e.g. shooter.pidf), with its
    documented description and range when the key is a leaf."""
    value = await robot().get_config(key)
    try:
        leaf = schema().leaf(key)
        value["doc"] = {"description": leaf.description, "min": leaf.minimum, "max": leaf.maximum}
    except (ValueError, FileNotFoundError):
        pass
    return value


@server.tool(annotations=ACT)
async def set_config(key: str, value: Any) -> dict:
    """Set a config leaf in robot memory, after checking its documented type and range.

    Takes effect on the next loop wherever code reads config each loop (e.g. shooter.pidf.*,
    shooter.ks, shooter.kick.*); values copied at construction need an OpMode re-init, and any
    OpMode init discards every set (config.reload()). Not persisted: edit config.yaml and run
    ./gradlew pushConfig to keep a value.
    """
    check_config(key, value)
    return await robot().set_config(key, value)


# --- Panels configurables ----------------------------------------------------------------------


@server.tool(annotations=READ)
async def list_configurables(filter: str = "") -> list[dict]:
    """The Panels configurables (public static fields of @Configurable classes, e.g. a tuning
    OpMode's target velocity or test power), with type and current value. `filter` matches the
    key, e.g. "ShooterVelocityTuner" or "power"."""
    return await robot().configurables(filter or None)


@server.tool(annotations=READ)
async def get_configurable(key: str) -> dict:
    """One Panels configurable by key (Class.field, or Class.field.member for an object field)."""
    return await robot().get_configurable(key)


@server.tool(annotations=ACT)
async def set_configurable(key: str, value: Any) -> dict:
    """Set a Panels configurable (number, integer, boolean, string, enum constant name). Takes
    effect wherever the code reads the field; the Panels dashboard shows the new value.

    Unlike config, it is NOT reset by an OpMode init and stays until set again or the robot app
    restarts: put it back when you are done (run_experiment's `configurables` does that for you).
    Some OpModes assign their statics during init (copying config); set those after init, or use
    the config key instead.
    """
    return await robot().set_configurable(key, value)


# --- experiments ---------------------------------------------------------------------------------


@server.tool(annotations=ACT)
async def run_experiment(
    opmode: str,
    duration_s: float,
    ctx: Context,
    gamepad: GamepadScript | None = None,
    config_overrides: dict[str, Any] | None = None,
    start: bool = True,
    init_wait_s: float = 1.0,
    coast_s: float = 1.5,
    rate_hz: float | None = None,
    label: str = "",
    flight_record: bool = False,
    independent_recording: bool = True,
    configurables: dict[str, Any] | None = None,
    restore_configurables: bool = True,
) -> dict:
    """Run one OpMode under recording and save everything.

    Two recordings, which answer different questions:
    - independent_recording (default on): the bridge samples every hub itself at rate_hz. Ground
      truth, including motor currents, at the cost of one extra hub read per hub per sample.
    - flight_record: one row per read the OpMode itself made (each new bulk read, each Pinpoint
      update), taken from the SDK's caches: exactly what the code saw, at its loop rate, at no hub
      cost. Comparing the two shows stale reads, missed loops and loop timing (loop_seq, t_ms).
      Saved under flight/ in the run directory.

    configurables sets Panels configurables (e.g. {"ShooterVelocityTunerOpMode.targetRPM":
    1300}) before init, since OpModes often copy them during init; after init each is read back,
    and one the OpMode overwrote is reported in configurables_overwritten (it was derived from
    config: use config_overrides). With restore_configurables (default) the original values are
    put back after the run, so the next person's run is not changed.

    Sequence: set configurables; start the recorder(s); init `opmode`; wait init_wait_s; apply
    config_overrides (after init, because init reloads config); press START unless start=False;
    play `gamepad` (its times count from START, or from init if start=False); run for duration_s;
    stop the OpMode; keep recording coast_s more (coast-down shows friction and stored energy);
    stop the recorder.

    Saves hardware.csv, telemetry.jsonl and meta.json in a new directory under
    calibration-data/agent-runs/ and returns its path with a summary. On any error it stops the
    OpMode and the recorder before raising.
    """
    if duration_s <= 0 or duration_s > 600:
        raise ToolError("duration_s must be in (0, 600]")
    overrides = config_overrides or {}
    for key, value in overrides.items():
        check_config(key, value)

    r = robot()
    status = await r.status()
    if status["opmode"]["state"] != "none":
        raise RobotError(f"'{status['opmode']['name']}' is active: stop it first (stop_opmode)")

    tel_seq = int(status.get("telemetry_seq", 0))
    telemetry: list[dict] = []
    events: list[dict] = []
    t0 = time.monotonic()

    def event(what: str, **data: Any) -> None:
        events.append({"t_s": round(time.monotonic() - t0, 3), "event": what, **data})

    async def pull_telemetry() -> None:
        nonlocal tel_seq
        page = await r.telemetry(tel_seq, 1000)
        for e in page.get("entries", []):
            telemetry.append(e)
            tel_seq = max(tel_seq, int(e["seq"]))

    if not (independent_recording or flight_record):
        raise ToolError("enable independent_recording, flight_record, or both")
    statics = configurables or {}
    originals: dict[str, Any] = {}
    statics_applied: dict[str, Any] = {}
    overwritten: dict[str, Any] = {}
    try:
        for key, value in statics.items():
            change = await r.set_configurable(key, value)
            originals[key] = change["before"]
            statics_applied[key] = change["after"]
    except Exception:
        await _restore_configurables(r, originals)
        raise
    if statics:
        event("configurables_set", keys=list(statics))
    flight_meta: dict | None = None
    recorder_layout: dict = {}
    try:
        if flight_record:
            flight_meta = await r.start_flight(label or opmode)
            event("flight_started", id=flight_meta["id"])
        if independent_recording:
            recorder_layout = await r.start_recording(rate_hz)
            event("recording_started")
    except Exception:
        # Nothing has run yet: undo what was started and every configurable set.
        if flight_meta:
            await r.stop_flight()
        if restore_configurables:
            await _restore_configurables(r, originals)
        raise
    applied: dict[str, Any] = {}
    recorder_status: dict = {}
    try:
        await r.init_opmode(opmode)
        event("init", opmode=opmode)
        await _sleep_polling(r, init_wait_s, pull_telemetry)
        for key, wanted in statics_applied.items():
            now = (await r.get_configurable(key))["value"]
            if now != wanted:
                overwritten[key] = {"set": wanted, "after_init": now}
        for key, value in overrides.items():
            applied[key] = await r.set_config(key, value)
        if overrides:
            event("config_overrides", keys=list(overrides))
        if start:
            await r.start_opmode()
            event("start")
        if gamepad is not None:
            await r.play_gamepad(gamepad.payload())
            event("gamepad_script", end_ms=gamepad.end_ms, pad=gamepad.pad)

        end = time.monotonic() + duration_s
        while (left := end - time.monotonic()) > 0:
            await ctx.report_progress(duration_s - left, duration_s, f"running {opmode}")
            await asyncio.sleep(min(0.5, left))
            await pull_telemetry()
            st = await r.status()
            if st["opmode"]["state"] == "none":
                event("opmode_ended_itself")
                break
    finally:
        try:
            await r.stop_opmode()
            event("stop")
            await _sleep_polling(r, coast_s, pull_telemetry)
        finally:
            try:
                if independent_recording:
                    recorder_status = await r.stop_recording()
                    event("recording_stopped")
            finally:
                try:
                    if flight_meta:
                        flight_meta = await r.stop_flight()
                        event("flight_stopped", id=flight_meta["id"])
                finally:
                    if restore_configurables and originals:
                        await _restore_configurables(r, originals)
                        event("configurables_restored", keys=list(originals))

    await pull_telemetry()
    run = _run_dir(label or opmode)
    csv_text = ""
    if independent_recording:
        csv_text = await r.recording_csv()
        (run / "hardware.csv").write_text(csv_text)
    flight_out = None
    if flight_meta:
        flight_out = await _download_flight(r, flight_meta["id"], run / "flight")
    with (run / "telemetry.jsonl").open("w") as f:
        for e in telemetry:
            f.write(json.dumps(e) + "\n")
    meta = {
        "opmode": opmode,
        "duration_s": duration_s,
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "gamepad": gamepad.payload() if gamepad else None,
        "config_overrides": applied,
        "events": events,
        "recorder": recorder_status,
        "devices": recorder_layout.get("devices"),
        "skipped_devices": recorder_layout.get("skipped"),
        "flight": flight_meta,
        "configurables": statics_applied,
        "configurables_overwritten": overwritten,
        "configurables_restored": restore_configurables,
    }
    (run / "meta.json").write_text(json.dumps(meta, indent=2))

    out: dict[str, Any] = {
        "dir": str(run),
        "files": sorted(str(p.relative_to(run)) for p in run.rglob("*") if p.is_file()),
        "events": events,
        "telemetry_updates": len(telemetry),
        "last_telemetry": telemetry[-1]["lines"] if telemetry else None,
    }
    if statics:
        out["configurables"] = statics_applied
        out["configurables_restored"] = restore_configurables and bool(originals)
    if overwritten:
        out["configurables_overwritten"] = overwritten
    if independent_recording:
        out["recorder"] = recorder_status
        out["summary"] = summarize_csv(csv_text)
    if flight_out:
        out["flight"] = flight_out
    return out


# --- flight recorder -----------------------------------------------------------------------------

FLIGHT_FILES = ("hardware.csv", "telemetry.jsonl", "events.jsonl", "meta.json")


async def _download_flight(r: RobotClient, flight_id: str, dest: Path) -> dict:
    """Copies a flight's files to dest (in full, from offset 0) and summarizes its hardware.csv."""
    dest.mkdir(parents=True, exist_ok=True)
    sizes = {}
    for name in FLIGHT_FILES:
        try:
            sizes[name] = await _fetch_file(r, flight_id, name, dest / name, 0)
        except RobotError as e:
            if "no " + name in str(e):
                continue
            raise
    csv_path = dest / "hardware.csv"
    meta_path = dest / "meta.json"
    return {
        "id": flight_id,
        "dir": str(dest),
        "bytes": sizes,
        "meta": _without_layout(json.loads(meta_path.read_text())) if meta_path.exists() else None,
        "summary": summarize_csv(csv_path.read_text()) if csv_path.exists() else None,
    }


async def _fetch_file(r: RobotClient, flight_id: str, name: str, path: Path, offset: int) -> int:
    """Appends the file's bytes from offset to path until caught up; returns the next offset."""
    mode = "ab" if offset else "wb"
    with path.open(mode) as f:
        while True:
            data, offset, size = await r.flight_chunk(flight_id, name, offset)
            f.write(data)
            if not data or offset >= size:
                return offset


def _without_layout(meta: dict) -> dict:
    return {k: v for k, v in meta.items() if k not in ("columns", "devices", "skipped")}


@server.tool(annotations=READ)
async def start_flight(label: str = "agent") -> dict:
    """Start the flight recorder: one row per read the running OpMode makes, from the SDK's caches
    (no hub cost), written to /sdcard/FIRST/flight/ on the robot until stop_flight. Works across
    OpMode init/start/stop, which it logs as events."""
    return _without_layout(await robot().start_flight(label))


@server.tool(annotations=READ)
async def stop_flight(fetch: bool = True) -> dict:
    """Stop the flight recorder; with fetch, copy the flight to calibration-data/agent-runs/."""
    meta = await robot().stop_flight()
    if not fetch:
        return _without_layout(meta)
    return await _download_flight(robot(), meta["id"], _run_dir("flight-" + meta["label"]))


@server.tool(annotations=READ)
async def list_flights() -> list[dict]:
    """Flights stored on the robot, newest first: agent flights and, with
    agent_bridge.flight_auto, one per OpMode run (matches included)."""
    return await robot().flights()


@server.tool(annotations=READ)
async def fetch_flight(flight_id: str, ctx: Context, follow_s: float = 0.0) -> dict:
    """Copy one flight (by id from list_flights) to calibration-data/agent-runs/ and summarize it.

    Works while it is still recording: with follow_s > 0 it keeps appending what the robot writes
    for that many seconds (a live tail, as fresh as agent_bridge.flight_flush_ms) before
    summarizing.
    """
    r = robot()
    dest = _run_dir("flight-" + flight_id)
    offsets = {}
    for name in FLIGHT_FILES:
        try:
            offsets[name] = await _fetch_file(r, flight_id, name, dest / name, 0)
        except RobotError as e:
            if "no " + name not in str(e):
                raise
    end = time.monotonic() + max(0.0, min(follow_s, 600.0))
    while (left := end - time.monotonic()) > 0:
        await ctx.report_progress(follow_s - left, follow_s, f"following {flight_id}")
        await asyncio.sleep(min(1.0, left))
        for name in list(offsets):
            # meta.json is rewritten on the robot, not appended: always take it whole.
            start = 0 if name == "meta.json" else offsets[name]
            offsets[name] = await _fetch_file(r, flight_id, name, dest / name, start)
    csv_path, meta_path = dest / "hardware.csv", dest / "meta.json"
    return {
        "id": flight_id,
        "dir": str(dest),
        "bytes": offsets,
        "meta": _without_layout(json.loads(meta_path.read_text())) if meta_path.exists() else None,
        "summary": summarize_csv(csv_path.read_text()) if csv_path.exists() else None,
    }


async def _restore_configurables(r: RobotClient, originals: dict[str, Any]) -> None:
    """Puts Panels configurables back; tries every one even if some fail."""
    errors = []
    for key, value in originals.items():
        try:
            await r.set_configurable(key, value)
        except RobotError as e:
            errors.append(f"{key}: {e}")
    if errors:
        raise RobotError("could not restore configurables: " + "; ".join(errors))


async def _sleep_polling(r: RobotClient, seconds: float, pull) -> None:
    """Wait while keeping the robot's deadman fed and draining telemetry."""
    end = time.monotonic() + seconds
    while (left := end - time.monotonic()) > 0:
        await asyncio.sleep(min(0.5, left))
        await pull()


def _run_dir(label: str) -> Path:
    slug = re.sub(r"[^A-Za-z0-9_-]+", "-", label).strip("-")[:48] or "run"
    path = RUNS_DIR / f"{datetime.now():%Y%m%d-%H%M%S}-{slug}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def main() -> None:
    server.run()


if __name__ == "__main__":
    main()
