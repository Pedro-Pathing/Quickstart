# robot-mcp

An MCP server that lets an AI agent run experiments on the robot and read the raw data: start
OpModes, play gamepad input, record every motor/servo/sensor at up to 200 Hz, capture telemetry,
and set config values live.

Two parts:

| Part | Where | What it does |
|---|---|---|
| Agent bridge | `TeamCode/.../agent/` (runs inside the robot app) | HTTP endpoints under `/agent/` on the Robot Controller's web server, port 8080 |
| MCP server | `tools/robot-mcp/` (runs on the laptop) | Turns those endpoints into MCP tools, saves runs, checks config ranges |

## Setup

1. Install the current TeamCode build on the robot. The bridge ships inside it, so there's
   nothing else to install.
2. Connect the laptop to the robot's Wi-Fi (`192.168.43.1`). Internet over Ethernet can stay
   connected at the same time.
3. Open the repo in your agent and trust the folder. The server is registered for each tool,
   and `uv` builds its environment on first start (it needs [uv](https://docs.astral.sh/uv/)
   installed: `brew install uv`).

| Agent | Registered by | Loaded when |
|---|---|---|
| Claude Code | `.mcp.json`, approved by `enabledMcpjsonServers` in `.claude/settings.json` | the folder is trusted |
| Codex | `.codex/config.toml` (`[mcp_servers.robot]`) | the project is trusted in Codex |
| Antigravity (`agy` and the IDE) | workspace plugin `.agents/plugins/robot/` (`plugin.json` + `mcp_config.json`), registered in `.agents/plugins.json` | the workspace opens; tools appear namespaced |

Codex and Antigravity don't document a server's working directory, so their configs start
`tools/robot-mcp/launch.sh` through `sh`, found with `git rev-parse --show-toplevel`. That
needs a POSIX shell: it works on macOS and Linux, and on Windows only from Git Bash or WSL.

Check it works: ask the agent for `robot_status`, or run
`curl http://192.168.43.1:8080/agent/status`.

Environment: `ROBOT_HOST`/`ROBOT_PORT` (default `192.168.43.1:8080`), `ROBOT_ADB` (device for
the `logcat` tool), `ROBOT_MCP_RUNS` (where runs are saved).

## Tools

| Tool | Use |
|---|---|
| `run_experiment` | The main tool: records hardware and telemetry around one OpMode run (init, config overrides, start, gamepad script, duration, stop, coast-down) and saves it under `calibration-data/agent-runs/<time>-<label>/`. `flight_record: true` adds the flight recording; `independent_recording: false` leaves only the flight recording |
| `start_flight`, `stop_flight`, `list_flights`, `fetch_flight` | The flight recorder on its own; `fetch_flight` copies any stored flight, and its `follow_s` tails one that is still recording |
| `play_gamepad`, `press_button`, `stop_gamepad` | Timed gamepad scripts into the running OpMode |
| `start_recording`, `stop_recording`, `hardware_snapshot` | The hardware recorder on its own |
| `read_telemetry` | What the OpMode shows on the Driver Station |
| `get_config`, `set_config` | Live config, checked against `config-docs.yaml`'s ranges |
| `list_configurables`, `get_configurable`, `set_configurable` | The Panels configurables: the `public static` fields of `@Configurable` classes, such as `ShooterVelocityTunerOpMode.targetRPM` or `MaxVelocityTest.power`. `run_experiment`'s `configurables` sets them before init and restores them after the run |
| `init_opmode`, `start_opmode`, `stop_opmode`, `list_opmodes`, `robot_status` | OpMode control |
| `logcat` | Robot log over ADB (crashes, bridge events) |

## What gets recorded

`hardware.csv` has one row per sample (`t_ms`), with these columns:

- `<motor>.pos`, `.vel`: encoder ticks and ticks/s, signed as `DcMotorEx` reports them
- `<motor>.power`: what the OpMode last commanded
- `<motor>.amps`: motor current, every `current_every_n_samples`th sample
- `<servo>.pos`, `<crservo>.power`, `<analog>.volts`, `<digital>.state`
- `hub<N>.bulk_ms`: how long the recorder's own hub read took, i.e. its cost to the OpMode's loop
- `hub<N>.volts`, `hub<N>.amps`
- `opmode_state`: 0 none, 1 init, 2 running
- `gp1.*`/`gp2.*`: the gamepad input the OpMode actually saw
- `telemetry_seq`: joins rows to `telemetry.jsonl`

I2C devices (the Pinpoint, color sensors) aren't sampled, because a second reader would disturb
their drivers.

## Flight recorder

The independent recorder answers "what is the hardware doing?". The flight recorder answers
"what did the code see?": one row each time the OpMode read something new, holding exactly the
values it got.

- **Sources.** Each hub's bulk cache is checked `agent_bridge.flight_poll_hz` times a second;
  when the OpMode replaces it, which is one bulk read, a row is written. The Pinpoint is read from
  its driver's cached fields: position, heading, velocities, encoders, loop time and status. No
  command is sent to any hub, so flight recording doesn't change the run it records.
- **What it doesn't capture.** Reads outside the bulk cache aren't recorded: motor current,
  `VoltageSensor`, I2C sensors other than the Pinpoint. Neither is anything from an OpMode that
  turns bulk caching off. For those, use the independent recorder.
- **Columns.** The same as the independent recorder, minus current, hub voltage and `bulk_ms`,
  plus `loop_seq` (a row counter) and `hub<N>.fresh`, which is 1 when that hub's data is new in
  this row. Gaps in `t_ms` show the OpMode's loop timing.
- **Storage.** It writes to `/sdcard/FIRST/flight/<time>-<label>/` on the robot as it runs:
  `hardware.csv`, `telemetry.jsonl`, `events.jsonl` (OpMode init/start/stop, gamepad scripts,
  config sets) and `meta.json`. Files are flushed every `flight_flush_ms`, so a crash loses at
  most that much. The newest `flight_keep` flights are kept.
- **File, not stream.** Writing on the robot keeps recording independent of Wi-Fi and of whether
  an agent is connected. Watching it live is reading the growing file: `fetch_flight` with
  `follow_s`. You can also copy flights with
  `adb pull /sdcard/FIRST/flight calibration-data/flight`.
- **Every run.** `agent_bridge.flight_auto: true` records each OpMode run without an agent
  (practice and matches included), to be fetched later with `list_flights` and `fetch_flight`.

## How it works, and its limits

- **Independent readings.** The recorder sends its own bulk-read command to each hub. It never
  calls `LynxModule.getBulkData()`, which would clear the OpMode's bulk cache in the middle of a
  loop. The cost is one hub transaction per hub per sample; check `hub<N>.bulk_ms`, and lower
  `rate_hz` if it matters.
- **Gamepad input.** Scripts go through the same SDK call as Driver Station packets
  (`newGamepadDataAvailable`), so unmodified OpModes respond to them, `aWasPressed()` included.
  A real gamepad assigned on the Driver Station overrides the script whenever it sends a packet,
  so leave the gamepads unassigned during agent runs.
- **Telemetry** is captured by wrapping the OpMode's telemetry sink, so the Driver Station still
  shows exactly what it did before. The SDK sends telemetry about 4 times a second, so use the
  recording for anything fast.
- **Panels configurables persist, config doesn't.** A configurable keeps the agent's value
  across OpMode runs until the app restarts, which is also how the Panels dashboard behaves.
  That's why `run_experiment` restores them by default. If an OpMode assigns one during init
  (`Casablanca` copies its statics from config), `run_experiment` reports it under
  `configurables_overwritten`; use the config key instead. After a set, Panels refreshes the
  class, so an open dashboard shows the new value.
- **Config sets are in memory only.** Any OpMode init discards them, because init calls
  `config.reload()`. That's why `run_experiment` applies `config_overrides` after init. To keep a
  value, edit `config.yaml` and run `./gradlew pushConfig`.
- **Safety.** The bridge never writes an actuator; OpModes still own every motor and servo.
  - An OpMode the bridge started is stopped after `agent_bridge.deadman_ms` with no requests.
  - Gamepad scripts are length-capped and always end neutral.
  - The Driver Station stop button always works.
  - Set `agent_bridge.bridge_enabled: false` before competitions.

## Development

```bash
cd tools/robot-mcp && uv run pytest
```

The tests run the real MCP server against a fake bridge, so no robot is needed. The bridge's own
tests run in `./gradlew verifyBuild`, under `TeamCode/src/test/.../agent/`.
