# decode — Contributor & Developer Guide

> FTC Robot Controller codebase for **Decode**
> Written in Java · Android · Gradle

This document is the authoritative reference for anyone contributing to this project. It describes where everything lives, what it does, how the systems interact, and how to make changes safely.

---

## Table of Contents

1. [Repository Layout](#1-repository-layout)
2. [TeamCode Package Map](#2-teamcode-package-map)
3. [Robot Container & Subsystems](#3-robot-container--subsystems)
4. [Autonomous OpModes](#4-autonomous-opmodes)
5. [TeleOp OpMode](#5-teleop-opmode)
6. [Configuration System](#6-configuration-system)
7. [Safety Systems](#7-safety-systems-casablanca--sentinel)
8. [Drivetrain & Pathing](#8-drivetrain--pathing)
9. [Hardware Abstraction](#9-hardware-abstraction--dairy-framework)
10. [Build System](#10-build-system)
11. [Library Reference](#11-library-reference)
12. [Contributing Rules](#12-contributing-rules)

---

## 1. Repository Layout

```
decode/
├── FtcRobotController/         # FTC SDK app module (do not edit)
├── TeamCode/                   # Our robot code (all new code goes here)
│   └── src/main/java/org/firstinspires/ftc/teamcode/
│       ├── auto/               # Autonomous OpModes (alliance-parameterized)
│       ├── teleop/             # TeleOp + calibration/autotune OpModes
│       ├── robot/              # Subsystems, Robot container, config facade
│       │   └── config/         # config.yaml, config-docs.yaml, generated RobotConfig.java + schema
│       ├── records/            # Immutable record/enum types (Alliance, MatchProfile, ...)
│       ├── utilities/          # Safety systems (Sentinel/Casablanca), vision, drawing, deprecated config shim
│       ├── pedro/              # Pedro Pathing constants & tuning OpMode
│       └── tests/              # Calibration/diagnostic OpModes
│   └── src/test/java/...       # JVM unit tests (Robolectric) — math/geometry + config schema
├── config-compiler/            # Standalone Android library: runtime YAML→POJO ConfigLoader

├── pedro-repo/                 # Pedro Pathing library source (reference)
├── pedro-docs/                 # Pedro Pathing documentation site source
├── dairy-docs/                 # Dairy framework documentation
├── dairy-cachinghardware/      # Dairy CachingHardware library source
├── dairy-sloth/                # Dairy Sloth library source
├── AGENTS.md                   # AI assistant rules & architecture context (source of truth — see §12)
├── CLAUDE.md / GEMINI.md       # One-line `@AGENTS.md` imports
├── .agents/skills/             # Shared AI skills (.claude/skills and .github/skills symlink here)
├── SETUP_HELPER.md             # Setup guide for new contributors
└── build.dependencies.gradle   # Centralized dependency versions
```

---

## 2. TeamCode Package Map

All robot-specific code lives under:
```
org.firstinspires.ftc.teamcode
```

| Package | Purpose |
|---|---|
| `auto/` | `AllianceAutoNew` / `AllianceOppositeNew` (alliance-parameterized autonomous logic) + `UnifiedAutos` (registers the four concrete OpModes) |
| `teleop/` | `TeleOp` (single, alliance-parameterized driver OpMode) + `AutotuneOpMode`-based and standalone calibration OpModes |
| `robot/` | Subsystem classes (`Intake`, `Shooter`, `Turret`, `Lumos`), `Robot` (composition root), `ShotController` |
| `robot/config/` | `config.yaml`, `config-docs.yaml` (hand-maintained), `RobotConfig.java` + `config-schema.json` (**generated** — never hand-edit) |
| `records/` | Immutable data: `Alliance`, `Field`, `MatchProfile`, `LaunchParameters`, `PIDGains` |
| `utilities/` | `Sentinel` / `Casablanca` (safety), `VisionUtil`, `DrawingUtil`, `PIDAutotuner`, deprecated `ConfigLoader` shim |
| `pedro/` | Pedro Pathing constants, localizer setup, and the vendored tuning OpMode |
| `tests/` | Calibration and diagnostic OpModes (not JUnit tests — those live under `src/test/java`) |

---

## 3. Robot Container & Subsystems

### `Robot.java` — composition root
Every OpMode constructs exactly one `Robot(hardwareMap, telemetry, matchProfile)`. Its constructor:
- Sets all REV hubs to manual bulk-caching mode.
- Builds the cached `Follower` via `Constants.create(hardwareMap)`.
- Constructs `Intake`, `Shooter`, `Turret` (wired to the follower's pose supplier), `Sentinel(profile.alliance())`, `Casablanca(sentinel)`, and `ShotController`.

`Robot.update()` must be called once per loop. It is the **only** place subsystem state advances:
```java
public void update() {
    for (LynxModule hub : allHubs) hub.clearBulkCache();
    follower.update();
    intake.periodic();
    shooter.periodic();
    turret.periodic();
    shotController.periodic();
    Scheduler.execute();
}
```
Never call `robot.update()` more than once per loop, and never skip it — subsystems only write to hardware from inside their own `periodic()`, so skipping it stalls the whole robot.

### `Intake.java`
Front and mid roller control. `run(frontPower, transferPower)` / `stop()`. `periodic()` is a documented no-op (pure pass-through actuator, nothing to reconcile each loop).

### `Shooter.java`
Dual flywheel + hood. **Do not call a velocity-setting method directly.** Call `setTargetPower(double)`; `periodic()` (invoked by `Robot.update()`) is the only code that calls the private `powerOnLauncher()`. Setting a raw power outside `periodic()` will be silently reverted on the next loop.
- `calculateLaunchParameters(pose, goalX, goalY, alliance)` → `LaunchParameters` (power, wait time, launch angle) based on distance; the far-shot base power is alliance-dependent (`RobotConfig.SHOOTER_RED_FAR_BASE_POWER` / `SHOOTER_BLUE_FAR_BASE_POWER`).
- `setShooterPIDFCoefficients()` re-reads `Shooter.PID_P/I/D/F` — call after changing them at runtime (e.g. from an autotune OpMode).

### `Turret.java`
CRServo turret with IMU yaw feedback, plus caches its own 4 drivetrain motor references for the heading-correction `updateTurn()` path. Aiming is driven by `AimMode`:
- `IDLE` — servo held at 0.
- `HOLD` — holds `holdAngle` (set via `setHoldAngle(degrees)`).
- `AIM_AT_GOAL` — continuously re-aims at `(goalX, goalY)` set via `setGoal()`.

Set the mode with `setAimMode(AimMode)`; `periodic()` resolves it every loop. Do not call `setTurretPowerRaw()` from match code — it's for calibration OpModes only, and bypasses the mode machine.

### `ShotController.java`
Owns the shot lifecycle: `startShot(power, checkAlignment)` sets the shooter target power and (if `checkAlignment`) switches the turret to `AIM_AT_GOAL`; `stopShot()` reverts the turret to `HOLD` at the current heading and stops the shooter. `periodic()` gates the intake feed on flywheel velocity and, if `checkAlignment`, turret alignment (`turret.isAimed(pose)`). Both `TeleOp` and the auto OpModes fire shots exclusively through this controller — there is no separate `Shooter.takeShot()`/`isShooting()` API to call from OpMode code.

### `Lumos.java`
goBILDA RGB indicator light. Colors are `Lumos.Color` (an enum with a `position` field), set via `setColor(Color)` — not one method per color.

---

## 4. Autonomous OpModes

Located in `auto/`. All extend `OpMode` and are built on the `com.pedropathing.ivy` Command API (`sequential()` / `instant()` / `waitUntil()` chains executed via `Scheduler.execute()` inside `Robot.update()`).

- `AllianceAutoNew` — abstract, takes `Alliance` in its constructor. "Score preload, then cycle the 3 field lines plus the drink gate twice."
- `AllianceOppositeNew` — abstract, takes `Alliance` in its constructor. "Score preload, cycle lines 2–4 plus the drink gate, then park," run from the wall opposite `AllianceAutoNew`.
- `UnifiedAutos` — registers the four concrete OpModes (`Blue Auto NEW`, `Red Auto NEW`, `Blue Opposite NEW`, `Red Opposite NEW`) via `@OpModeRegistrar`, as one-line subclasses (`super(Alliance.BLUE)` / `super(Alliance.RED)`). There are no separate per-alliance auto files to keep in sync — **add new autonomous behavior to the abstract class, not to a subclass.**

Each loads its poses via `ConfigLoader.loadMerged(RobotConfig.NormalAuto.class /* or OppositeAuto */, "auto_poses.normal." + allianceStr, "auto")`, which merges the alliance-specific pose block with the shared `auto.*` operational fields (wait times, powers) into one typed object (`this.config`).

### Blackboard Pose Handoff
Every auto/teleop OpMode persists its final pose so the next OpMode can resume from the correct field position:
- Red → `blackboard.put("RED_POSE", follower.getPose())`
- Blue → `blackboard.put("BLUE_POSE", follower.getPose())`

`TeleOp` reads this in `init()` and calls `follower.setStartingPose(pose)` if non-null. Autos also `blackboard.put("ALLIANCE", alliance)` so `TeleOp` can pick up the correct alliance automatically after an autonomous run.

---

## 5. TeleOp OpMode

There is a single `TeleOp.java` (not per-alliance files). It reads `Alliance` from the blackboard (defaulting to `RED` if unset — e.g. when run standalone), builds a `MatchProfile` via `RobotConfig.loadMatchProfile(alliance)`, and constructs `Robot` from it.

### Joystick Conventions
The FTC SDK returns **inverted Y-axis values** (pushing forward = negative). Negate the left stick Y and X before passing to the follower:
```java
double y = Math.clamp(Math.pow(-driver.left_stick_y, 3), -MAXSPEED, MAXSPEED);
double x = Math.clamp(Math.pow(-driver.left_stick_x, 3), -MAXSPEED, MAXSPEED);
```
Gamepads are traditional FTC SDK `Gamepad` instances (`gamepad1`, `gamepad2`), with rising/falling edge tracking managed via `ButtonTracker` in `TeleOpBase`.

### Casablanca Integration
Drive inputs pass through `robot.getCasablanca().adjustDriveInput(pose, velocity, x, y, r)` before being sent to the follower. This applies safety scaling when the robot approaches opponent goal zones. **Never bypass this call.**

### Command Bindings
All driver-facing behavior (drive, park/score/drink hold-points, intake, aim/shoot) is expressed as `Command`s built once in `init()` and scheduled/cancelled from button edges in `loop()` — see §3's `ShotController` note for how shooting is actually fired.

---

## 6. Configuration System

Config lives in one YAML file, with descriptions/constraints kept in a sibling docs file:
```
robot/config/
├── config.yaml         # Values only — no comments needed, descriptions live in config-docs.yaml
├── config-docs.yaml     # desc/min/max metadata, keyed by leaf key name
├── RobotConfig.java     # GENERATED — typed facade, do not hand-edit
└── config-schema.json   # GENERATED — JSON Schema, do not hand-edit
```

### Runtime loading
A separate Gradle module, `config-compiler`, provides `org.firstinspires.ftc.teamcode.config.ConfigLoader` — a small reflection-based YAML→POJO binder. It prefers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled classpath resource, so config changes can be hot-reloaded onto a connected robot without an APK rebuild.

`RobotConfig` (generated) wraps that loader with nested static classes mirroring `config.yaml`'s structure, plus flat `UPPER_CASE` aliases:
```java
double p = RobotConfig.turret.pidf.p;      // nested access
double p2 = RobotConfig.TURRET_PID_P;      // flat alias, same value
Pose start = RobotConfig.teleop.poses.red.start;
MatchProfile profile = RobotConfig.loadMatchProfile(Alliance.RED);
```
Access config through `RobotConfig` in subsystem code. Do not call `org.firstinspires.ftc.teamcode.config.ConfigLoader` directly except where you need `loadMerged(...)` for a class not already exposed as a `RobotConfig` field (as the auto OpModes do for per-alliance pose bundles). The old `utilities.ConfigLoader` (string-path `getDouble`/`getInt`/`getString`) is `@Deprecated` — it still works (it's a thin shim over the new loader) but new code should not call it.

### Adding a new config value
1. Add the value to `config.yaml`.
2. Add a `desc:` (and `min:`/`max:` if numeric) entry for its leaf key name in `config-docs.yaml`.
3. Run `./gradlew generateConfig` (or `./gradlew verifyBuild`) to regenerate `config.java` and `config-schema.json`.
4. If the key mentions an alliance (`red_*` / `blue_*`), make sure its mirror exists — `./gradlew checkConfigKeys` will fail the build otherwise (see below).
5. Access it via `config.<section>.<key>` (or its generated flat alias).

### Static checks
- `./gradlew checkConfigKeys` (part of `verifyBuild`) runs a pure Java check task (`ConfigGeneratorMain`), which flags alliance-name asymmetry (a `red_*` key with no `blue_*` mirror — build-failing) and fuzzy near-duplicate key names (Levenshtein distance ≤ 2 — warning only). Suppress a false positive with `suppress_similarity_check: true` under that key in `config-docs.yaml`.
- `ConfigValidationTest` (JVM unit test) validates `config.yaml` against the generated `config-schema.json`.


### Hot-deploying config to a connected robot
```bash
./gradlew pushConfig   # runs checkPushConfig (route + config checks), then adb pushes config.yaml; no rebuild
./gradlew resetConfig  # remove the override, revert to the bundled config
```
Re-init the OpMode on the Driver Station after pushing for the new values to take effect.

---

## 7. Safety Systems (Casablanca & Sentinel)

`Sentinel` and `Casablanca` are **per-match instances**, constructed once by `Robot` and retrieved via `robot.getSentinel()` / `robot.getCasablanca()` — they are not static utility classes and there is no global "current alliance." `new Sentinel(alliance)` computes goal/launch zones for that alliance; `new Casablanca(sentinel)` reads its friction/smoothing/protection config and holds that sentinel reference.

### `Sentinel.java`
Manages field geometry and protected zones (opponent goal, valid launch zones):
- Goal zones: `android.graphics.RectF` — axis-aligned bounding rectangles.
- Robot footprint: `android.graphics.PointF[]` — four rotated corner points.
- Overlap detection tries `android.graphics.Path.op(INTERSECT)` first, falling back to a hand-written Separating Axis Theorem (SAT) check when `Path.op` isn't available — this is what makes footprint rotation and intersection math unit-testable under Robolectric (see `MathSafetyTest#testSentinelFootprintRotationAndIntersections`).

**Key APIs:** `getRobotBounds(pose)`, `getProtectedZone()`, `isLaunchAllowed(pose)`, `violatesActiveGoal(footprint)`, `isRotationSafe(pose, turnInput, lookaheadRad)`.

### `Casablanca.java`
Intercepts drive commands and scales/repels velocity when the robot approaches its protected zone, using a `PredictiveBrakingController` to compute required stopping distance. Depth (X-axis) and side (Y-axis) protection are computed by the same `calculateAxisState(...)` helper called twice with the axis-appropriate bounds — **keep these two call sites symmetric** if you touch this method; a prior regression swapped the side-protection call to read the zone's X-bounds instead of its Y-bounds, and `MathSafetyTest#testCasablancaSymmetryAndCollisionMath` exists specifically to catch that class of bug.

**Integration:** call `casablanca.adjustDriveInput(pose, velocity, x, y, r)` every TeleOp loop. The returned `double[3]` is the adjusted `{strafe, forward, turn}`.

---

## 8. Drivetrain & Pathing

Pathing uses the **Pedro Pathing** library (`com.pedropathing:revhub:3.0.2`).

### Motor Caching
Use `Constants.create(hardwareMap)` from `org.firstinspires.ftc.teamcode.pedro` to build the configured follower. Pedro 3’s `Mecanum` drivetrain caches motor writes internally:
```java
Follower follower = Constants.create(hardwareMap);
```
Match and calibration/tuning OpModes use this same factory.

### Path Building
Paths are built in a `buildPaths()` method using `follower.pathBuilder()`. Use `BezierLine` for straight segments and `BezierCurve` for smooth arcs with control points.

### Poses
The FTC field coordinate system used by Pedro Pathing:
- X: 0 (left wall) → 144 (right wall) inches
- Y: 0 (bottom wall) → 144 (top wall) inches
- Heading: radians, 0 = facing right (+X), increases counter-clockwise

---

## 9. Hardware Abstraction & Dairy Framework

The **Dairy** framework is used for:

| Module | Use |
|---|---|
| `CachingDcMotorEx` | Caches motor power writes; only sends I2C commands when the value changes beyond a tolerance |
| `CachingCRServo` | Same caching behavior for continuous rotation servos |
| `CachingServo` | Same caching behavior for positional servos |
| `Pasteurized` / `SDKGamepad` | Enhanced gamepad wrapper with rising/falling-edge and bindable-threshold state helpers |
| `Sloth` | Loop timing and bulk read management |

**Caching tolerances** live in `config.yaml` under the `caching` key (`RobotConfig.caching.*`). Tighter tolerances = more I2C writes = slower loops. Start with `0.01` for motors.

---

## 10. Build System

| File / Module | Role |
|---|---|
| `build.gradle` | Top-level; configures AGP classpath |
| `build.dependencies.gradle` | All dependency version declarations (Pedro, Dairy, etc.) |
| `TeamCode/build.gradle` | TeamCode module build; applies FTC conventions, Spotless/JaCoCo, and the custom tasks below |
| `config-compiler/build.gradle` | Standalone Android library module providing the runtime `ConfigLoader` |
| `gradle.properties` | JVM args, Gradle caching, parallel execution, config cache |
| `settings.gradle` | Module declarations (`FtcRobotController`, `TeamCode`, `config-compiler`), plugin management |

**Custom Gradle tasks** (defined in `TeamCode/build.gradle`):
| Task | Purpose |
|---|---|
| `verifyBuild` | Spotless format, compile, JVM unit tests, and `checkConfigKeys` — no deployment |
| `checkConfigKeys` | Runs `scripts/check_config_keys.py` (alliance symmetry + fuzzy-duplicate check on `config.yaml`) |
| `pushConfig` | Runs `checkPushConfig`, then ADB-pushes `config.yaml` to a connected robot for hot reload (no rebuild) |
| `checkPushConfig` | The `push-config`-tagged tests (autonomous routes, config sanity, match windows) against `config.yaml`; recompiles nothing for a value-only change |
| `resetConfig` | Removes the ADB-pushed override |
| `jacocoTestReport` | Coverage report over the JVM unit tests |

**Useful vanilla Gradle tasks:**
```bash
./gradlew compileDebugJavaWithJavac    # Check for compile errors fast
./gradlew testDebugUnitTest            # Run the JVM/Robolectric unit tests
./gradlew clean                        # Remove build artifacts
./gradlew assembleDebug                # Build the APK
```

---

## 11. Library Reference

| Library | Where | Purpose |
|---|---|---|
| Pedro Pathing | `pedro-repo/`, published as `com.pedropathing` | Path following, motion profiling, localizer, Ivy Command scheduler |
| SnakeYAML | Maven, used by `config-compiler` | Parse `config.yaml` at runtime |
| Robolectric + Mockito | Maven, test-only | Run Android-dependent JVM unit tests (`Path`, `RectF`, mocked `HardwareMap`) without a device |
| json-schema-validator + Jackson | Maven, test-only | Validate `config.yaml` against the generated `config-schema.json` |
| Android SDK | Provided by FTC SDK | `Path`, `RectF`, `PointF` geometry types used in `Sentinel` |
| FTC SDK | Provided | All robot hardware APIs |

---

## 12. Contributing Rules

These are enforced by the AI assistant rules (`AGENTS.md`) and apply to all contributors:

1. **No hardcoding** — all tunable values go in `config.yaml`, described in `config-docs.yaml`, accessed via `RobotConfig`.
2. **Prefer libraries** — never write custom implementations of things libraries already do (geometry, PID, angle math, collections). The one sanctioned exception is `Sentinel`'s SAT fallback, which exists purely to make intersection math unit-testable under Robolectric.
3. **Cache hardware** — use Dairy caching wrappers where applicable. The drivetrain created by `Constants.create(hardwareMap)` uses Pedro 3’s built-in motor write caching.
4. **Single writer per actuator** — a subsystem's hardware-writing method may only be called from that subsystem's own `periodic()`. Callers set target state (`setTargetPower`, `setAimMode`, ...); `Robot.update()` is the only caller of `periodic()`.
5. **Dependency injection over static globals** — per-match state (alliance, goals, poses) flows through constructors (`Sentinel(Alliance)`, `Robot(..., MatchProfile)`), never a mutable static holder.
6. **Never bypass safety** — `Sentinel` and `Casablanca` (via `robot.getSentinel()`/`robot.getCasablanca()`) must remain active in all driving code paths.
7. **Command framework for autonomous** — new autonomous behavior belongs in `AllianceAutoNew`/`AllianceOppositeNew` using the Ivy Command API, not a new per-alliance file.
8. **Keep generated config files generated** — never hand-edit `RobotConfig.java` or `config-schema.json`; re-run `scripts/generate_config.py` instead.
9. **Math/geometry needs JVM tests** — pure math, coordinate rotations, and bounding-box/polygon intersections (`Sentinel`, `Casablanca`, `Turret`'s angle wrapping) must have unit test coverage in `src/test/java` (see `MathSafetyTest`).

### AI Rule Maintenance

To update the rules that AI assistants follow:
1. Edit `AGENTS.md` directly and commit it. Claude Code, Codex, GitHub Copilot and Antigravity all read it; there is no generation step.
2. Leave `CLAUDE.md` and `GEMINI.md` as bare `@AGENTS.md` imports — anything added there reaches only one tool.
3. Put shared skills in `.agents/skills/<name>/SKILL.md`. `.claude/skills` and `.github/skills` are symlinks to it (on Windows, enable `core.symlinks`).
