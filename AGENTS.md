# biobuzz

Instructions for AI coding agents (Claude Code, Codex, GitHub Copilot, Antigravity) working in this repository. This file is the single source: edit it directly. `CLAUDE.md` and `GEMINI.md` only import it, and shared skills live in `.agents/skills/`.

Follow these coding and design standards in the `biobuzz` codebase:

## Coding Conventions
1. **Dynamic Configuration Over Hardcoding**:
   - Never hardcode mechanical coefficients, tolerances, PID constants, safety margins, velocity scales, or autonomous timing values in Java code.
   - Add or change values in `config.yaml`; access them in Java via the generated `config` facade (e.g. `config.turret.pidf.p`) — do not call the low-level `config.ConfigLoader.load(...)` directly from subsystem code.
   - When adding a new config key, also add its description (and `min`/`max` where applicable) to `config-docs.yaml`, then run `./gradlew generateConfig` (or `./gradlew verifyBuild`) to regenerate `config.java` and `config-schema.json`. Never hand-edit either generated file.
   - If a new key mentions an alliance (`red_*`/`blue_*`), add its mirror for the other alliance or the `checkConfigKeys` Gradle task (part of `verifyBuild`) will fail the build.
   - Write field poses for RED and give BLUE the value `m` unless a measured BLUE value differs: `m` is the 180-degree rotation about the field centre, the BIOBUZZ field's real symmetry. Field geometry lives in `config.field` and is read through `game.FieldModel`; do not scatter field coordinates through other classes.
   - Bump nothing about `meta.season` within a season; change it only when the game changes, so stale ADB-pushed configs from another season are rejected.

2. **Prefer Libraries Over Custom Implementations**:
   - Never write custom code for something a library can already do — even if that library is not yet installed. Identify and add the appropriate dependency first.
   - Use JTS Topology Suite (`org.locationtech.jts.geom.Coordinate`, `Envelope`, `Polygon`) for 2D spatial geometry and polygon intersections instead of custom record types.
   - Use Pedro 3's `com.pedropathing.controllers.PIDController`/`Controller` and `com.pedropathing.api.Paths`/`PoseFactory` instead of any custom PID/PIDF implementations.
   - Use FTC SDK's `org.firstinspires.ftc.robotcore.external.navigation.AngleUnit` normalization methods instead of custom modulo math.
   - Applies to PID controllers, geometry types, angle math, data structures, interpolation, collections, and anything else with a well-supported library equivalent.

3. **Command-Based Subsystems**:
   - Use the `com.pedropathing.ivy.Command` API to structure actions (e.g., `runIntakeCommand`, `shootCommand`), with `.requiring(this)` constraints to prevent resource conflicts. `Robot.update()` runs `Scheduler.execute()` once per loop, before `ShotController.periodic()` and the subsystem writers.

4. **Single-Writer Actuator Pattern**:
   - Every actuator has exactly one method that writes to hardware, and it is only ever called from that subsystem's `periodic()` — see `Shooter.periodic()`, `Turret.periodic()`, and `Intake.periodic()` (which also applies the G407 inventory cap, so no caller can drive a fifth SCORING ELEMENT in).
   - `Robot.update()` is the only caller of subsystem `periodic()` methods, once per loop, in a fixed order: `ShotController` (decides targets), then the writers `Intake`, `Shooter`, `Turret`.
   - Commands, `ShotController`, and OpModes must only set *target* state (`setTargetPower`, `setAimMode`, `setHoldAngle`, ...) — never call a raw hardware-writing method (e.g. a motor's `setPower`/`setVelocity`) from outside a subsystem's own `periodic()`. Direct writes that bypass `periodic()` get silently overwritten on the next loop and have caused real regressions (a manual-rev button that appeared to do nothing because `periodic()` reverted it one loop later).

5. **Dependency Injection Over Static Global State**:
   - Per-match state (current alliance, field geometry, HIVE state, starting poses) is threaded through explicit constructor parameters — `FieldModel(Alliance)`, `Sentinel(FieldModel)`, `Casablanca(Sentinel)`, `ShotController(..., FieldModel, HiveTracker, ...)`, `Robot(HardwareMap, Telemetry, MatchProfile)` — not mutable public static fields.
   - Do not reintroduce a static "current alliance" or "current match" holder. A prior static-global design caused an initialization-ordering bug where `Sentinel` captured the wrong alliance depending on which OpMode constructed it first; constructor injection makes that class of bug structurally impossible and makes the safety/geometry classes independently unit-testable.

6. **Safety & Collision Avoidance**:
   - Do not bypass `Sentinel` or `Casablanca` protections during automated or manual driving sequences.
   - `Sentinel` and `Casablanca` are per-match instances owned by `Robot` (see rule 5) — always get them via `robot.sentinel` / `robot.casablanca`, never construct a second instance for the same match.
   - Any modifications to the safety calculations should use JTS geometry checks, and depth (X-axis) vs. side (Y-axis) protection must stay symmetric — if you change one axis's bounds wiring in `Casablanca.adjustDriveInput`, mirror it in the other.
   - Never launch at a CELL the `HiveTracker` does not believe is upward-facing (G417), and never feed with a `ShotSolution` whose epoch or ball type is not current (`ShotController.isCurrent`). Autonomous routes must keep the swept footprint on the alliance side (G402) and clear of the HIVE frame; `AutoRouteSafetyTest` enforces this for every configured path.

7. **Modern Java Features**:
   - Prefer modern Java syntax like `switch` expressions and `record` types for simple data containers (e.g. `MatchProfile`, `PIDGains` inside the `records` package).

8. **Autotuning & Calibration**:
   - For relay-based PID autotuning, extend `AutotuneOpMode` (which wraps `PIDAutotuner`) rather than reimplementing the oscillation/telemetry loop — implement its three abstract methods (`createAutotuner`, `readCurrentValue`, `applyPower`). Nothing uses it yet. Flywheel gains come from `ShooterCharacterizationOpMode`: feedforward kS and kV fitted at `nominal_voltage`, `p = 2 kV` (loop gain 2) and `i = p / 2 s`, integrating only inside `integral.band_ticks`, clamped by `integral.max_contribution`. That structure was chosen by sweep and is held by `FlywheelClosedLoopProperties` (calibration error kS ±30% / kV ±10%, 11.5-13.8 V, sensor lag, ball hits); re-run it after changing any of those. The old `ShooterAutotuneOpMode` is `@Disabled` because its fallback gain oscillates. The runtime uses this loop with `shooter.use_ftc_pid: false` (the default); `true` switches to the hub's firmware PIDF (`motor_pidf`, REV's recipe p = 0.1 F, i = 0.1 p).
   - Every tuning OpMode calls `OpModeUtil.reportConfig(telemetry)` right after `config.reload()`, so a rejected `pushConfig` or a `ConfigSanity` problem shows at INIT; new ones must too.
   - For non-PID calibration (e.g. finding a friction/kS constant via a power ramp), model the OpMode after `FrictionCalibrationOpMode` instead — it is not a `PIDAutotuner` use case.
   - Pedro 3 drivetrain, Pinpoint, and Foresight tuning lives in `pedro/Tuning.java` as `@Tuner` procedures. Copy measured values into `config.yaml`, document the keys in `config-docs.yaml`, and regenerate the facade.
   - Turret gains come from `TurretPlantCalibrationOpMode`: measured `ks_positive`/`ks_negative`, `kv = 1/K`, `p = 1/(K tau)` and `i = d = f = 0`. The convergence proof (block 10, `verifyTurretLean`) covers exactly that form. After changing any turret gain, run `python3 TeamCode/verification/TurretProof/conditions.py`: it checks the proof's conditions against `config.yaml` for the plant box and prints the target distances the proof covers (exit 1 if not covered to `--max-distance`). A non-zero `i`, `d` or `f`, or friction the P + kS drive cannot beat at the tolerance's edge, puts the tuning outside the proof; the tests still run, but nothing is proven about convergence.

9. **Mathematical & Algorithmic Unit Test Coverage**:
   - High overall global code coverage (e.g., 80% across the entire codebase) is not required.
   - However, all pure mathematical functions, geometry collision calculations (e.g., coordinate rotations, bounding box intersections in `Sentinel`/`Casablanca`, the alliance rotation in `FieldModel`, camera transforms in `HiveVision`), state machines (`HiveTracker`, `MatchClock`, `BallInventory`) and numeric helpers (e.g., yaw angle wrapping in `Turret`) **must** have full local JVM unit test coverage to prevent logic regressions. See `MathSafetyTest` and `AutoRouteSafetyTest` for the baseline.
   - Tests are JUnit 6 (Jupiter); state invariants as jqwik properties where an invariant exists (alliance symmetry, X/Y symmetry, "never toward the HIVE", epoch/TIP consistency), and check derived quantities against an independent oracle (e.g. a numerical derivative) rather than re-deriving the formula. Turret control changes must keep `TurretClosedLoopProperties` green.
   - Never put a non-heading quantity in a Pedro `Pose` heading: the constructor wraps it into [0, 2pi). Velocities are `com.pedropathing.math.Velocity`.
   - A test that changes `config` must restore it (Jupiter tests are reloaded automatically by `FreshConfigExtension`; jqwik classes restore in `@AfterProperty`).

10. **Verification Task Workflow**:
    - `./gradlew verifyTurret` proves the JML contracts in `TurretLaw`, `TurretPlant` (test sources: the plant physics `TurretSim` runs) and `verification/java/.../TurretStep.java` (one control loop as exact formulas over both) with OpenJML, and beyond the turret: `FeedLaw` (every shot feed-gate decision, including G417 and solution currency), `HiveLaw` (`HiveTracker`'s transitions: epoch, confirm frames, settle window) and the static transitions of `AntiWindupIntegrator` and `BallInventory` (including the G407 collect rule `Intake` calls). Only static methods are proven and each proven file compiles on its own (java.lang plus the SDK), so keep these decisions in those static methods and have the stateful classes call them; `setupOpenJml` downloads it to `~/.openjml` once. `-PexactDoubles` proves with bit-exact IEEE-754 rounding (CVC5, minutes); the default proves over real numbers. Contracts for library methods the proofs call (`AngleUnit`, `Math`) are in `TeamCode/verification/specs` and are trusted, so keep them faithful to the library. Never call `AngleUnit.normalizeDegrees` on an unbounded value: it never returns for some huge finite inputs; use `TurretLaw.wrap`.
    - `./gradlew verifyTurretLean` checks the Lean 4 + Mathlib proof (`verification/TurretProof`, `TurretStep.converges`) that the turret, from settled or approaching a still target, ends settled and aimed and stays there, for any plant in the stated box and gains meeting the `Witness` conditions. It regenerates `TurretProof/Step.lean` from `TurretStep.java`'s contracts (`gen_step.py`) first, so the Lean model is the formulas OpenJML proves the code computes; never edit `Step.lean`, and keep `TurretStep`'s contracts in the formula language its javadoc lists. It assumes the autotuner's `i = d = f = 0`; whether a given tuning meets the conditions is `conditions.py`'s job (block 8), not the build's. Needs Lean (`brew install elan-init`); `setupLean` fetches Mathlib's prebuilt cache (~5 GB) once.
    - Both proof tasks run with every assemble/install and `verifyBuild`, in parallel with the build, and are cached on their input files: they only re-run when a proven file, a spec or the Lean proof changes.
    - Write JML as `/*@ ... @*/` blocks, never `//@` lines: Spotless (google-java-format) turns `//@` into a plain `// @` comment, silently dropping the proof step. Keep OpenJML to code-level facts (a method computes this formula, never throws, stays in range): OpenJML models every `double` with NaN, infinities and signed zeros, so a theorem spanning several calls becomes an SMT query too large to finish. Put mathematical arguments over several loops in the Lean proof. In Lean, prefer `linarith only [...]` and named facts over open-ended `nlinarith`, and introduce derived quantities with `obtain ⟨x, hx⟩ : ∃ x, x = ...` rather than `set`, so tactics never unfold real arithmetic.
    - Unit tests run in one JVM (a second costs more warm-up than it saves), with Mockito's subclass mock maker and no coverage instrumentation (`-Pcoverage` for `jacocoTestReport`). Simulations take an injected clock (`TurretSim`, the package-private `Turret` constructor); never `Thread.sleep` or spin on `System.nanoTime` in a test.
    - Prefer running the custom task `./gradlew verifyBuild` to run the exact same verification checks as a standard build (Spotless auto-formatting, compiler error checks, local JVM tests, and the `checkConfigKeys` config-symmetry check) without deploying the application.
    - Always run `./gradlew` commands outside the terminal sandbox (`BypassSandbox: true`) because Gradle requires access to global cache lock files (e.g. `~/.gradle/wrapper/dists/...`) which are restricted by standard sandboxing.
    - For live tuning on a connected robot without a full APK rebuild, use `./gradlew pushConfig` to hot-deploy `config.yaml` over ADB (re-init the OpMode to pick it up) and `./gradlew resetConfig` to revert to the bundled config. `pushConfig` first runs `checkPushConfig`: only the JUnit tests tagged `push-config` (`AutoRouteSafetyTest`, the shipped-config `ConfigSanity` check, the match windows), reusing the unit-test build, so a value-only change recompiles nothing. A new test that guards a config value the robot does not re-check gets `@Tag("push-config")` and its class goes in `checkPushConfig`'s filter; keep the set's test time under 500 ms. The robot does not re-check autonomous routes at INIT: these tests are the only route check. This is a deployment convenience, not part of `verifyBuild`.

## Project Overview

This project is a First Tech Challenge (FTC) robot controller application written in Java for the 2026-2027 game, BIOBUZZ presented by RTX (Competition Manual TU05). Robots collect POLLEN (2.8 in yellow balls) and NECTAR (3.6 in alliance-colored balls) and LAUNCH them into the upward-facing CELL of their alliance's HIVE; enough SCORING ELEMENTS TIP the HIVE (20 points, and one more NECTAR for the human player), which turns the other CELL upward.

## Core Architecture Blocks

1. **Subsystem Decomposition**:
   - Robot functionality is split into modular subsystems located under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/`.
   - Key subsystems:
     - `Intake.java`: Front and middle rollers plus the G407 four-element `BallInventory`. Callers set a target with `run` (collect), `feed`, `eject` or `stop`; only `periodic()` writes the motors, and it holds a collect off once the inventory is full. An optional intake-entry `DigitalChannel` (`config.inventory`) counts elements in; without it the count is unknown once the intake runs, and an unknown count never blocks the intake.
     - `Shooter.java`: Double flywheel and hood positioning. Requires `shooter.max_rpm` from config (every `*_rpm` value is `DcMotorEx.setVelocity` encoder ticks/second, despite the name). Exposes only `setTargetPower(double)`; the flywheel is driven exclusively from `periodic()`.
     - `Turret.java`: CRServo-driven turret rotation, positioned via an absolute analog encoder plus the `Follower`'s pose supplier. Aiming is driven by an `AimMode` (`IDLE` / `HOLD` / `AIM_AT_TARGET` / `MANUAL`) resolved once per loop in `periodic()`; `ShotController` sets the target point to the upward CELL every loop. Every turret decision (encoder angle, angle wrap, tolerance, settle hysteresis, commands, the servo output gate, the aim claim) lives in the pure `TurretLaw`, whose JML contracts OpenJML proves against the source; change `TurretLaw` only with its contracts, and keep `Turret` to reading config/hardware, calling it, and writing the servo.
     - `ShotController.java`: Owns the shot lifecycle (`startShot`/`stopShot`/`periodic()`, `aimAndShootCommand`, autonomous `volleyCommand`). Its target is whichever CELL the `HiveTracker` believes is upward. Feeding requires every gate: flywheel in band, turret aligned, a solution that is valid AND current (same HIVE epoch and ball type as now), and the HIVE settled after a TIP. Runs its solve (`ballistics.ShotSolver`) on a dedicated background thread.
     - `Lumos.java`: goBILDA RGB indicator light via `ServoImplEx` with PWM range control. Colors are a `Lumos.Color` enum.
   - Subsystem behaviors are orchestrated using the `com.pedropathing.ivy.Command` command framework.

2. **Robot Container (`Robot.java`)**:
   - `Robot` is the single composition root for a match: it builds the cached `Follower`, the per-match `FieldModel`, `HiveTracker`, `Sentinel`, `Casablanca`, all subsystems, `ShotController` and `HiveVision`, from a `MatchProfile` (see block 7).
   - `Robot.update()` is called once per loop and is the *only* place subsystem state advances, in this order: clear the bulk cache, `follower.update()`, feed new camera frames to the `HiveTracker` (once per frame), `Scheduler.execute()`, then `ShotController.periodic()` (decides targets), then the writers `Intake.periodic()`, `Shooter.periodic()`, `Turret.periodic()`. Deciding before writing means every target set this loop reaches hardware this loop.
   - `Robot.relocalizeFromHive()` re-anchors odometry on the upward CELL (operator-triggered).
   - `Robot.shutdown()` stops the shot, the solver thread and the camera, and resets the Scheduler. It must be called from the owning OpMode's `stop()`; a new `Robot` is built on every init. It is idempotent.

3. **Drivetrain & Pathing (Pedro Pathing)**:
   - Pedro 3.0.2: `Follower`, `Foresight`, `Mecanum`, `PinpointLocalizer`, `com.pedropathing.math.Pose`, `com.pedropathing.api.Paths`. `pedro/Constants.java` builds them from generated `config.pedro` values via `Constants.create(hardwareMap)`; `pedro/Tuning.java` registers the AutoTune procedures.
   - Pedro frame used everywhere: origin at the field corner where the audience wall meets the RED alliance wall, +x toward the BLUE wall, +y away from the audience, heading 0 = +x, CCW positive. The field is `config.field.size_in` (141.5) wide; the RED side is x < size/2.

4. **BIOBUZZ Game Model (`game/`)** — the season-specific layer; replace this package next season:
   - `FieldModel`: immutable per-alliance geometry (HIVE CELL openings up/down, HIVE frame footprint, LOADING ZONE, GARDEN, alliance-side FLOWERS, alliance side, AprilTag cluster ownership). Everything is defined in RED's frame and BLUE is the 180-degree rotation about the field centre — the BIOBUZZ field is rotationally, not mirror, symmetric.
   - `HiveCell.A` is the alliance's CELL that starts upward-facing (RED audience CELL, tags 34-37; BLUE rear CELL, tags 42-45); `B` is the other.
   - `HiveTracker`: which CELL is up, TIP count, an `epoch` that changes on every accepted change, and a settle window after each change. Fed by AprilTag height classification (N consecutive disagreeing frames required) and by the operator (`recordTip`/`undoTip`). Shot solutions carry the epoch, so a solution from before a TIP can never fire.
   - `MatchClock`: AUTO/TELEOP period timing and the G410/G426 final-minute unlock cue, the park-alert rule (`isParkAlert`: inside the last `final_warning_remaining_ms`, time left at most the estimated drive to park plus a margin), and the human-player NECTAR allowance.

5. **Vision (`utilities/HiveVision.java`)**:
   - SDK 12's BIOBUZZ AprilTag library defines each four-tag CELL cluster with its origin at the CELL opening centre and no field position, because CELLS move. A detection is therefore "where is this CELL opening relative to the camera", not a field pose. `HiveVision` converts `ftcPose` to the robot frame using the configured camera mount and identifies the CELL from the cluster name (SDK 12 exposes no member IDs).
   - Height above the TILES classifies a CELL as upward (~59.6 in) or downward (~38.2 in). Only while the HIVE is settled is the upward CELL's opening a fixed landmark usable for relocalization; never reset odometry from a moving HIVE.

6. **Ballistics (Shot Solving)**:
   - `ballistics/` holds `BallisticsModel`, `ShotSolver`, `ShotTable`, `ShotTimeTable`, `FlywheelFeedforwardFit`.
   - `ShotSolver.solve(...)` looks up hood and flywheel by horizontal distance to the target CELL opening in the measured table for the SCORING ELEMENT being shot (`shooter.shot_table.pollen_points` / `nectar_points`), leads moving shots, refuses shots from outside the opening's approach cone (`max_off_axis_deg`; the opening faces outward and 30 degrees up), and refuses everything while `shooter.shot_table.calibrated` is false. `ShotController` runs it on a daemon thread; never call `ShotSolver.solve()` from `periodic()`, and any new state shared with the solver thread must be `volatile` or guarded by `solveLock`.
   - `ShotTimeTable` is distance-keyed (`auto.shot_time_points`), exactly the format the Shot Timing Tuner exports.
   - `FlywheelDipDetector` counts balls fired by the flywheel's RPM dip measured against a running reference; each counted shot also decrements the inventory.

7. **PID Anti-Windup**:
   - Pedro 3's `PIDController` integral term has no clamp; `Shooter` uses a separate `AntiWindupIntegrator` (`utilities/`) for the I term. When it is in use the `PIDController` must have `kI = 0`. Flywheel feedforward is applied separately.

8. **Safety (Sentinel & Casablanca)**:
   - `Sentinel` and `Casablanca` (`utilities/`) are per-match instances owned by `Robot`: `new Sentinel(fieldModel)`, `new Casablanca(sentinel)`. There is no static "current alliance".
   - `Sentinel` uses JTS polygons for the robot footprint: protected zones (the HIVE frame — G417 makes ramming it a foul), fully-on-alliance-side (G304.A / G402), PARK (partly in the LOADING ZONE), LEAVE (not touching a wall), swept-path checks for autonomous routes (`routeProblem`) and a HIVE-avoiding `parkRoute`.
   - `Casablanca` slows and repels TeleOp drive input near every protected zone, using `Casablanca.brakingDistance` (calibrated linear/quadratic stopping model). X and Y protection use the same `calculateAxisState` helper per zone with swapped bounds — keep them symmetric.

9. **Alliance-Parameterized OpModes**:
   - One `TeleOp`/`TeleOpBase` pair. It reads `Alliance` from the blackboard; if no AUTO set it, it defaults to RED and lets the drivers pick with B (RED) / X (BLUE) during INIT, rebuilding the robot. It builds a `MatchProfile` via `config.loadMatchProfile(alliance)`.
   - Autonomous: `AllianceAutoBase<T>` (no route check on the robot: `AutoRouteSafetyTest` at build and push time; CELL-aware `volley(cell)` that never launches at a downward CELL, and a park guard that cancels the routine at its park deadline, the estimated drive to park (`ParkTimeEstimate`: stopping, then `Sentinel.parkRoute`) plus `auto.park_margin_ms` and never later than `auto.park_reserve_ms`, and parks for LEAVE + PARK; the same estimate drives TeleOp's park alert) extended by `CellAAuto` (tip CELL A with the pre-load, optional FLOWER refill and CELL B volley), `CellBAuto` (partner routine: volley into CELL B after the partner's TIP, falling back to CELL A) and `ParkOnlyAuto`. `UnifiedAutos.java` registers each for RED and BLUE.
   - OpModes persist the pose (`"RED_POSE"` / `"BLUE_POSE"`) and a time-stamped `Handoff` (HIVE state, TIPS, inventory) to the blackboard. TeleOp restores the handoff only if it is fresh (`OpModeUtil.HANDOFF_MAX_AGE_MS`), so a previous match's TIPS never leak into a new one.

10. **Dynamic Configuration System**:
    - One YAML file, `robot/config/config.yaml`, with descriptions and min/max in the sibling `config-docs.yaml` (keyed by leaf name). Poses and zones are written for RED; BLUE values are `m`, which `ConfigLoader` resolves as the 180-degree rotation about the field centre (`ConfigLoader.allianceTransform`).
    - Runtime loading is the `config-compiler` module's `ConfigLoader`. It layers an ADB-pushed override at `/sdcard/FIRST/teamcode/config.yaml` over the bundled file — but only if the override's `meta.season` matches the bundled season; otherwise it is ignored and the reason shown in telemetry (`ConfigLoader.overrideRejection()`).
    - `config.java` (under `robot/config/generated/`) is a generated typed facade; regenerate with `./gradlew generateConfig` and never hand-edit it or `config-schema.json`. `checkConfigKeys` (part of `verifyBuild`) fails the build on `red_*`/`blue_*` asymmetry.

11. **Dedicated Records Package**:
    - `org.firstinspires.ftc.teamcode.records` holds immutable data: `Alliance`, `HiveCell`, `BallType`, `CellTarget`, `MatchProfile` (alliance plus TeleOp start/park/shoot_a/shoot_b/flower poses), `PIDGains`, `ShotInputs` / `ShotSolution` / `BallisticsParameters`, `SlewProfile` / `SlewTuning`, `TiltReading`.

12. **Test Layout — Two Different "tests"**:
    - `TeamCode/src/test/java/...`: real JUnit 6 (Jupiter) example tests and jqwik property tests (`*Properties`), run by `./gradlew testDebugUnitTest` (part of `verifyBuild`). Includes `AutoRouteSafetyTest`, which checks every configured autonomous path for both alliances against the HIVE frame, the centre line and the walls, `TurretClosedLoopProperties`, which runs the real `Turret` against `TurretSim` (randomized servo gain, asymmetric stiction, lag, encoder noise, loop jitter), and `FlywheelClosedLoopProperties`, which runs the real `Shooter` against `FlywheelSim` (calibration error, battery voltage, spin-up lag, velocity-sensor lag, ball hits). `FreshConfigExtension` reloads the bundled config around every Jupiter test so no test can leak a config change into another; jqwik property classes restore what they change in `@AfterProperty`.
    - `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tests/`: on-robot calibration and tuning **OpModes** (e.g. `BallisticsCalibrationOpMode`, `ShotTimingTunerOpMode`, `FrictionCalibrationOpMode`) — not JUnit tests. Ballistics and timing calibration both shoot at RED's CELL A along the same `CalibrationRay`.
