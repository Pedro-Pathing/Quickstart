# DECODE → BIOBUZZ upgrade report

Branch `biobuzz-season-upgrade` (uncommitted). Rules basis: Competition Manual **TU05** (`~/Documents/BIOBUZZ_Competition_Manual_TU05.pdf`), the official Event Field Setup Guide (re-downloaded; byte-identical to kickoff v1.0) and the official field CAD (via the ChatGPT project's extraction). Day-to-day reference for the team: `BIOBUZZ-SEASON.md` (rules → code, bring-up checklist) and `teleop_controls.md`.

## 1. What the code does now

The DECODE game layer is gone and a BIOBUZZ one replaces it. The robot architecture (mecanum + Pinpoint/Pedro 3, intake/transfer, dual flywheel + hood, CR turret, webcam, command-based subsystems, generated YAML config, Casablanca drive assist) is kept, because BIOBUZZ is again "collect balls, launch them into a target".

**Ripped out (DECODE-only):** goal coordinates and goal zones, launch-zone polygons, the end-of-auto evacuate/freeze-at-launch-zone logic, `EndgameSpot`, `records/Field`, "drink"/gate poses and commands, the Normal/Opposite autos, the field-pose AprilTag reset (`VisionUtil`, which filtered "not Obelisk" tags), the X-mirror at 70.75, the `opposite_` config hack, and every DECODE pose, auto timing and shot table.

**Added:**

| Area | What | Why (rule) |
| --- | --- | --- |
| `game/FieldModel` | BIOBUZZ geometry: CELL openings up/down, HIVE frame, LOADING ZONE, GARDEN, FLOWERS, alliance side, AprilTag cluster ownership. RED defined once, BLUE = 180° rotation | Field is rotationally symmetric (manual Fig 9-2); CAD-measured values |
| `game/HiveTracker` | Which CELL is up, TIP count, epoch, settle window. Vision (height classification, N-frame confirmation, once per camera frame) + operator TIP/undo | Only the upward CELL is a legal target (G417); TIP flips the target; launching into a tipping HIVE can stop the TIP (10.5.1) |
| `utilities/HiveVision` | SDK 12 cluster detections → CELL opening position in the robot frame → up/down classification; relocalization only from the settled upward CELL | SDK 12 BIOBUZZ clusters have no field pose (origin = opening centre) |
| `ShotController` / `ShotSolver` | Target = upward CELL; solutions tagged with HIVE epoch + ball type, stale ones can't move the hood/flywheel or feed; approach-side gate (±60° off the opening's axis); POLLEN/NECTAR tables; `calibrated: false` blocks auto-fire; lob flight-time model for lead | Opening faces outward and 30° up; two ball sizes |
| `Intake` + `BallInventory` | Real single-writer intake (target set → `periodic()` writes), 4-element cap with optional debounced entry sensor, eject | G407 (4 max), G408 (eject opponent NECTAR) |
| `Sentinel` / `Casablanca` | Protect the HIVE frame (multi-zone), alliance-side, PARK/LEAVE checks, swept-path checks, HIVE-avoiding park route | G417 ramming, G402, G304, 10.5.4 |
| Autos | `A: tip first`, `B: second CELL` (with CELL A fallback), `Park Only`, for both alliances; route check on the Driver Station at INIT; park guard always ends in LEAVE+PARK | SWARM RP = 2 robots × (3+5) = 16; field calibration TIPS at 3 NECTAR + 3 POLLEN |
| TeleOp | Alliance pick at INIT if no AUTO, time-stamped AUTO→TeleOp handoff of HIVE state/inventory, rumble at 1:00 (NECTAR/FLOWER unlock) and 0:20 (park), human NECTAR allowance on screen, hold poses for the up CELL / LOADING ZONE / FLOWER | G410, G426, PARK |
| Config | `meta.season` (stale ADB overrides rejected), `field`, `hive`, `match`, `inventory`, camera extrinsics in inches/degrees; `m` = 180° rotation | Last season's pushed config would otherwise override everything |
| Calibration OpModes | Ballistics Calibration and Shot Timing Tuner aim at RED CELL A along a shared off-axis ray; per-ball export; tuner output now matches what the runtime reads | Fixed DECODE producer/consumer mismatch |

## 2. Tradeoffs and decisions

1. **Kept the DECODE hardware architecture as the baseline** instead of inventing a new robot. Everything new is config-driven; a turret-less robot works (`turret.enabled: false` makes the devices optional and gates on chassis heading).
2. **Shot tables are placeholders and refuse to auto-fire** (`calibrated: false`). Firing DECODE numbers at a 59.6 in, outward-tilted opening would be wrong; the alternative of guessing would look like it works and miss.
3. **Field size 141.5 in, not the manual's nominal 144.** CAD perimeter glass is at ±70.67 in and the zone tape/bottom rail at ±70.10 in; the team already measured 70.75 last season. Zones are taken from the CAD tape relative to the field centre, so the half-inch uncertainty doesn't move them.
4. **Approach gate ±60° off the opening axis.** ChatGPT's CAD path checks passed every yaw in ±60° at 24–36 in; a side position (~113°) worked only with steep 75° descents and ~0.8 in margin. Raise `max_off_axis_deg` only after measuring a steep, repeatable shooter.
5. **HIVE frame protected as a whole rectangle by default.** CAD shows the floor is open between the two foot bars, so robots can pass under a 30.6 in HIVE, but the A-frame legs lean inward to the pivots and aren't modelled; bases-only mode is a config switch with that caveat.
6. **Vision TIP detection by height, not tag pose.** Up openings sit ~59.6 in, down ~38.2 in: a 21 in gap is robust to range error, where a pose-based tilt estimate would be noisy. TIPS the camera misses are recorded by the operator.
7. **Feeding stays continuous while gates pass**, not one ball at a time: metering needs hardware (a gate/indexer) the robot may not have. A TIP detected mid-volley stops further feeding.
8. **"rpm" names kept** (`max_rpm`, `*_rpm` are encoder ticks/s): renaming touched ~80 sites in calibration code for no behaviour change; documented in config instead.
9. **`Robot.update()` order changed** to read → commands → decide (`ShotController`) → write (`Intake`, `Shooter`, `Turret`) so a target set this loop reaches hardware this loop.
10. **Handoff freshness (2 min)** applies to alliance confirmation, pose, HIVE state and inventory together. Without a fresh matching handoff, TeleOp uses its profile start pose and prompts B = RED / X = BLUE to confirm alliance.
11. **Camera pitch default 50°** (was horizontal): from the shooting poses the opening is 45–65° above the lens.
12. **No FLOWER scorer.** See §4.

## 3. ChatGPT's work: what was useful, what wasn't

Its project (`biobuzz-simulation`) has two parts: an engineering study (CAD extraction, flight model, MuJoCo HIVE contact model, queue/match model) and a static code review of an older DECODE commit.

**Used directly**
- **CAD extraction** (`outputs/cad/parts.json`): CELL opening geometry, HIVE pivot, frame foot bars, FLOWER ring positions, zone tape. Better than reading figures.
- **Its flag that the HIVE bottom height disagreed** (manual 25.5 in vs CAD 30.65 in) was right: V1 had a typo; TU05 says 30.6 in.
- **Official calibration facts it extracted** (TIP at 8 POLLEN or 3 POLLEN + 3 NECTAR) — the basis for "the pre-load alone TIPS CELL A".
- **Code-review findings I confirmed and fixed:** shot-timing tuner/runtime format mismatch; stale solution reused after a new shot/target; intake writing hardware outside `periodic()`; stale ADB config overriding new defaults; disabled turret always reporting "aimed"; vision pose reset unsuitable for moving tags; no inventory model for G407.
- **Launch-model numbers** as seeds only: lob flight time (~0.63 s at 24 in → 0.80 s at 60 in) and placeholder table shape; ±60° off-axis band.

**Not used / out of date**
- The code plan reviewed commit `c54e1a0` (Pedro 2.1.2, SDK 11.2). Its P0 "SDK 12 migration" and P2 "Pedro 3 migration" were already done in this repo; its "mutable Pose" concern doesn't apply to Pedro 3's immutable `Pose`.
- **MuJoCo TIP dynamics**: by its own admission unconverged (tip time moved 3.29 → 3.54 → 3.25 s with time step) on invented contact parameters. Its packet "tip fractions" aren't predictions. The code observes TIPS instead of predicting them.
- **Queue/match scoring model** (7–13 tips per match): driven by assumed cycle times; not actionable for code.
- **Launcher power, wheel inertia, packaging**: useful for the mechanical team, not code.
- It used manual **V1**; TU05 differences are minor (pre-loads must be on your own side, sort-and-discard explicitly scrutinised under G407/G408, NECTAR re-entry) and only strengthen its "avoid blind intake + sort" advice.
- Its 55° descent target is a choice, not a requirement: the opening faces 30° up, so a ball descending at 30° enters square-on; steeper also works.

## 4. Biggest open items (need the team)

1. **FLOWER scoring** is the largest unaddressed points source: topping a FLOWER that still holds its 4 POLLEN with your NECTAR is 2 × 5 + 5 (bottom bonus) = 15 points, up to 4 FLOWERS, in the last minute. It needs a placement mechanism (4 in opening vs 3.6 in NECTAR). Note the tension: pulling FLOWER POLLEN for HIVE shots lowers what an owned FLOWER is worth.
2. **Calibrate** shot tables (both balls), shot windows, camera extrinsics, and the drivetrain if it changed.
3. **Hardware decisions**: intake-entry sensor (G407), color sensor (G408 / auto ball type), turret or not, steeper hood (>65°).

## 5. Aiming review (turret, shot lead, heading lock)

Last season's turret instability had concrete causes, reproduced in a closed-loop simulation with the plant model from `TurretLoopDiagnosticTest` (120 deg/s per unit power, 0.06 stiction, 20 ms loops):

| Finding | Effect (simulated, old code) | Fix |
| --- | --- | --- |
| `isAimed` compared against the travel-clamped target | CELL 60 deg off: turret parks at its stop and reports **aimed**, so it feeds ~35 deg off target | Aim is judged against the unclamped angle; the gate says "outside turret travel"; aim-and-shoot turns the chassis to face the CELL when the turret can't reach it (if the swing clears the HIVE) |
| Pedro's `PIDController` integral is unclamped and its first `calculate` integrates the whole time since construction or the last call | First aim after 3 s idle overshot by **9 deg** | P/D only in Pedro's controller; I term in the clamped `AntiWindupIntegrator`; the loop restarts from rest after any 100 ms gap |
| No feedforward for chassis rotation | **~10 deg lag** behind a 30 deg/s turn, never aimed while turning | `turret.kv` (kS·sign(v) + kV·v) from Pinpoint's angular and linear velocity: **0.6 deg lag** with kv = 1/K |
| Hard on/off settle band, same band for the shot gate | Hunting and gate flicker at the tolerance edge | Settle hysteresis (`tolerance.exit_factor`) shared by the controller and `isAimed` |
| Stall fault latched for the rest of the OpMode (nothing called `clearFault`) | One ball jam = no turret for the match | Stall faults auto-clear after `stall.retry_after_sec` |
| Lead switched on in one step at `lead_min_speed_ips`, and dropped to the raw bearing whenever the solution went invalid (speed/range gate, uncalibrated table) | Turret jumped by the whole lead angle (>10 deg at 10 in/s with ~0.7 s lob flights) | Lead fades in linearly; the turret keeps the solver's azimuth whenever the solution is current, valid or not |
| Bearings and shot distance from the robot centre, not the turret axis | 6-9 deg aim error for a 4 in offset at 24-36 in | `turret.pivot_forward_in` / `pivot_left_in` (0 until measured); calibration OpModes measure from the pivot too |
| Heading lock: friction step saturates the output at the deadband edge (a relay), and the measured braking model coasts ~4 deg at 1.2 rad/s, the full ±2 deg deadband | Chassis rings around its target; a turret aiming at "bearing − heading" chases it | The lock stops pushing once `pedro.braking.heading` predicts the chassis will coast onto the target |
| `tolerance.max_drift` 5 in / `max_deg` 2 were DECODE goal numbers | Needlessly tight for a 20 in opening | 8 in / 4 deg, sized from the opening minus a NECTAR |

Still to do on hardware: measure `kv`, the pivot offset and re-centre the servo neutral (see `BIOBUZZ-SEASON.md`).

## 6. Other bugs (2026-10-19)

The continued review fixed these issues, with JVM regressions:

| Bug | Fix | Regression |
| --- | --- | --- |
| Blackboard alliance and pose survived into later matches | Pure `OpModeUtil.resolveTeleOpStart` trusts them only with a fresh matching handoff; otherwise prompts alliance confirmation and uses the profile start. INIT telemetry identifies the pose source. | `OpModeUtilTest.teleOpDiscardsStaleAllianceAndPoseTogether` / `teleOpRestoresAllianceAndPoseOnlyWithMatchingFreshHandoff` |
| Hood moved during INIT (G403) | Constructor keeps the hood target; only `periodic()` writes it after START. Constructor audit also found Lumos enabling PWM: its PWM setup and color writes now run from `periodic()`. Intake and turret constructors set no actuator power/position. | `ShooterWriterTest.hoodWaitsForPeriodicAndSettersNeverWriteHardware`, `LumosWriterTest.constructorAndColorTargetDoNotEnablePwm` |
| Repeated flywheel velocity commands wasted hub transactions, including idle | Cache each motor's velocity command, invalidate on mode changes and explicit stop; `stop()` now requests a target instead of bypassing the writer. | `ShooterWriterTest.unchangedVelocityIsWrittenOncePerMotorIncludingIdle` / `modeChangesInvalidateEachMotorsVelocityCache` |
| Braking coefficients were fixed to field axes | Transform velocity into the chassis frame, apply forward/strafe stopping models, rotate the stop vector back, and use its projected magnitudes symmetrically. Heading zero retains the old numbers. | `CasablancaBrakingFrameTest` (zero, ±90°, diagonal, both protection axes) |
| AUTO priming and shot-window lookup measured from the robot centre | Both now measure from the turret pivot, matching the live solver. | `ShotPivotDistanceTest.initialSolveAndAutoWindowUseTheTurretPivot` |
| A commanded 12% speed drop counted a phantom ball | Reproduced before changing code. Re-arm detection within the new speed window; preserve the volley count and keep detecting real dips. | `ShotCountingTest.commandedSpeedDropDoesNotCountAPhantomBall` |
| Undo with zero TIPS selected the downward CELL | Ignore undo when there is no TIP to undo, preserving CELL, epoch and settle state. | `HiveTrackerTest.testUndoPastTheFirstTipCannotSelectTheDownwardCell` and strengthened zero-count test |
| Manual rotation checked only the lookahead endpoint | Reuse the swept-rotation check so an intermediate corner collision is rejected even when both endpoints clear the HIVE. | `RotationSweepSafetyTest.lookaheadRejectsAnIntermediateCollisionEvenWhenBothEndsAreClear` |
| Invalid localization could produce a valid NaN shot, keep a previous shot feeding, or send NaN turret power | Reject non-finite solver/table inputs, close feeding immediately on a missing/non-finite pose or velocity, and require a fresh solve after recovery. Turret control cuts power and resets before invalid inputs poison PID state; the hardware writer also rejects non-finite commands. | `ShotSolverTest.testNonFinitePoseVelocityAndTargetNeverYieldAValidShot`, `ShotTableTest.testNonFiniteLookupNeverReturnsACalibratedShot`, `ShotInputSafetyTest.localizationFaultImmediatelyBlocksAnAlreadyValidShot`, `TurretAimingStabilityTest.invalidLocalizationCutsPowerAndControlRecoversWithoutReinitializing` / `nonFiniteManualPowerNeverReachesTheServo` |
| An in-place pose assist checked only its start and end footprints | Sample the entire turn through `Sentinel.rotationProblem`, checking both the HIVE and field walls before holding the target heading. | `PoseHoldSafetyTest.inPlaceHoldRejectsIntermediateHiveAndWallCollisions` |
| Unknown/missing inventory handoffs became known-empty with an entry sensor fitted | Explicitly restore the unknown total independently of whether an entry sensor exists; new entry readings cannot reconstruct an unknown starting total. Driver corrections and known handoffs still establish a known count. | `InventoryHandoffTest.missingOrUnknownHandoffCannotBecomeKnownEmptyWithASensor` |

The additional reviews reproduced undo-at-zero, both rotation gaps, invalid-localization handling and unknown-inventory restoration with failing regressions before their fixes. Existing flywheel, turret and Casablanca math tests remain passing.

## 7. Verification

`./gradlew verifyBuild` **passed** on 2026-10-18 (re-run 2026-10-19 after the aiming and continued reviews): config generation, Spotless formatting, Java compilation (including Error Prone/NullAway), config-key symmetry, and local JVM tests. Of 368 tests, **366 passed, 0 failed, and 2 were skipped**. This adds 21 tests to the 347-test baseline. The skipped tests are the existing `TurretLoopDiagnosticTest` watchdog diagnostics (`runawayWatchdogToleratesNormalConvergence` and `runawayWatchdogCutsPowerWhenErrorGrowsUnderPower`), gated on `turret.runaway.enabled`. Compiler/deprecation warnings remain; they did not fail verification. `git diff --check` also passed.

The continuation completed the interrupted test rewrite and fixed the issues exposed by verification and review:

- **Config binding:** `score_b_cp` now binds to `scoreBCp`; generated autonomous global fields preserve their YAML names so `loadMerged` can populate them. Either issue could otherwise leave autonomous settings missing at INIT.
- **Alliance safety:** the centre-line margin applies only at the centre line, preserving legal wall starts. Footprint containment tolerates floating-point rotation error at exact field boundaries. BLUE park waypoints rotate their heading once.
- **Autonomous paths:** the CELL A shortcut with collection disabled crossed the HIVE frame; it now follows the clear FLOWER-side lane with collection off. `AutoRouteSafetyTest` checks every configured path for both alliances and emergency parking from 21 sampled positions along each path. Failed INIT validation blocks the routine but still drives a checked LEAVE + PARK route from the start if one is clear; a shot-table parse error is a warning only. Zero-length parking legs no longer construct an invalid Pedro line.
- **TeleOp pose assists:** shooting, park and FLOWER holds refuse a direct approach that intersects the HIVE frame or leaves the field. The `Pose assist` telemetry explains the refusal; the driver can reposition and retry.
- **Vision:** multiple sightings from one frame cannot count as multiple TIP confirmations. Old/out-of-order or future-dated observations are rejected, and relocalization also requires a fresh frame.
- **Shot lifecycle:** solver generations prevent an in-flight result from a previous shot or a previous ball-type selection from being published after invalidation. A deterministic concurrency test covers shot restart and POLLEN→NECTAR→POLLEN changes.
- **Inventory:** requesting collection while the known preload is full keeps the count known and the motors stopped, including without an entry sensor. A regression test covers the single-writer/cap interaction.
- **Shot range:** the minimum valid range now matches the placeholder tables' first row (24 in). TeleOp's fixed-power shot still requires a valid current solution and a settled HIVE; calibration OpModes provide the intentional measurement bypass.

Review of that continuation (2026-10-18): every fix above was checked against the code and is correct, including two bugs in the original port (`score_b_cp` never bound because the old snake-case conversion produced `score_bcp`; wall-touching starts failed the alliance-side check). Follow-ups added on review:

- **Calibration ray** kept only 11 in from the walls regardless of heading, so the far end of the ray put the robot's corner past the audience wall, and an 18 in endpoint touched the HIVE frame. Clearance now uses the footprint's half-diagonal; calibration distances run 24–48 in (the ray reaches ~52 in).
- **Blocked AUTO** now still takes a checked LEAVE + PARK instead of standing still (8 points plus half of the SWARM ranking point).
- **New tests** for things the report claimed as fixes but nothing exercised: the stale-season config guard and alliance transform (`ConfigSeasonGuardTest`), the distance-keyed shot-timing table (`ShotTimeTableTest`), the calibration ray (`CalibrationRayTest`), and every volley pose facing its CELL within turret reach (`AutoRouteSafetyTest`).

These are software checks against configured geometry and mocked hardware. Path checks sample the planned footprint; they do not establish physical tracking accuracy or stopping performance. Changes remain uncommitted on `biobuzz-season-upgrade`.

Not verified: anything on hardware or a real field — camera detection ranges, TIP detection latency, shot accuracy, path following, and the 141.5 in field assumption.
