# BIOBUZZ season notes

Robot code for BIOBUZZ presented by RTX (2026-2027), Competition Manual TU05. This file maps the game rules onto the code and lists what must be measured on the real robot before a match. Driver controls are in `teleop_controls.md`.

## Field frame

Pedro coordinates, origin at the corner where the audience wall meets the RED alliance wall, +x toward BLUE, +y away from the audience, heading 0 = +x, CCW positive. The field is `field.size_in` = 141.5 in inside the walls (centre 70.75). The field is **180-degree rotationally symmetric**, not mirror symmetric, so every pose and zone is written for RED and BLUE uses `m` (a half-turn about the centre).

| Feature (RED) | Where | Source |
| --- | --- | --- |
| RED HIVE pivot | (58.0, 70.75), 43.95 in up | manual Fig 9-8/9-10 (HIVE centres 25.5 in apart) |
| CELL A opening (starts UP) | (58.0, 55.06), 59.56 in up, faces the audience (-90 deg) | CAD; SDK cluster origin, tags 34-37 |
| CELL B opening (starts down) | (58.0, 86.44) when up, faces the rear wall | CAD; tags 30-33 |
| HIVE frame footprint | x 46.0-95.5, y 51.3-90.2 (both HIVES) | manual 49.46 x 38.95 in; CAD foot bars |
| LOADING ZONE | x 0-11, y 94.66-117.35 | CAD tape |
| GARDEN | x 0-22.69, y 0-2 | CAD tape |
| Alliance-side FLOWERS | (2.65, 47.36) and (47.36, 138.85) | CAD top rings |

BLUE's CELL A is its rear CELL (tags 42-45), the rotation of RED's CELL A. The manual's V1 "bottom of HIVE 25.5 in" was a typo fixed in TU05 (30.6 in), matching the CAD.

## Rules the code enforces or assists

| Rule | Code |
| --- | --- |
| Only launch into an upward CELL (G417); TIP state flips the target | `HiveTracker` (vision + operator), `ShotController` epoch gate, `AllianceAutoBase.volley` never fires at a down CELL |
| Pause launching while the HIVE tips (manual 10.5.1) | `hive.settle_ms` feed lockout |
| Don't ram the HIVE frame (G417) | `Sentinel.getProtectedZones()` + `Casablanca` slow-down/repulsion |
| Max 4 SCORING ELEMENTS controlled (G407) | `BallInventory` cap in `Intake.periodic()` (needs the optional entry sensor); operator eject |
| Opponent NECTAR (G408) | Operator eject (no color sensor in the baseline) |
| AUTO: stay on your side (G402) | Every configured path checked by `AutoRouteSafetyTest` with every APK build and before every `pushConfig` |
| Start: touching the wall, own side, not in LOADING ZONE (G304) | Configured start poses checked by `AutoRouteSafetyTest` (build and `pushConfig`); placing the robot is the drive team's job |
| LEAVE + AUTO PARK (SWARM RP = both robots, 16 pts) | Park guard ends every routine partly in the LOADING ZONE, off the wall. It starts parking at the estimated drive from wherever the robot is (stop, then the route around the HIVE) + 2.5 s (`auto.park_margin_ms`), and never later than 3 s before the end (`auto.park_reserve_ms`); the Driver Station shows the current deadline |
| NECTAR into FLOWERS / all human NECTAR only in the last minute (G410, G426) | `MatchClock` rumble + telemetry; human NECTAR allowance (1 per TIP) shown |
| PARK at the end of TELEOP | Park alert from 0:20: once the time left is the estimated drive to park + 4 s, rumble, red gamepad light, "park now" voice and PARK NOW telemetry until parked; driver park hold |

## Autonomous routines

| OpMode | Start (RED) | Plan |
| --- | --- | --- |
| `Red/Blue A: tip first` | (58, 9) on the audience wall facing CELL A | 4 POLLEN into CELL A (it already holds 3 NECTAR; the field is calibrated to TIP at 3 NECTAR + 3 POLLEN), around the HIVE past the side-wall FLOWER (refilling there only if `cell_a_collect_flower`; off on this robot) to an angled CELL B pose, volley any leftover POLLEN if CELL B is up, park low in the LOADING ZONE |
| `Red/Blue B: second CELL` | (30, 132.5) on the rear wall | Wait at the CELL B pose for the partner's TIP, volley the pre-load (a rear-wall FLOWER refill and second volley only if `cell_b_collect_flower`; off on this robot), park high in the LOADING ZONE. If CELL B never comes up: detour to CELL A and empty the pre-load there |
| `Red/Blue Park Only` | (9, 75) on the alliance wall | LEAVE + PARK |

Pick complementary routines with your alliance partner (one A, one B). Toggles: `auto.cell_a_collect_flower`, `auto.cell_a_second_volley`, `auto.cell_b_collect_flower`. Routes are not checked on the robot: `AutoRouteSafetyTest` rejects any configured route that would cross the centre line, hit the HIVE frame or fail LEAVE/PARK, both when the APK is built and when `pushConfig` runs (`checkPushConfig`), so such a config never reaches the robot. A config copied onto the Control Hub some other way (plain `adb push`) is not checked. A shot-table parse error is shown at INIT as a warning and does not block driving. Disabling FLOWER collection in the CELL A routine still uses the same clear lane with the intake off.

## Bring-up checklist (hardware decisions first)

1. **Decide the BIOBUZZ hardware.** The code keeps the DECODE robot's architecture (mecanum + Pinpoint, two intake motors, two flywheel motors, hood servo, CR turret with analog encoder, webcam). If the robot has no turret, set `turret.enabled: false`: the turret devices become optional and the shot gate checks chassis heading instead.
2. **Camera mount.** CELL tags are 30-60 in up; from the shooting poses the opening is ~45-65 degrees above the robot. Mount the webcam pitched up (default 50 deg), then measure `vision.camera.*`. Verify on the field that the HIVE telemetry flips on a TIP.
3. **Intake-entry sensor** (recommended, G407): wire a beam break as `intakeBeam`, set `inventory.entry_sensor_enabled: true`.
4. **Turret.** Run Turret Plant Calibration and paste its `ks_positive`, `ks_negative` and `kv` lines. `kv` turns on the chassis-rate feedforward: in simulation it cut aim lag behind a 30 deg/s chassis turn from ~10 deg to under 1 deg. Measure the turret axis offset from the robot centre into `turret.pivot_forward_in` / `pivot_left_in`. Last season's breakaway powers (+0.0009 / -0.0663) mean the CR servo's neutral sits about 0.033 off centre, so zero power leaves it on the edge of creeping; re-centre the servo's PWM neutral (servo programmer) and re-measure. The calibration OpMode recommends `pidf.i: 0`; the config keeps the old 0.008, now clamped.
5. **Drivetrain.** If the chassis changed, re-run Pedro AutoTune (`MIGRATION-PEDRO3.md`). The `pedro.*` values are last season's.
6. **Shooter.** POLLEN (2.8 in, ~25 g) and NECTAR (3.6 in, ~41 g) need steep lobs: a launch-model estimate is 64-78 deg elevation at 24-60 in, above the DECODE hood's 65 deg maximum. Re-run Shooter Characterization / Autotune if the flywheel changed.
7. **Shot tables.** Run Ballistics Calibration (RED CELL A up, robot at `teleop.poses.red.shoot_a`) for POLLEN. Paste the rows into `shooter.shot_table.pollen_points`, then set `calibrated: true`. Until then TeleOp and autonomous shots never feed, including the fixed-power TeleOp shot. The calibration OpModes provide the deliberate fixed-power bypass for collecting measurements.
8. **Shot windows.** Run Shot Timing Tuner; paste its `shot_time_points` under `auto:`.
9. **Field check.** Drive every routine slowly on a real field; adjust RED poses only (BLUE follows) unless BLUE measures differently, in which case replace that BLUE `m` with explicit values.
10. **Clear old configs.** A DECODE `config.yaml` pushed to the Control Hub is ignored automatically (season tag), but `./gradlew resetConfig` removes it.

TeleOp pose holds check the direct approach for HIVE and wall clearance. A blocked approach stops and reports the reason in `Pose assist` telemetry; reposition manually before retrying.

## Known gaps

- No FLOWER mechanism: the robot can neither take POLLEN from a FLOWER nor score into one. `auto.cell_a_collect_flower`, `auto.cell_b_collect_flower` and `teleop.flower_hold_enabled` are false, so each auto fires only its 4 pre-loads. Ownership is worth 2 points per element plus the 5-point bottom NECTAR bonus, but the 4 in opening leaves ~0.2 in around a 3.6 in NECTAR; it needs a dedicated mechanism decision.
- No NECTAR mechanism: the robot cannot intake or launch NECTAR, so `shooter.shot_table.nectar_enabled` is false and every shot uses the POLLEN table. AUTO is unaffected: pre-loads, FLOWERS and GARDENS start with POLLEN only (manual 10.3.1). To add NECTAR later, calibrate `nectar_points` (right bumper at Ballistics Calibration INIT) and set `nectar_enabled: true`.
- No color sensing: opponent NECTAR (G408) is ejected manually.
- Feeding is continuous while every gate passes (not one ball at a time). A TIP detected mid-volley stops further feeding, but balls already in the air cannot be recalled.
- Hood settling is not measured (the servo readback is the command), and a flywheel setpoint clamped by `max_shot_rpm` is not re-validated against the table.
- Moving shots use first-order lead only; `max_moving_speed_ips` is a conservative 20.
