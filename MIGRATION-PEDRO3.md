# Pedro 3 migration notes

This branch merges Pedro Pathing QuickStart v3.0.1 with the team's existing `decode` code. It preserves both histories, so future QuickStart updates can be merged from `upstream`. The DECODE game code has since been replaced for BIOBUZZ; see `BIOBUZZ-SEASON.md`.

## What changed

- FTC SDK dependencies are 12.0.0. The repository keeps its existing Android Gradle Plugin, Gradle wrapper, Java 21, formatting, config compiler, and safety tests.
- Pedro is `com.pedropathing:revhub:3.0.1`; Ivy is `com.pedropathing.ivy:pedro:1.1.1`; QuickStart's AutoTune dependency is retained.
- The old follower builder, pose, path chain, and PIDF APIs are replaced by Pedro 3 `Follower`, `Pose`, `Paths`, `Foresight`, and `PIDController`. The prior robot motor names, directions, and Pinpoint pod offsets are carried into the new follower factory.
- The old Pedro 2 tuning OpModes have been removed. The QuickStart `pedro/Tuning.java` now registers Mecanum, Pinpoint, Foresight, and test procedures for the robot-hosted AutoTune page.
- SDK 12's AprilTag detection types are handled for both single tags and tag clusters. The prior vision-to-field pose mapping is preserved and covered by a unit test.
- `Casablanca` retains its prior calibrated linear/quadratic stopping-distance model. Pedro 3 removed its standalone `PredictiveBrakingController`; this local safety calculation is tested on both velocity signs. Depth and side protection remain symmetric.

## Decisions and tradeoffs

- Foresight needs calibration data that Pedro 2 did not provide. Known heading gains, maximum forward/strafe speeds, and shared braking coefficients were carried over. Forward/strafe feedback gains, feedforward, heading braking, and natural deceleration are **provisional starting values** in `config.pedro`; they are not verified robot measurements. Foresight may follow paths differently until AutoTune is completed.
- The team's QuickStart custom build setup was retained because it supplies config generation, Spotless, Error Prone, and JVM tests. The SDK and Pedro dependencies are updated to the QuickStart 3.0.1 versions.
- `PoseFactory` is used to preserve the prior alliance mirror behavior across Pedro's removed `Pose.mirror()` method. Direct autonomous path helpers retain linear heading interpolation.
- The robot uses the QuickStart-style `pedro.Constants.create(hardwareMap)` entry point. Pedro 3's Mecanum class caches motor writes internally. Manual Lynx bulk-read cache clearing still happens in `Robot.update()`.

## Robot bring-up

1. Build and deploy the app on the robot with FTC SDK 12. Confirm drivetrain motor directions and Pinpoint X/Y signs and offsets before driving full speed.
2. Open Pedro AutoTune at `http://192.168.43.1:10158` while connected to the Robot Controller. Run the Mecanum and Pinpoint procedures, then Foresight. Enter measured Pedro values in `robot/config/config.yaml` and descriptions in `config-docs.yaml`, then run `./gradlew verifyBuild` and rebuild or use `./gradlew pushConfig`.
3. Check field-centric TeleOp direction, the goal-zone safety limiter, heading lock, AprilTag pose corrections, turret aim, and each autonomous route at low speed. Paths may complete and hold differently under Foresight; review end constraints and shot timing on the actual field.
4. BIOBUZZ geometry, poses, routines and placeholder shot tables are in place; see `BIOBUZZ-SEASON.md` for the BIOBUZZ bring-up checklist.

`./gradlew verifyBuild` passes locally. Physical robot behavior, AutoTune results, and AprilTag camera calibration have not been validated on hardware.
