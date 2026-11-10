# CalderDrones (FTC team 21024): robot code

Robot code for FTC team 21024, CalderDrones, for the BIOBUZZ 2026-27 season. It is Java for the FTC SDK 12, built on Pedro Pathing 3 with Ivy commands, Panels for live tuning, and a Limelight 3A for vision. The drivetrain is a custom 3-wheel Kiwi.

The code runs on the real robot and in the team's PC simulator (the BIOBUZZ sim, which is not in this repository).

The SDK's own README, with its release history, is kept at [doc/FTC_SDK_README.md](doc/FTC_SDK_README.md).

## Overview

The robot code is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. It covers:

- **Drive**: a Kiwi drivetrain driven by Pedro Pathing odometry (Pinpoint), with TeleOp drive controls and auto-collect.
- **Launchers**: two flywheels (Pollen and Nectar), each with its own feed gate, and RPM tables worked out from camera distance.
- **Vision**: a Limelight 3A on a tilt mount, finding the HIVE AprilTags to aim at and balls on the floor.
- **Commands**: Ivy commands for driving, launching, collecting and aiming, with requirements so they don't fight over the same parts.
- **OpModes**: the Competition TeleOp, test TeleOps, tuning OpModes and autonomous routines (see [OpModes](#opmodes)).
- **Sim support**: the Looping Autos are sim-only stress tests, and the BIOBUZZ sim applies its own drive values to `pedro/Constants.java` from its `vhub/sim-overrides.txt`.

Packages under `teamcode/`:

| Package | Contents |
| --- | --- |
| `commands` | Ivy commands for driving, launching and robot actions, plus their requirements and priorities |
| `drive` | `TeleOpDrive` (TeleOp drive controls and heading lock) and `CollectAssist` (auto-collect) |
| `flywheel` | `Flywheel`, velocity control for a launcher, with fault detection |
| `gates` | `Gates`, the Pollen and Nectar feed servos |
| `helpers` | `BulkCache`, `Devices` (safe hardware lookup), `MatchState` (alliance and pose handed from auto to TeleOp), `SystemStatus` |
| `intake` | `Intake`, the intake motor and pickup detection from motor current |
| `kiwi` | `Kiwi` drivetrain and its `KiwiConfig` |
| `launch` | RPM tables, the flight model, scoring positions and shot quality |
| `leds` | `Leds` (REV Blinkin) and `RobotState` |
| `opmodes` | `auto` (the Looping Autos and `TestAuto`) and `teleop` (Competition, test and tuning OpModes) |
| `pedro` | Pedro Pathing `Constants` (drivetrain, localizer, Foresight) and `Tuning`, the tuner entry points; `pedro/procedures` holds the tuner procedures |
| `vision` | `LimelightCamera`, `HiveTarget`, `HiveAim`, `LimelightConstants`, `FloorBall`, `PoseHistory` |

Other folders: `TeamCode/limelight/floor_balls.py` is the Limelight Python pipeline for balls on the floor. `TeamCode/src/test` holds the PC unit tests.

## OpModes

Names are as they appear on the Driver Station. "Robot" means the real robot, "Sim" the PC simulator's stress tests, and "Tuning" the setup and tuning OpModes.

| Driver Station name | Group | Class | Purpose | For |
| --- | --- | --- | --- | --- |
| Competition TeleOp | TeleOp | `CompetitionTeleOp` | Match TeleOp for two drivers (driver and gunner): drive, collect, launch, aim | Robot |
| More Than Driving Test | TeleOp | `MoreThanDrivingTestTeleOp` | The same launching and driving on one gamepad | Robot |
| Driving Test | TeleOp | `DrivingTestTeleOp` | Drive controls only (`TeleOpDrive`) | Robot |
| Test Auto | Autonomous | `TestAuto` | Drives path 1, aims at our HIVE and fires four Pollen | Robot |
| SIM Red 1 Looping Auto | Sim | `Red1LoopingAuto` | Sim-only stress test: loops collect and shoot with no drivers | Sim |
| SIM Red 2 Looping Auto | Sim | `Red2LoopingAuto` | As above, Red 2 station | Sim |
| SIM Blue 1 Looping Auto | Sim | `Blue1LoopingAuto` | As above, Blue 1 station | Sim |
| SIM Blue 2 Looping Auto | Sim | `Blue2LoopingAuto` | As above, Blue 2 station | Sim |
| Fly Me To The Wheel | Tuning | `FlyMeToTheWheel` | Holds one flywheel at a target RPM | Tuning |
| Flywheel Tuner | Tuning | `FlywheelTuner` | Feedforward, step and shot tests for a flywheel's gains | Tuning |

The Looping Autos share their code in `LoopingAuto`, and each station's class only sets its alliance and starting spot.

The Pedro tuners are registered with `@Tuner` in `pedro/Tuning.java`, so they appear in the tuning menu rather than the Driver Station list: **Kiwi Tuner**, **Pinpoint Tuner**, **Foresight Tuner** and **Tests**. Other tuner procedures exist in `pedro/procedures` (OTOS, OctoQuad, three-wheel, two-wheel, mecanum and the three-wheel IMU), but they are not registered in `Tuning.java`.

## Robot configuration names

Each name below must match the robot configuration exactly. Most parts check for their devices at INIT and disable only themselves if one is missing. Missing devices are listed on telemetry as `... DISABLED: "name" not found` (see `helpers/Devices.java` and `helpers/SystemStatus.java`). The rest of the robot keeps running.

| Name | Device type | Notes | If missing |
| --- | --- | --- | --- |
| `MotorA` | DC motor | Kiwi wheel A, the rear wheel that drives left and right. Reversed in `pedro/Constants.java` | Drive is disabled |
| `MotorB` | DC motor | Kiwi wheel B, front left. Reversed | Drive is disabled |
| `MotorC` | DC motor | Kiwi wheel C, front right. Reversed | Drive is disabled |
| `pinpoint` | GoBilda Pinpoint odometry computer (Pedro `PinpointLocalizer`) | Localizer for the Pedro follower. Set as `c.name` in the ROBOT set in `pedro/Constants.java`: check it matches your configuration | Drive is disabled |
| `PollenFlywheel` | DC motor (`DcMotorEx`) | Pollen launcher, direction reversed. Speed comes from the encoder | Pollen launcher is disabled: reads 0 RPM and is never ready |
| `NectarFlywheel` | DC motor (`DcMotorEx`) | Nectar launcher, direction forward | Nectar launcher is disabled, as above |
| `pollenGate` | Servo | Pollen feed gate. Open for `Gates.feedMs` per ball, at `Gates.pollenOpenPosition` / `pollenClosedPosition` | Pollen gate is disabled and never opens |
| `nectarGate` | Servo | Nectar feed gate, at `Gates.nectarOpenPosition` / `nectarClosedPosition` | Nectar gate is disabled and never opens |
| `intake` | DC motor (`DcMotorEx`) | Intake roller, direction forward, power 1.0. The code notes this is a placeholder until the intake is built. A CR servo on a motor port also works | Intake is disabled and never reports a pickup |
| `limelight` | Limelight 3A (`Limelight3A`) | Vision. Pipeline 0 is AprilTags and pipeline 1 is the floor pipeline | Vision is disabled: no tags are seen, and the flywheels use the default speed |
| `cameraTilt` | Servo | Swings the Limelight between shooting and intake pitch | Camera stays at the shooting pitch and FLOOR mode is unavailable |
| `blinkin` | REV Blinkin LED Driver | Plugged into a servo port, and configured as "REV Blinkin LED Driver". The strip length (78 LEDs) is stored on the Blinkin itself | LEDs are disabled and the calls do nothing |

The hub is found with `hardwareMap.getAll(LynxModule.class)` for bulk reads, so it has no name to configure.

Tuners ask for some names at run time, with these defaults: `PollenFlywheel` (Fly Me To The Wheel) and `Launcher` (Flywheel Tuner), both set in Panels, `motorA`, `motorB` and `motorC` (Kiwi Tuner), `imu` (the IMU tuners), `pinpoint` (Pinpoint Tuner) and `otos` (OTOS Tuner).

## Controls

### Competition TeleOp

Gamepad 1 is the driver and gamepad 2 the gunner.

| Driver (gamepad 1) | Action |
| --- | --- |
| Left stick | Drive |
| Right stick | Turn (cancels the gunner's aim) |
| L1 / R1 held | Crawl / sprint (otherwise walk) |
| Circle | Toggle field-centric and robot-centric |
| Cross | Auto-collect the nearest Pollen or Nectar of ours. Press again, or move a stick, to cancel. Also lowers the camera |
| R2 | Intake in, on or off |
| L2 held | Intake out |
| Options held (0.5 s) | Re-zero the field heading |
| Triangle | Cancel everything |

| Gunner (gamepad 2) | Action |
| --- | --- |
| R2 / L2 | Launch Pollen / Nectar. The launch is queued until the launcher is ready. Press again to cancel |
| R1 held | Fire everything |
| Square | Auto-aim on or off. Launches wait for the aim lock when auto-aim is on |
| D-pad up / down | Camera to shooting / collecting position |

INIT:

- D-pad left / right on either gamepad picks red or blue. If an auto has run since the app started, its alliance is used.
- The start pose is the one the last auto saved (`MatchState`). If no auto has run, the default pose is used. Triangle on either gamepad switches between the saved pose and the default; telemetry shows how old the saved pose is. With no press the saved pose is used, however old.
- The flywheels idle at `idleRPM` unless a launch, Fire All or aim is active.

### More Than Driving Test

One gamepad. Drive controls are the same as `TeleOpDrive`'s (heading lock, circle for robot/field-centric, bumpers for slow and sprint), plus:

| Button | Action |
| --- | --- |
| Square | Auto-aim at our HIVE until the right stick is turned |
| Cross | Collect the nearest Pollen or Nectar of ours. Press again, or move a stick, to cancel |
| Right trigger | Feed a Pollen (only when its flywheel is at speed) |
| Left trigger | Feed a Nectar (only when its flywheel is at speed) |

The flywheels always run, at the RPM for the camera's distance to the CELL, or at the default speed when no tags are in view. The LEDs show how good the robot's position is for scoring. Which HIVE to aim at (red or blue) is set in Panels.

## Setup

### Building and deploying

1. Open the project in Android Studio and let Gradle sync.
2. Connect the Robot Controller phone or Control Hub.
3. Build and run the `TeamCode` module on it with Android Studio's Run button.

Fast deploy with Sloth: the `deploySloth` run configuration (`.run/deploySloth.run.xml`) runs the Gradle task `:TeamCode:deploySloth`. `TeamCode/build.gradle` applies the Sloth load plugin (`dev.frozenmilk:Load`), which adds the `deploySloth` and `removeSlothRemote` tasks. From a shell:

```bash
./gradlew :TeamCode:deploySloth
```

The project uses Java 17.

### Panels

Panels is the live dashboard for the robot. Values marked `@Configurable` in the code (for example `CompetitionTeleOp.alliance` or `Gates.feedMs`) can be changed in Panels and take effect without rebuilding. Telemetry and graphs also show in Panels. Tune values there, then copy the final values into the code.

### Drive constants and the sim

`pedro/Constants.java` holds the real robot's drive constants. The Pinpoint values marked TODO and the Foresight gains are placeholders until the robot is tuned. Until then `Constants.foresightTuned` is false, and the TeleOp heading lock uses PID instead of Foresight. Set it to true once the Foresight tuner's results are in.

The BIOBUZZ sim ([LEdgeley/BioBuzzSim](https://github.com/LEdgeley/BioBuzzSim)) keeps its own values out of TeamCode: its `vhub/sim-overrides.txt` sets them on `Constants`' public static fields by name, for example `Constants.foresightTuned = true`. Keep the names `drivetrainConfig`, `localizerConfig`, `foresightConfig` and `foresightTuned` as they are; `ConstantsTest` fails if one is renamed or made final.

### Limelight pipelines

The Limelight needs two pipelines, set up on its web page:

- **Pipeline 0** (`LimelightConstants.aprilTagPipeline`): AprilTags, family 36h11, tag size 82.5 mm.
- **Pipeline 1** (`LimelightConstants.floorPipeline`): the Python pipeline. Paste in `TeamCode/limelight/floor_balls.py`. Its HSV ranges are placeholders to tune under field-like lighting.

### Unit tests

The PC unit tests are in `TeamCode/src/test` and use JUnit 4 and Mockito. They run on the PC, not on the robot.

From Android Studio, run the `testDebugUnitTest` task for the `TeamCode` module from the Gradle tool window, or run a test class from `TeamCode/src/test` directly.

From a shell, Gradle needs `JAVA_HOME` set to Android Studio's JBR, and this machine needs `JAVA_TOOL_OPTIONS` as well. In Git Bash:

```bash
JAVA_HOME="/c/Program Files/Android/Android Studio/jbr" JAVA_TOOL_OPTIONS=-Djdk.net.unixdomain.tmpdir=C:/Users/Public ./gradlew :TeamCode:testDebugUnitTest
```

In PowerShell:

```powershell
$env:JAVA_HOME = "C:\Program Files\Android\Android Studio\jbr"
$env:JAVA_TOOL_OPTIONS = "-Djdk.net.unixdomain.tmpdir=C:\Users\Public"
.\gradlew :TeamCode:testDebugUnitTest
```

Change the JBR path if Android Studio is installed somewhere else.

## More

The SDK's own README, with its release history, is at [doc/FTC_SDK_README.md](doc/FTC_SDK_README.md).
