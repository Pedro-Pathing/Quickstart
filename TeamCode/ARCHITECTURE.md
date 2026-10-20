# TeamCode guide

The robot code is organized by responsibility so OpModes stay small and hardware or control logic
can be reused between TeleOp and Autonomous.

## How the code fits together

| Package | Responsibility |
| --- | --- |
| `config` | Hardware-map names and tunable robot-wide values |
| `control` | Generic feedback controllers |
| `drive` | Hardware-independent drivetrain values and mecanum math |
| `localization` | Pinpoint odometry, Limelight AprilTag ranging, and pose models |
| `navigation` | Waypoints and field-to-robot pose control |
| `subsystems` | Classes that own and command physical robot hardware |
| `telemetry` | Driver Station and Dashboard presentation |
| `opmodes` | Match lifecycle and coordination of the other modules |
| `pedroPathing` | Pedro follower configuration, chassis adapter, and tuning suite |

`LimelightTeleOp` reads the gamepad and passes movement commands to `MecanumDrive`.
`LimelightAutoNav` passes each `Waypoint` and the current `PoseEstimate` to
`HolonomicPoseController`, then sends the resulting drive command to `MecanumDrive`.
`RobotLocalization` updates two independent components. `PinpointOdometryLocalizer` reads fused
translation and heading from the Pinpoint odometry computer; it is the only source of the
field pose used for navigation. `AprilTagTracker` uses Limelight only to measure each visible tag
relative to the robot; tag observations never modify the odometry pose.
`LocalizationTelemetry` displays both sets of data without coupling them to Driver Station or
Dashboard code.

`TeleTourney` and `Pedro Auto Nav` use Pedro's follower and `PinpointLocalizer` directly. Their
translation and heading come from `pinpoint`, with two pods and its onboard IMU. `Limelight TeleOp` and
`Limelight Auto Nav` retain the custom stack described above as separate OpModes.

## Robot configuration names

These names are case-sensitive and must exactly match the devices configured on the Robot
Controller. Change them in `RobotConfig.Hardware` if the configuration uses different names.

| Configured name | Type | Physical location and purpose | Software direction |
| --- | --- | --- | --- |
| `frontLeft` | Motor | Front-left mecanum drive wheel | Forward |
| `frontRight` | Motor | Front-right mecanum drive wheel | Reversed |
| `backLeft` | Motor | Rear-left mecanum drive wheel | Forward |
| `backRight` | Motor | Rear-right mecanum drive wheel | Reversed |
| `pinpoint` | goBILDA Pinpoint Odometry Computer | Control Hub I²C bus 1; forward pod on X, strafe pod on Y | Directions in `RobotConfig.Pinpoint` |
| `limelight` | Limelight 3A | Measures visible AprilTag range and relative height | Not applicable |
| `imu` | Control/Expansion Hub IMU | Used only by the standalone hub IMU test | Not applicable |
| `intake` | Motor | Intake mechanism | Controlled by A and Y |
| `Shoot1` | 6000 RPM motor with encoder | First shooter wheel | Controlled by X |
| `Shoot2` | 6000 RPM motor with encoder | Second shooter wheel | Controlled by X |
| `servoMotor` | Continuous-rotation servo | Auxiliary servo mechanism | Controlled by B |

The two right-side motors are reversed in `MecanumDrive`, allowing the same positive drive command
to move all four wheels forward. If a wheel spins the wrong way, first verify the physical motor
position and wiring before changing that direction rule.

The current installed drivetrain swaps and reverses the standard mecanum forward and strafe motor
patterns. `RobotConfig.Drive.SWAP_FORWARD_AND_STRAFE` compensates for that layout, so joystick up is
physical forward, joystick down is backward, joystick left strafes left, and joystick right strafes
right. Set it to `false` if the motor wiring or hub configuration is later changed to the standard
layout.

`RobotConfig.Drive.REVERSE_ROTATION` corrects the installed drivetrain's reversed turn direction.
The two trigger values are subtracted and passed through a signed-square response curve. This keeps
full-trigger rotation at full power while making partial-trigger rotation slower and smoother.

Both odometry pods now plug into the Pinpoint, freeing the two hub motor encoder ports. Configure
the Pinpoint as `pinpoint` on **Control Hub I²C bus 1**, device slot 0. Java hardware-map lookup does
not choose a bus; `RobotConfig.Hardware.PINPOINT_I2C_BUS` documents the required wiring. See
[Pinpoint setup](../doc/PINPOINT_SETUP.md) for wiring and the XML configuration entry.

Set the shared pod resolution, pod directions, and center offsets in `RobotConfig.Pinpoint`.
`FORWARD_POD_Y_INCHES` is positive left of center; `STRAFE_POD_X_INCHES` is positive ahead of
center. Both offsets are still zero and require measurement. At heading 0°,
positive X is forward and positive Y is left. The autonomous route currently resets its starting
pose to `(0, 0, 0°)`.

Limelight reports tag poses in meters; `AprilTagTracker` converts them to inches. For every visible
tag, it provides forward and left offset, horizontal distance, straight-line 3D range, and signed
height relative to the robot origin. Accurate height values require the Limelight's position and
orientation to be configured correctly in the Limelight web interface.

## Driver controls

Both gamepads use the same robot-centric driving map:

| Input | Action |
| --- | --- |
| Left stick Y | Drive forward/backward |
| Left stick X | Strafe left/right |
| Left trigger | Rotate left |
| Right trigger | Rotate right |
| Right stick | Unassigned |
| A | Toggle intake motor on/off |
| Y | Toggle intake direction forward/reverse |
| X | Run both shooter motors at their configured target RPM while held |
| B | Toggle continuous-rotation servo on/off |

Gamepad 1 has priority whenever it has active drive input. When gamepad 1 is idle, gamepad 2 can
drive with the same controls. Inputs from the two controllers are never added together, and a 0.05
deadband removes small joystick drift. Telemetry identifies the controller currently in command.

In `Limelight TeleOp`, `TeleOpControlThreads` runs two independent 10 ms control loops. The drive
thread only reads movement inputs and writes drivetrain power. The mechanism thread separately
handles the intake, shooter, and servo buttons. A wait or timed sequence added to the mechanism
thread therefore does not pause drivetrain updates. The main OpMode thread continues localization
and telemetry. Stopping the OpMode—or an exception in either worker—signals both workers to stop
and sets all controlled motors and the servo to zero.

The mechanism buttons above apply to `Limelight TeleOp` on either gamepad. In `TeleTourney`, A
re-anchors the Pedro/Limelight consistency check and B clears it; no mechanisms operate there. The
`Test: Odo vs AprilTag` diagnostic does not operate the mechanisms; in that OpMode, A captures a
baseline and B clears it. Pedro Pathing's generated `Tuning` OpMode retains its own controls and
does not use this shared mapping.

`TeleTourney` is the movement, odometry, and AprilTag-tracking competition OpMode. It uses the same
dual-gamepad drive mapping at full power and reports field X/Y, Pinpoint heading, active controller,
drive commands, and both odometry encoder counts on Driver Station. It also starts the Limelight
AprilTag pipeline and reports connection status, visible-tag count, and the nearest tag's ID,
robot-relative forward/left offset, horizontal/3D distance, and height. Its drive commands and pose
now come from Pedro's follower. Odometry and the IMU remain
the only source of the robot pose; Limelight measurements do not overwrite field X/Y or heading.

Mechanism power values are configured in `RobotConfig.Mechanisms`. The `servoMotor` mapping assumes
a continuous-rotation servo (`CRServo`); a positional servo would require on/off positions instead
of power values.

## Localization validation

Both `TeleTourney` and `Test: Odo vs AprilTag` compare **Pedro's Pinpoint** pose with
Limelight using `PedroAprilTagValidation`. `TeleTourney` automatically anchors the check to the
first connected, valid, non-stale visible tag. On either gamepad, A re-anchors to the nearest
visible tag and B clears the baseline and disables automatic capture until A is pressed again.
The baseline is retained when its tag leaves view; a different tag does not replace it. Missing
or disconnected data reports NO DATA/DISCONNECTED, never PASS.

Run the Driver Station OpMode `Test: Odo vs AprilTag` with a tag in view. This diagnostic uses
Pedro drive at 35% maximum power and requires manual baseline capture. Press A to capture that
tag as a fixed reference, then drive and rotate slowly while keeping it visible. The test converts
the baseline Limelight observation into field coordinates, uses subsequent odometry poses to
predict where the tag should appear relative to the robot, and compares that prediction with the
new Limelight observation. Telemetry reports distance error, planar position error, height drift,
and a live PASS/FAIL result. It also shows LL-inferred X/Y computed from the fixed baseline tag
position and current odometry/IMU heading. Press B to clear the baseline.

This is a relative drift check. The tag's baseline field position is inferred from the initial
Pedro pose, so an initial pose error is shared by both estimates. LL-inferred X/Y also depends on
IMU heading and camera calibration. The check does not reset, fuse, or correct Pedro's pose.
Absolute field re-localization would require known field tag coordinates and a calibrated camera
transform. Delayed camera frames can cause temporary disagreement while driving quickly.

The default pass limits are 3 inches of planar position error and 2 inches of height drift. Adjust
the `VALIDATION_*_TOLERANCE_INCHES` constants only after measuring the physical system's normal
noise. This is a relative-motion consistency test; an accurate absolute field pose still depends on
proper odometry calibration and a correct Limelight camera transform.

Desktop geometry tests can be run with:

```shell
./gradlew :TeamCode:testDebugUnitTest
```

## Component diagnostics

The Driver Station's `Diagnostics` group contains isolated hardware tests. Each test initializes
only the listed hardware so another unconfigured mechanism does not prevent it from starting.

| OpMode | Hardware loaded | Controls |
| --- | --- | --- |
| `Test: Drive Motors` | Four drivetrain motors | D-pad selects one motor; left-stick Y runs it at up to 35% power |
| `Test: Odometry Wheels` | Pinpoint on I²C bus 1 and its two pods | Move robot by hand; A zeros displayed pose/counts; telemetry shows status |
| `Test: Odo 12-inch Movement` | Four drive motors and Pinpoint | Each fresh Y press: forward, left, backward, right 12 in; B aborts |
| `Test: IMU` | Hub IMU | Rotate the robot by hand; A resets yaw |
| `Test: Limelight` | Limelight only | View live tag measurements; A freezes/unfreezes the display |
| `Test: Intake` | Intake motor only | A toggles power; Y reverses direction |
| `Test: Shooter` | Both shooter motors | Hold X to run both at 6000 RPM |
| `Test: Dual Shooter RPM` | `Shoot1` and `Shoot2` only | X/A raise/lower Shoot1; Y/B raise/lower Shoot2 by 100 RPM |
| `Test: Servo Motor` | Continuous-rotation servo only | B toggles power |
| `Test: Odo vs AprilTag` | Pedro drive, Pinpoint, Limelight | A captures a reference; B clears it |

Both gamepads work in every diagnostic. Lift the drive wheels off the floor before using the motor
test, verify the selected motor name in telemetry, and keep clear of all mechanisms. Every powered
diagnostic stops its output in a `finally` block when the OpMode ends.

The dual-shooter RPM test begins at a 6000 RPM target for each motor after Play and shows both target
and encoder-measured RPM. Button changes are edge-triggered, so holding a button changes its target
only once; release and press again for another 100 RPM step. Targets cannot be reduced below zero.
The motor controller must have the correct encoder-equipped motor type selected in the Robot
Controller configuration for velocity measurements and control to be accurate.

## Dependency direction

OpModes compose subsystems and services. Hardware-independent math does not import OpModes, and
subsystems do not read gamepads or write telemetry. Keep new mechanisms behind a subsystem class,
put reusable control math in `control` or `navigation`, and expose hardware names/tuning through
`RobotConfig`.

## Pedro Pathing setup and tuning

`pedroPathing.Constants.createFollower()` supplies a drivetrain and `PinpointLocalizer` to
Pedro 2.1.2. The device name comes from `RobotConfig.Hardware.PINPOINT`; encoder directions,
resolution (ticks per inch), and pod offsets come from `RobotConfig.Pinpoint`. Heading comes from
the Pinpoint's onboard gyro, not the hub IMU. Keep the robot stationary during initialization:
Pedro resets the Pinpoint and calibrates its gyro, taking approximately 300 ms. Do not
initialize a second localizer alongside the follower in the same OpMode.

`RobotMecanum` uses Pedro's vector allocation, motor caching, and configured motor directions.
It refreshes the wheel vectors when the measured forward/lateral velocities change, including
runtime tuning edits.
`PedroDriveLayout` converts its LF/LR/RF/RR outputs through the same axis/rotation corrections used
by `MecanumDrive`. Pedro commands and poses use inches, radians, positive Y left, and positive
heading counterclockwise. `TeleTourney` converts the shared controller's right-strafe/clockwise
commands to those conventions.

The follower PID values and drivetrain velocities in `pedroPathing.Constants` are library defaults,
not calibrated robot values. The pod offsets in `RobotConfig.Pinpoint` are still zero and
must be measured. Before using autonomous:

1. Use `Tuning` → Localization → Localization Test to verify forward increases X, left increases
   Y, and counterclockwise rotation increases heading. Translation should stay nearly constant
   while rotating in place once pod offsets are correct.
2. Measure offsets and confirm the pod specification. Enter them in `RobotConfig.Pinpoint`.
   The Pinpoint receives **ticks per inch**, not inches per tick. Its two pods must use the same
   resolution. The generic Forward/Lateral/Turn multiplier tuners are hidden from the menu because
   Pedro's Pinpoint localizer exposes raw counts rather than scale factors. The Offsets tuner may
   be used with both configured offsets initially zero; enter the resulting measurements in source.
3. Run forward/lateral velocity and zero-power acceleration tuning. Persist velocity values in
   `Constants.mecanumConstants` and follower acceleration/PID values in `Constants.followerConstants`.
   Panels edits are runtime settings; save the measured values in source before rebuilding.
4. Tune translational, heading, and drive controllers, then run the Line/Triangle/Circle tests.
5. Select `Pedro Auto Nav` to run the 24-inch square at 35% maximum power. Each segment times out
   after 10 seconds and aborts the remaining route. This is a demonstration route, not a match route.

Limelight continues providing independent tag measurements in `TeleTourney`; it never changes the
Pedro pose. `Pedro Auto Nav` loads only the four drive motors and Pinpoint.
