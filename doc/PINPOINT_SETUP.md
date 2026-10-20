# Pinpoint wiring and configuration

| Connection | Destination |
| --- | --- |
| Parallel / forward odometry pod | Pinpoint X encoder connector |
| Perpendicular / strafe odometry pod | Pinpoint Y encoder connector |
| Pinpoint I²C cable | Control Hub I²C port / bus **1** |

The pods no longer connect to hub motor encoder ports. The four drive motors keep their existing
connections. Follow the Pinpoint manufacturer's cable and mounting instructions; its onboard IMU
now supplies localization heading. The hub IMU remains available for `Test: IMU` only.

## Robot Controller configuration

Edit the active robot configuration, select **Control Hub → I²C Bus 1**, select device slot **0**,
choose **goBILDA Pinpoint Odometry Computer**, and name it exactly **pinpoint**. Save and activate
the configuration. The default I²C address is managed by the SDK driver; do not use bus number 1 as
an I²C address. Remove unused `odoForward`/`odoStrafe` entries if present.

The equivalent entry inside the existing Control Hub `LynxModule` is:

```xml
<goBILDAPinpoint name="pinpoint" port="0" bus="1" />
```

This is a configuration fragment, not a complete uploadable robot XML. Keep the module address,
hub serial number, motors, servos, and other devices from your actual robot configuration. Java
looks up the name `pinpoint`; the active XML/UI configuration assigns that name to I²C bus 1.

## Pod settings and verification

`RobotConfig.Pinpoint` currently preserves the previous pod specification: 1000 counts per wheel
revolution and 0.6889-inch radius, approximately 231 ticks per inch. Confirm this against your
actual pods. A Pinpoint uses one encoder resolution for both pods. Set `TICKS_PER_INCH` to the
correct common value if the pods change. Do not copy a ticks-per-mm value directly into this field.

All TeamCode OpModes (including standalone component tests), Pedro tuning telemetry, and FTC
Dashboard localization packets display `Ticks to Inches Coefficient (in/tick, configured)`.
This is `1 / TICKS_PER_INCH`, approximately **0.00432849 inches per tick** with the current settings.
Multiply an encoder count change by this coefficient to convert it to pod travel in inches.
The displayed value is configured, not a measured calibration or a hardware connection check;
standalone component tests do not initialize Pinpoint just to display it.

Set measured `FORWARD_POD_Y_INCHES` (positive left of center) and `STRAFE_POD_X_INCHES` (positive
ahead of center). Change each direction to `EncoderDirection.REVERSED` if needed. The values
currently default to zero offsets and FORWARD directions, pending physical verification.

Keep the robot stationary during INIT while the Pinpoint calibrates. Run `Test: Odometry Wheels`:

- Status should become READY; pod-detection faults identify an unplugged or miswired X/Y pod.
- Moving forward should increase pose X; moving left should increase pose Y.
- Counterclockwise rotation should increase heading; with correct offsets, rotating in place
  should produce little translation.
- A on either gamepad resets displayed pose and count baselines without recalibrating the gyro.

Then verify `Tuning → Localization Test`, retune the follower as needed, and run
`Test: Odo vs AprilTag`. Limelight validation continues using Pedro's pose, now provided by Pinpoint.

In `Tuning → Localization → Localization Test`, live field X/Y (starting at 72/72 inches),
heading in degrees, Pinpoint status, and raw X/Y pod counts appear on both Driver Station and
Panels before and after Play. Select the test with right bumper; the selection menu itself does
not read Pinpoint. Keep still during initial gyro calibration, then push the robot by hand during
INIT to verify that the pose and counts change without motor commands. After Play, the test also
enables its existing gamepad-1 stick driving controls; leave the sticks centered for hand testing.
The `Odometry Updates (software)` counter confirms software refreshes, not successful sensor
communication. If it increases but pose/counts remain frozen while moving, inspect Pinpoint status,
pod cables/contact, and the active hub configuration; the configured coefficient should stay fixed.

## Powered 12-inch movement test

Select `Diagnostics → Test: Odo 12-inch Movement`, keep still during calibration, then press Play.
Leave the robot on the floor with a clear test area and an operator ready to press B or Stop.
Each fresh Y press on either controller runs one leg: **forward 12 in → strafe left 12 in →
backward 12 in → strafe right 12 in**, then the sequence repeats. Each target is relative to the
robot's pose at that press, keeping that leg's starting heading. The robot stops between legs;
there is no automatic continuation or active position hold during the pause. Sticks do not drive.

Power is capped at 25%. Each leg must stay within 0.5 inches of its target and 3 degrees of its
starting heading for 0.2 seconds, with a 10-second timeout. B aborts immediately at the next control
loop. Aborts, timeouts, and Pinpoint faults do not advance the sequence: release Y, resolve the
problem, then press Y to retry the same direction. Busy or held-button presses are not queued.
Only READY status and a finite pose permit starting movement. These are diagnostic safeguards,
not collision detection; follower gains and pod offsets still require physical tuning.

Telemetry remains live in INIT, during movement, and while paused: Pinpoint pose/status, raw X/Y
counts, configured inches/tick, target/error, leg forward/left displacement, commanded-direction
distance, and per-leg tick changes. The pose origin is reset to zero at Play without recalibrating
the gyro. Results remain visible while paused so you can measure actual ground travel with a ruler.

The commanded 12 inches is **Pinpoint-estimated**, not guaranteed physical travel. This test cannot
independently calibrate its own distance source. For straight legs without rotation/slip, compute
`new inches/tick = old inches/tick × measured physical travel / reported travel`, then set the
reciprocal in `RobotConfig.Pinpoint.TICKS_PER_INCH` and rebuild. Repeat forward/backward and
left/right measurements; Pinpoint has one shared resolution for both pods. The previous
12-inch-actual versus 26-inch-reported result suggests approximately 500.56 ticks/inch, but this
test does not automatically apply that calibration.
