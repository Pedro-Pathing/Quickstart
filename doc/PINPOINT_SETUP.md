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
