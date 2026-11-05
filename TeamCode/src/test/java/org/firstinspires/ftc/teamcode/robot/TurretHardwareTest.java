package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

/**
 * The turret's hardware-facing edges: servo direction, the disabled turret, a disabled encoder,
 * re-zeroing and live config reloads. Config changes here are undone by FreshConfigExtension.
 */
public class TurretHardwareTest {
  private final AtomicReference<Double> power = new AtomicReference<>(0.0);
  private final AtomicReference<CRServo.Direction> direction =
      new AtomicReference<>(CRServo.Direction.FORWARD);
  private final AtomicReference<Double> volts = new AtomicReference<>(0.0);
  private long nanos = 1_000_000_000L;

  private Turret build() {
    HardwareMap hw = Mockito.mock(HardwareMap.class);
    CRServo servo = Mockito.mock(CRServo.class);
    AnalogInput analog = Mockito.mock(AnalogInput.class);
    Mockito.when(hw.get(CRServo.class, "turn")).thenReturn(servo);
    Mockito.when(hw.get(AnalogInput.class, "turnanalog")).thenReturn(analog);
    Mockito.doAnswer(inv -> power.getAndSet(inv.getArgument(0)))
        .when(servo)
        .setPower(Mockito.anyDouble());
    Mockito.doAnswer(inv -> direction.getAndSet(inv.getArgument(0)))
        .when(servo)
        .setDirection(Mockito.any());
    Mockito.when(servo.getDirection()).thenAnswer(inv -> direction.get());
    Mockito.when(analog.getVoltage()).thenAnswer(inv -> volts.get());
    volts.set(config.turret.analog_encoder.zero_voltage);
    return new Turret(hw, null, () -> new Pose(72, 72, 0), Velocity::zero, () -> nanos);
  }

  @Test
  public void servoDirectionFollowsTheConfigAndIsReassertedOnEveryWrite() {
    config.turret.servo_direction_inverted = true;
    Turret turret = build();
    assertEquals(CRServo.Direction.REVERSE, direction.get());

    direction.set(CRServo.Direction.FORWARD); // e.g. another OpMode changed it
    turret.setTurretPowerRaw(0.0);
    assertEquals(CRServo.Direction.REVERSE, direction.get());

    config.turret.servo_direction_inverted = false;
    turret.setTurretPowerRaw(0.0);
    assertEquals(CRServo.Direction.FORWARD, direction.get());
  }

  @Test
  public void aDisabledTurretNeverDrivesTheServo() {
    Turret turret = build();
    turret.setAimMode(Turret.AimMode.MANUAL);
    turret.setManualPower(0.4);
    turret.periodic();
    assertEquals(0.4, power.get(), 1e-12);

    turret.setEnabled(false);
    turret.periodic();
    assertEquals(0.0, power.get(), 0.0);
    assertEquals("TURRET DISABLED", turret.getPowerBlockedReason());
  }

  @Test
  public void withTheEncoderDisabledNothingTrustsAReading() {
    config.turret.analog_encoder.enabled = false;
    Turret turret = build();
    volts.set(3.0);
    assertTrue(Double.isNaN(turret.getRawVoltage()));
    assertEquals(0.0, turret.getCurrentTurnAngle(), 0.0);
    assertTrue(turret.isReadingWithinTravel());

    // No reading means no motion to judge: the watchdogs must not trip on a turret that does not
    // appear to move.
    turret.setAimMode(Turret.AimMode.MANUAL);
    turret.setManualPower(0.4);
    for (int i = 0; i < 100; i++) {
      nanos += 20_000_000L;
      turret.periodic();
    }
    assertFalse(turret.isFaulted(), turret.getFaultReason());
    assertEquals(0.4, power.get(), 1e-12);
  }

  @Test
  public void zeroingAtTheCurrentPositionMakesItReadZero() {
    Turret turret = build();
    volts.set(config.turret.analog_encoder.zero_voltage + 0.2);
    assertTrue(Math.abs(turret.getCurrentTurnAngle()) > 5);
    turret.zeroAtCurrentPosition();
    assertEquals(0.0, turret.getCurrentTurnAngle(), 1e-9);
    assertEquals(config.turret.analog_encoder.zero_voltage + 0.2, turret.getZeroVoltage(), 1e-12);
  }

  @Test
  public void reloadingConfigUpdatesTheGains() {
    Turret turret = build();
    config.turret.pidf.p = 0.123;
    config.turret.max_power_output = 0.33;
    turret.reloadFromConfig();
    assertEquals(0.123, turret.getP(), 0.0);
    assertEquals(0.33, turret.getMaxPower(), 0.0);

    // ...and the controller actually uses them: a 5 deg error commands about p * 5 more than kS.
    config.turret.ks_positive = 0.0;
    config.turret.ks_negative = 0.0;
    config.turret.pidf.i = 0.0;
    config.turret.max_power_output = 0.9;
    turret.reloadFromConfig();
    turret.setHoldAngle(5.0);
    turret.setAimMode(Turret.AimMode.HOLD);
    nanos += 20_000_000L;
    turret.periodic();
    assertEquals(0.123 * 5.0, power.get(), 1e-3);
  }

  /** TeleOp's chassis align: the heading from where the robot is, not from the field origin. */
  @Test
  public void alignPoseFacesTheTargetFromAnywhere() {
    assertEquals(Math.PI / 2, Turret.alignPose(10, 20, 10, 30).heading(), 1e-9);
    Pose p = Turret.alignPose(50, 50, 40, 40);
    assertEquals(50, p.x(), 0.0);
    assertEquals(50, p.y(), 0.0);
    assertEquals(
        0.0,
        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians(
            p.heading() + 3 * Math.PI / 4),
        1e-9);
  }
}
