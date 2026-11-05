package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

/**
 * Turret used to read an IMU that gave an absolute (world-frame) heading, so the code had to
 * subtract the robot's current heading to get the turret's angle relative to the chassis. The team
 * has since switched to an analog voltage reading taken directly off the turret servo, which
 * already reports the angle relative to the chassis -- no robot heading involved at all.
 *
 * <p>These tests drive the real {@link Turret} class with a mocked, controllable analog voltage and
 * pose supplier to verify physically-observable behavior: rotating the whole robot chassis in place
 * must not change what the turret safety logic thinks the turret is doing, since the servo/encoder
 * never moved.
 */
public class TurretMigrationDiagnosticTest {

  private Turret turret;
  private CRServo mockServo;
  private double[] voltageBox; // mutable box so the Mockito stub can see later updates
  private Pose[] poseBox;

  @BeforeEach
  public void setUp() {
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    mockServo = Mockito.mock(CRServo.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);
    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(mockServo);

    AnalogInput mockAnalog = Mockito.mock(AnalogInput.class);
    voltageBox = new double[] {config.turret.analog_encoder.zero_voltage};
    Mockito.when(hardwareMap.get(AnalogInput.class, "turnanalog")).thenReturn(mockAnalog);
    Mockito.when(mockAnalog.getVoltage()).thenAnswer(inv -> voltageBox[0]);

    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    // config.turret.enabled defaults to false on a freshly-built robot; this suite exercises the
    // aiming/safety math, not the enabled flag, so force it on regardless of test order.
    config.turret.enabled = true;

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    poseBox = new Pose[] {new Pose(72, 72, 0)};
    turret = new Turret(hardwareMap, mockTelemetry, () -> poseBox[0]);
  }

  /**
   * Sets the mocked analog voltage so getCurrentTurnAngle() reports the given chassis-relative
   * degrees.
   */
  private void setPhysicalTurretAngle(double chassisRelativeDegrees) {
    var enc = config.turret.analog_encoder;
    double delta =
        enc.inverted
            ? -chassisRelativeDegrees / enc.degrees_per_volt
            : chassisRelativeDegrees / enc.degrees_per_volt;
    voltageBox[0] = enc.zero_voltage + delta;
  }

  @Test
  public void encoderReadingMatchesPhysicalAngleFromVoltageAlone() {
    // getCurrentTurnAngle() takes no pose/heading input at all -- it can only ever reflect the
    // analog voltage, which is exactly what we want from a chassis-relative sensor.
    setPhysicalTurretAngle(20.0);
    assertEquals(20.0, turret.getCurrentTurnAngle(), 1e-2);

    setPhysicalTurretAngle(-33.5);
    assertEquals(-33.5, turret.getCurrentTurnAngle(), 1e-2);
  }

  @Test
  public void boundarySafetyDoesNotFalsePositiveWhenChassisRotatesWithTurretCentered() {
    // Physically the turret is dead-center (0 deg relative to chassis) -- nowhere near either
    // mechanical limit.
    setPhysicalTurretAngle(0.0);

    // Driver spins the WHOLE ROBOT CHASSIS to a new field heading (50 deg) without the turret
    // servo moving at all -- the analog voltage is unchanged, so the turret is still physically
    // centered.
    Pose robotRotatedInPlace = new Pose(72, 72, Math.toRadians(50));

    // Ask the turret to hold a perfectly legal, safe target within mechanical travel.
    double safeTargetAngle = Math.max(-25.0, config.turret.travel.min_angle + 1.0);
    turret.setTargetTurnAngle(safeTargetAngle);

    double error = turret.getAimError(robotRotatedInPlace);
    assertEquals(safeTargetAngle, error, 1e-6);

    Mockito.reset(mockServo);
    turret.updateTurret(robotRotatedInPlace);

    // The turret physically has full safe travel available (it's centered). It
    // must not be refused power due to the robot chassis's field heading -- only the turret's
    // OWN angle relative to the chassis may trigger safety cutoff.
    Mockito.verify(mockServo, Mockito.never()).setPower(0.0);
  }

  @Test
  public void aimAtGoalConvertsWorldBearingToChassisRelativeAndClamps() {
    // Goal is due "north" (+Y) of the robot. Chassis facing "east" (heading 0) needs a 90 deg
    // relative turret angle to hit it -- beyond the mechanical limit, so it must clamp to
    // max_angle.
    turret.setTargetPoint(72.0, 200.0);
    setPhysicalTurretAngle(0.0);
    poseBox[0] = new Pose(72.0, 72.0, 0.0);

    turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);
    turret.periodic();
    assertEquals(config.turret.travel.max_angle, turret.getTargetTurnAngle(), 1e-6);

    // Rotate the chassis 90 deg to face "north" -- the same field-fixed goal now sits directly
    // ahead, so the required chassis-relative target collapses to 0.
    poseBox[0] = new Pose(72.0, 72.0, Math.PI / 2);
    turret.periodic();
    assertEquals(0.0, turret.getTargetTurnAngle(), 1e-6);
  }
}
