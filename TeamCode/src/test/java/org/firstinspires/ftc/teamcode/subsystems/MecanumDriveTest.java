package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.drive.DrivePower;
import org.junit.Test;

public final class MecanumDriveTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void forwardCommandUsesInstalledChassisForwardPattern() {
    DrivePower power = MecanumDrive.configuredRobotCentric(1.0, 0.0, 0.0);

    assertPower(power, -1.0, 1.0, 1.0, -1.0);
  }

  @Test
  public void strafeCommandUsesInstalledChassisStrafePattern() {
    DrivePower power = MecanumDrive.configuredRobotCentric(0.0, 1.0, 0.0);

    assertPower(power, -1.0, -1.0, -1.0, -1.0);
  }

  @Test
  public void installedLayoutReversesRotationPattern() {
    DrivePower power = MecanumDrive.configuredRobotCentric(0.0, 0.0, 1.0);

    assertPower(power, -1.0, 1.0, -1.0, 1.0);
  }

  private static void assertPower(
      DrivePower power,
      double frontLeft,
      double frontRight,
      double backLeft,
      double backRight) {
    assertEquals(frontLeft, power.getFrontLeft(), EPSILON);
    assertEquals(frontRight, power.getFrontRight(), EPSILON);
    assertEquals(backLeft, power.getBackLeft(), EPSILON);
    assertEquals(backRight, power.getBackRight(), EPSILON);
  }
}
