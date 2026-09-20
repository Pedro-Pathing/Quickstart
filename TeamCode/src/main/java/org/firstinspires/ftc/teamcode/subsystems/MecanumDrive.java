package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.DrivePower;
import org.firstinspires.ftc.teamcode.drive.MecanumKinematics;

/** Owns the drivetrain hardware and exposes robot-centric mecanum commands. */
public final class MecanumDrive {
  public enum MotorPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  private final DcMotor frontLeft;
  private final DcMotor frontRight;
  private final DcMotor backLeft;
  private final DcMotor backRight;

  public MecanumDrive(HardwareMap hardwareMap) {
    this(
        hardwareMap,
        RobotConfig.Hardware.FRONT_LEFT,
        RobotConfig.Hardware.FRONT_RIGHT,
        RobotConfig.Hardware.BACK_LEFT,
        RobotConfig.Hardware.BACK_RIGHT);
  }

  public MecanumDrive(
      HardwareMap hardwareMap,
      String frontLeftName,
      String frontRightName,
      String backLeftName,
      String backRightName) {
    frontLeft = hardwareMap.get(DcMotor.class, frontLeftName);
    frontRight = hardwareMap.get(DcMotor.class, frontRightName);
    backLeft = hardwareMap.get(DcMotor.class, backLeftName);
    backRight = hardwareMap.get(DcMotor.class, backRightName);

    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void driveRobotCentric(double drive, double strafe, double rotate) {
    setPower(MecanumKinematics.robotCentric(drive, strafe, rotate));
  }

  public void setPower(DrivePower power) {
    frontLeft.setPower(power.getFrontLeft());
    frontRight.setPower(power.getFrontRight());
    backLeft.setPower(power.getBackLeft());
    backRight.setPower(power.getBackRight());
  }

  /** Stops the drivetrain, then powers only the requested motor for hardware diagnostics. */
  public void runSingleMotor(MotorPosition position, double power) {
    stop();
    double clippedPower = Math.max(-1.0, Math.min(1.0, power));
    switch (position) {
      case FRONT_LEFT:
        frontLeft.setPower(clippedPower);
        break;
      case FRONT_RIGHT:
        frontRight.setPower(clippedPower);
        break;
      case BACK_LEFT:
        backLeft.setPower(clippedPower);
        break;
      case BACK_RIGHT:
        backRight.setPower(clippedPower);
        break;
    }
  }

  public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
    frontLeft.setZeroPowerBehavior(behavior);
    frontRight.setZeroPowerBehavior(behavior);
    backLeft.setZeroPowerBehavior(behavior);
    backRight.setZeroPowerBehavior(behavior);
  }

  public void stop() {
    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
  }
}
