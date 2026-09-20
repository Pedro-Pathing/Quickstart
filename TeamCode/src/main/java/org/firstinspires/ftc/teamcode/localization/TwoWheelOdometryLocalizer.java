package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Field localization from one forward pod, one strafe pod, and the hub IMU heading. */
public final class TwoWheelOdometryLocalizer {
  private final DcMotor forwardEncoder;
  private final DcMotor strafeEncoder;
  private final IMU imu;

  private double previousForwardTicks;
  private double previousStrafeTicks;
  private double headingOffsetRadians;
  private double previousHeadingRadians;
  private double x;
  private double y;
  private double headingRadians;

  public TwoWheelOdometryLocalizer(HardwareMap hardwareMap) {
    forwardEncoder =
        hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_FORWARD);
    strafeEncoder = hardwareMap.get(DcMotor.class, RobotConfig.Hardware.ODOMETRY_STRAFE);
    imu = hardwareMap.get(IMU.class, RobotConfig.Hardware.IMU);
    imu.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    resetPose(0, 0, 0);
  }

  /** Resets the field pose without resetting encoder hardware or the IMU. */
  public void resetPose(double x, double y, double headingDegrees) {
    this.x = x;
    this.y = y;
    headingRadians = Math.toRadians(headingDegrees);
    headingOffsetRadians = normalizeRadians(headingRadians - getRawImuHeadingRadians());
    previousHeadingRadians = headingRadians;
    captureEncoderPositions();
  }

  public void update() {
    double forwardTicks = getForwardTicks();
    double strafeTicks = getStrafeTicks();
    double currentHeading =
        normalizeRadians(getRawImuHeadingRadians() + headingOffsetRadians);

    double measuredForward =
        (forwardTicks - previousForwardTicks)
            / RobotConfig.Localization.ODOMETRY_FORWARD_TICKS_PER_INCH;
    double measuredStrafe =
        (strafeTicks - previousStrafeTicks)
            / RobotConfig.Localization.ODOMETRY_STRAFE_TICKS_PER_INCH;
    double deltaHeading = normalizeRadians(currentHeading - previousHeadingRadians);

    previousForwardTicks = forwardTicks;
    previousStrafeTicks = strafeTicks;
    previousHeadingRadians = currentHeading;

    // Remove encoder motion caused only by each pod rotating around the robot center.
    double deltaForward =
        measuredForward
            + RobotConfig.Localization.ODOMETRY_FORWARD_LATERAL_OFFSET_INCHES * deltaHeading;
    double deltaStrafe =
        measuredStrafe
            - RobotConfig.Localization.ODOMETRY_STRAFE_FORWARD_OFFSET_INCHES * deltaHeading;
    double midpointHeading = headingRadians + deltaHeading / 2.0;

    x += deltaForward * Math.cos(midpointHeading) - deltaStrafe * Math.sin(midpointHeading);
    y += deltaForward * Math.sin(midpointHeading) + deltaStrafe * Math.cos(midpointHeading);
    headingRadians = currentHeading;
  }

  public PoseEstimate getPoseEstimate() {
    return new PoseEstimate(x, y, Math.toDegrees(headingRadians));
  }

  public double getForwardEncoderTicks() {
    return getForwardTicks();
  }

  public double getStrafeEncoderTicks() {
    return getStrafeTicks();
  }

  private void captureEncoderPositions() {
    previousForwardTicks = getForwardTicks();
    previousStrafeTicks = getStrafeTicks();
  }

  private double getForwardTicks() {
    return forwardEncoder.getCurrentPosition()
        * RobotConfig.Localization.ODOMETRY_FORWARD_MULTIPLIER;
  }

  private double getStrafeTicks() {
    return strafeEncoder.getCurrentPosition()
        * RobotConfig.Localization.ODOMETRY_STRAFE_MULTIPLIER;
  }

  private double getRawImuHeadingRadians() {
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
  }

  private static double normalizeRadians(double radians) {
    while (radians > Math.PI) {
      radians -= 2.0 * Math.PI;
    }
    while (radians <= -Math.PI) {
      radians += 2.0 * Math.PI;
    }
    return radians;
  }
}
