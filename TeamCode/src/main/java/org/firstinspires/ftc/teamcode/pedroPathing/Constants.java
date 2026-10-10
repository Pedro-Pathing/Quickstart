package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Pedro configuration. Velocity/PID defaults are starting values requiring on-robot tuning. */
@Configurable
public final class Constants {
  public static FollowerConstants followerConstants = new FollowerConstants();
  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
  public static MecanumConstants mecanumConstants = new MecanumConstants()
      .leftFrontMotorName(RobotConfig.Hardware.FRONT_LEFT)
      .leftRearMotorName(RobotConfig.Hardware.BACK_LEFT)
      .rightFrontMotorName(RobotConfig.Hardware.FRONT_RIGHT)
      .rightRearMotorName(RobotConfig.Hardware.BACK_RIGHT)
      .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .useBrakeModeInTeleOp(true);

  private Constants() {}

  public static TwoWheelConstants twoWheelConstants() {
    return new TwoWheelConstants()
        .forwardEncoder_HardwareMapName(RobotConfig.Hardware.ODOMETRY_FORWARD)
        .strafeEncoder_HardwareMapName(RobotConfig.Hardware.ODOMETRY_STRAFE)
        .IMU_HardwareMapName(RobotConfig.Hardware.IMU)
        .IMU_Orientation(RobotConfig.Localization.imuOrientation())
        .forwardTicksToInches(1.0 / RobotConfig.Localization.ODOMETRY_FORWARD_TICKS_PER_INCH)
        .strafeTicksToInches(1.0 / RobotConfig.Localization.ODOMETRY_STRAFE_TICKS_PER_INCH)
        .forwardEncoderDirection(RobotConfig.Localization.ODOMETRY_FORWARD_MULTIPLIER)
        .strafeEncoderDirection(RobotConfig.Localization.ODOMETRY_STRAFE_MULTIPLIER)
        .forwardPodY(RobotConfig.Localization.ODOMETRY_FORWARD_LATERAL_OFFSET_INCHES)
        .strafePodX(RobotConfig.Localization.ODOMETRY_STRAFE_FORWARD_OFFSET_INCHES);
  }

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .twoWheelLocalizer(twoWheelConstants())
        .setDrivetrain(new RobotMecanum(hardwareMap, mecanumConstants))
        .pathConstraints(pathConstraints)
        .build();
  }
}
