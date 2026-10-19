package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

  public static PinpointConstants pinpointConstants() {
    return new PinpointConstants()
        .hardwareMapName(RobotConfig.Hardware.PINPOINT)
        .distanceUnit(DistanceUnit.INCH)
        .customEncoderResolution(RobotConfig.Pinpoint.TICKS_PER_INCH)
        .forwardEncoderDirection(RobotConfig.Pinpoint.FORWARD_DIRECTION)
        .strafeEncoderDirection(RobotConfig.Pinpoint.STRAFE_DIRECTION)
        .forwardPodY(RobotConfig.Pinpoint.FORWARD_POD_Y_INCHES)
        .strafePodX(RobotConfig.Pinpoint.STRAFE_POD_X_INCHES);
  }

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pinpointLocalizer(pinpointConstants())
        .setDrivetrain(new RobotMecanum(hardwareMap, mecanumConstants))
        .pathConstraints(pathConstraints)
        .build();
  }
}
