package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double CPR312 = ((((1+(46D/17))) * (1+(46D/11))) * 28);
    public static double CPR1150 = ((1+(46D/11)) * 28);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.7)
            .forwardZeroPowerAcceleration(-59.57090583619754)
            .lateralZeroPowerAcceleration(-84.38145712859071)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,.07,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1.1, 0, .07, .6,.01))
            .centripetalScaling(0.00025);

    public static PathConstraints pathConstraints = new PathConstraints(.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("front_left_motor")
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftRearMotorName("back_left_motor")
            .maxPower(.8)
            .xVelocity(73.49579456588785)
            .yVelocity(50.264192571955085)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static TwoWheelConstants localizerConst = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("back_left_motor")
            .strafeEncoder_HardwareMapName("front_left_motor")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)
            )
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(.002968434)
            .strafeTicksToInches(.00197895601)
            .forwardPodY(6+7.0/16)
            .strafePodX(3.2912680654);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConst)
                .build();
    }
}
