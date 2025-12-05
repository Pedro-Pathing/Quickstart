package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(25)
            .forwardZeroPowerAcceleration(-33.505427566651264)
            .lateralZeroPowerAcceleration(-68.20080269224975)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0.001, 0.6, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0025, 0, 0, 0.6, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.005, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.01, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0,0,0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.08, 0.01))
            .centripetalScaling(0.0004);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(76.27145025298351)
            .yVelocity(57.29479740172859);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 0.6, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-55)
            .strafePodX(-12.5)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(34.311)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
