package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
             .mass(10.8)
            .forwardZeroPowerAcceleration(-35.54014367765671)
            .lateralZeroPowerAcceleration(-58.69753099202689)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.003, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6,0.09,0.2,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.001, 0.01, 0.01))
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right1")
            .rightRearMotorName("right2")
            .leftRearMotorName("left2")
            .leftFrontMotorName("left1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//right
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)//right
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//right
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)//right
            .xVelocity(75.55703182671013)
            .yVelocity(55.64442600039986);

          public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(8)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
