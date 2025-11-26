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
            .mass(5) // TODO: Change this to the actual weight of the robot
            .forwardZeroPowerAcceleration(-45.7735)
            .lateralZeroPowerAcceleration(-53.7000)
            // Set following parameters to true to enable dual PID
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1, 0, 0.01, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0.3, 0.2, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3, 0.1, 0.01, 0.6, 0.6));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.3402)
            .yVelocity(46.4157);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2)
            .strafePodX(4.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
