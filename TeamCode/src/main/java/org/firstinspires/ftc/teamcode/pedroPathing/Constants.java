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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(17)
            .forwardZeroPowerAcceleration(-30.2711203755)
            .lateralZeroPowerAcceleration(-64.5169691986)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0,0,0.03,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1.0,0.0,0.001,0.6,0.25))
            .centripetalScaling(0.0005);
            //.useSecondaryTranslationalPIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryDrivePIDF(true)


    
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.9)

            .xVelocity(51.015311501)
            .yVelocity(38.870022270)
            .rightFrontMotorName("MotorFrontRight")
            .rightRearMotorName("MotorBackRight")
            .leftRearMotorName("MotorBackLeft")
            .leftFrontMotorName("MotorFrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.614)
            .strafePodX(2.913)
            .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
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