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
            .mass(15)
            .forwardZeroPowerAcceleration(-30)
            .lateralZeroPowerAcceleration(-56)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.05))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
            .useSecondaryTranslationalPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(4, 0, 0.1, 0))
            .useSecondaryHeadingPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0.01, 0.000001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0, 0.000005, 0.6, 0.01))
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.00015)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor2")
            .rightRearMotorName("motor3")
            .leftFrontMotorName("motor0")
            .leftRearMotorName("motor1")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67)
            .yVelocity(56.27);


    public static PinpointConstants localizerConstants = new PinpointConstants() // TODO: tune pinpoint localizers
            .forwardPodY(3.75)
            .strafePodX(-5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
