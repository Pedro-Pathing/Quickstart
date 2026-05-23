package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
//            .useSecondaryDrivePIDF(true)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryTranslationalPIDF(true)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25,0,0.02,0.015))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.11))
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.1,0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007,0,0.00001,.6,0.03))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.007,0,0.000005,0.6,0.005))
//            .forwardZeroPowerAcceleration(-62.8563)
//            .lateralZeroPowerAcceleration(-95.8876)
            .mass(10.3);
            // .centripetalScaling(0.005);

    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
            .xVelocity(74.4975)
            .yVelocity(55.9221);
//            .rightFrontMotorName("fR")
//            .rightRearMotorName("bR")
//            .leftRearMotorName("bL")
//            .leftFrontMotorName("fL")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants();
//            .forwardPodY(4.37)
//            .strafePodX(1.71)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 75, 0.5,1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
