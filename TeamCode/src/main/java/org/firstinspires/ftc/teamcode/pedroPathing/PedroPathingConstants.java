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
import org.firstinspires.ftc.teamcode.util.Configuration;

public class PedroPathingConstants {

    private static Configuration config = new Configuration();

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.2)
            .centripetalScaling(1)
            .forwardZeroPowerAcceleration(-25.9346931313679598)
            .lateralZeroPowerAcceleration(-67.342491844080064)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.16,
                    0,
                    0.02,
                    0
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.16,
                    0,
                    0.02,
                    0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.06    ,
                    0.02
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.06,
                    0.02
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.017,
                    0,
                    0.000001,
                    0.6,
                    0.002
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.017,
                    0,
                    0.000001,
                    0.6,
                    0.002
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(config.leftFront)
            .leftRearMotorName(config.leftRear)
            .rightFrontMotorName(config.rightFront)
            .rightRearMotorName(config.rightRear)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.261926752421046666666666666667)
            .yVelocity(61.494551922189565)
            .useBrakeModeInTeleOp(true);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-36.65/25.4)
            .strafePodX(-30.292/25.4)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            250,
            1.25,
            10,
            1
    );
    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();

    }

}