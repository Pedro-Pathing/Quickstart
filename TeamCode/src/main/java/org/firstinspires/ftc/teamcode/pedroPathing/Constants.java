package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(17.95)   // mass in kg
           // .forwardZeroPowerAcceleration(-42.944916498283)   // have to be negative
            //.lateralZeroPowerAcceleration(-53.72242120209232)
           // .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.015, 0.04))    // these are the PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.1, 0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.14, 0.1432543903599445, 0.0013771510969864621))
          //  .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.00001,0.6,0.05))
            .centripetalScaling(0.0);
    ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)   // max motor power
            .rightFrontMotorName("rf")   // Motor hardware maps
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)  // Motor directions
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.64853169598919)
            .yVelocity(46.88268946850394)
            .useBrakeModeInTeleOp(true);
;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3)   // Y offset
            .strafePodX(-7)  // X offset
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")  // hardware map for pinpoint computer
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)   // Encoder direction
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 0.3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
