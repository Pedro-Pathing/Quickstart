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
    // Настройки чувствительности для yamato 2 - улучшенный контроль
    public static FollowerConstants followerConstants = new FollowerConstants()
             .mass(9)
            .forwardZeroPowerAcceleration(-35.54014367765671)
            .lateralZeroPowerAcceleration(-58.69753099202689)
            // Улучшенные PID коэффициенты для более точного контроля
            .translationalPIDFCoefficients(new PIDFCoefficients(0.085, 0.001, 0.004, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.1, 0.25, 0))
            // Улучшенные коэффициенты для привода с фильтрацией
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.055, 0.001, 0.002, 0.12, 0.015))
            ;


    // Настройки привода с улучшенной чувствительностью для yamato 2
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)  // Максимальная мощность
            .rightFrontMotorName("right1")
            .rightRearMotorName("right2")
            .leftRearMotorName("left2")
            .leftFrontMotorName("left1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)//right
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)//right
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)//right
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)//right
            // Максимальные скорости для быстрого движения
            .xVelocity(75.55703182671013)
            .yVelocity(55.64442600039986);

          public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7)
            .strafePodX(7)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    // Улучшенные ограничения пути для более плавного и точного движения yamato 2
    public static PathConstraints pathConstraints = new PathConstraints(0.95, 90, 0.9, 0.9);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
