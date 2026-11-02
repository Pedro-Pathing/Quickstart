package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {





    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    //localizerConfig(threeWheelOdometry) goes here
    public static ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {
        c.leftEncoderName.set("lf");
        c.rightEncoderName.set("rr");
        c.strafeEncoderName.set("lr");
        c.imuName.set("imu");
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        c.leftPodY.set(3.8827259303486525);
        c.rightPodY.set(-6.900712145042778);
        c.strafePodX.set(-7.474621735789836);
        c.forwardTicksToInches.set(0.002001960160624636);
        c.strafeTicksToInches.set(0.002019889520748372);
        c.turnTicksToRadians.set(0.0032127810553454566);
        c.leftEncoderDirection.set(Encoder.REVERSE);
        c.rightEncoderDirection.set(Encoder.REVERSE);
        c.strafeEncoderDirection.set(Encoder.FORWARD);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21833312352793863);
                Controller secondaryTranslationalForward = Controller.proportional(0.08066825517506691);
                Controller primaryTranslationalLateral = Controller.proportional(0.38617018970518624);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14267956643855592);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0175460423692779));
                c.brake.set(Controller.proportionalFeedforward(0.014914136013886215));

                c.headingFeedback.set(Controller.proportional(3.076233083397824));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05870653841528911, -9.595017625251563E-4));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05784759147666409, 0.05550050554089865));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014602095910336817, 0.0015111919841190734));

                c.maxAchievableForwardVelocity.set(58.78585756410744);
                c.maxAchievableStrafeVelocity.set(51.03746367790937);
                c.naturalForwardDeceleration.set(37.431026613485905);
                c.naturalStrafeDeceleration.set(58.94394153193374);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new ThreeWheelIMULocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}