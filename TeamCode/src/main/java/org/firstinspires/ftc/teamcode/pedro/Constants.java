package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("f-l-drive");
        c.frontRightName.set("f-r-drive");
        c.backLeftName.set("b-l-drive");
        c.backRightName.set("b-r-drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.346042542945682);
        c.yPodOffset.set(0.056735183310321);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2653130511639212);
                Controller secondaryTranslationalForward = Controller.proportional(0.09802608310977637);
                Controller primaryTranslationalLateral = Controller.proportional(0.4947208786430955);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18278614547328523);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013420240062536976));
                c.brake.set(Controller.proportionalFeedforward(0.01140720405315643));

                c.headingFeedback.set(Controller.proportional(3.9507191601503298));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.044714858106918866, 0.006810337659967676));

                c.linearBrakeCoefficients.set(Matrix.diag(0.14168703892179885, 0.06348163155358467));
                c.quadraticBrakeCoefficients.set(Matrix.diag(6.473742817585828E-4, 0.0017593481650221808));

                c.maxAchievableForwardVelocity.set(77.96611517785864);
                c.maxAchievableStrafeVelocity.set(59.998585236678196);
                c.naturalForwardDeceleration.set(30.043054590300503);
                c.naturalStrafeDeceleration.set(63.305998669443504);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
