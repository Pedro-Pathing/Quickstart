package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lr");
                c.frontRightName.set("rf");
                c.backRightName.set("rr");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(1.8448);
                c.yPodOffset.set(-4.99393);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller largeTranslationalForward = Controller.proportional(.3);
                Controller smallTranslationalForward = Controller.proportional(.1);
                Controller smallTranslationalLateral = Controller.proportional(.1);
                Controller largeTranslationalLateral = Controller.proportional(.3);

                c.forwardTranslational.set(Controller.piecewise(smallTranslationalForward).put(2.5, largeTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(smallTranslationalLateral).put(2.5, largeTranslationalLateral));

                c.brake.set(Controller.proportionalFeedforward(0.005)); // 0.009

                c.maxBrakingPower.set(0.3);

                Controller largeHeading = Controller.proportional(2.4*3); // 2.4
                Controller smallHeading = Controller.proportional(1.3*3); // 1.3

                c.headingFeedback.set(Controller.piecewise(smallHeading).put(Math.PI/10, largeHeading));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0532, 0.0069));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0978, 0.0978));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016, 0.0016));

                c.maxAchievableForwardVelocity.set(73.7312);
                c.maxAchievableStrafeVelocity.set(56.2425);
                c.naturalForwardDeceleration.set(73.5);
                c.naturalStrafeDeceleration.set(65.03);

                c.coast.set(Controller.proportionalFeedforward(0.0105).plus(Controller.staticFeedforward(0.015)));

                c.cosineScale.set(false);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, driveConfig), new Foresight(foresightConfig));
    }
}