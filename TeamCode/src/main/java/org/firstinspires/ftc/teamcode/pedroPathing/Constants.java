package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.algorithm.ForesightV3;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
                Controller largeTranslationalForward = Controller.pid(0.0206,0,0).plus(Controller.staticFeedforward(0.0));
                Controller smallTranslationalForward = Controller.pid(0.0138,0,0);
                Controller smallTranslationalLateral = Controller.pid(.0340,0,0).plus(Controller.staticFeedforward(0.000));
                Controller largeTranslationalLateral = Controller.pid(.0227,0,0).plus(Controller.staticFeedforward(0.0));

                c.forwardTranslational.set(Controller.piecewise(smallTranslationalForward).put(2.5, largeTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(smallTranslationalLateral).put(2.5, largeTranslationalLateral));

                c.brake.set(Controller.proportionalFeedforward(0.009));

                c.maxBrakingPower.set(0.3);

                Controller largeHeading = Controller.pid(2.4, 0, 0);
                c.headingFeedback.set(largeHeading);

                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0532, 0.0069));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0978, 0.0978));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016, 0.0016));

                c.maxAchievableForwardVelocity.set(73.7312);
                c.maxAchievableStrafeVelocity.set(56.2425);
                c.naturalForwardDeceleration.set(73.5);
                c.naturalStrafeDeceleration.set(65.03);

                c.maxAchievableStrafeVelocity.set(150.0);
                c.maxAchievableForwardVelocity.set(150.0);

                c.naturalForwardDeceleration.set(64.33);
                c.naturalStrafeDeceleration.set(48.43);

                c.coast.set(Controller.proportionalFeedforward(0.0105).plus(Controller.staticFeedforward(0.015)));

                c.cosineScale.set(false);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, driveConfig), new ForesightV3(foresightConfig));
    }
}
