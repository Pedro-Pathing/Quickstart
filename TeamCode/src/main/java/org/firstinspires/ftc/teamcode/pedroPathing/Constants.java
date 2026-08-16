package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static double kP = 0.03;
    public static double headingFF = 0.065;
    public static double velocityFF = 0.009;
    public static double accelFF = 0.001;

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lb");
                c.frontRightName.set("rf");
                c.backRightName.set("rb");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.ticksPerUnit.set(19.89436789);
        c.encoderResolutionUnit.set(DistanceUnit.MM);
        c.headingScalar.set(1.0168);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.xPodOffset.set(-3.95);
        c.yPodOffset.set(-5.67);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller largeTranslationalForward = Controller.pid(0.07,0,0).plus(Controller.staticFeedforward(0.01));
                Controller smallTranslationalForward = Controller.pid(0.07,0,0);
                Controller smallTranslationalLateral = Controller.pid(.12,0,0).plus(Controller.staticFeedforward(0.0005));
                Controller largeTranslationalLateral = Controller.pid(.12,0,0).plus(Controller.staticFeedforward(0.01));

                c.forwardTranslationalController.set(Controller.piecewise(Controller.staticFeedforward(0)).put(0.5, smallTranslationalForward).put(2.5, largeTranslationalForward));
                c.lateralTranslationalController.set(Controller.piecewise(Controller.staticFeedforward(0)).put(0.5, smallTranslationalLateral).put(2.5, largeTranslationalLateral));

                c.brakeController.set(Controller.pid(kP, 0, 0).plus(Controller.dynamicFeedforward(velocityFF)));
                c.brakeAccelFeedforward.set(Controller.dynamicFeedforward(accelFF));

                c.maxBrakingPower.set(0.3);

                Controller largeHeading = Controller.pid(2.28, 0, 0.29).plus(Controller.staticFeedforward(0.01));
                c.headingController.set(largeHeading);
                c.headingFeedforward.set(Controller.dynamicFeedforward(headingFF));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0633, 0.0633));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00146, 0.00146));

                c.centripetalScaling.set(0.0005);
                c.cosineScale.set(false);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new OctoQuadLocalizer(h, localizerConfig), new Mecanum(h, driveConfig), new Foresight(foresightConfig));
    }
}
