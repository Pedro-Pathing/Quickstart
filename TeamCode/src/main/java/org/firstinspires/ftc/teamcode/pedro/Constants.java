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
        c.frontLeftName.set("LF");
        c.frontRightName.set("RF");
        c.backLeftName.set("LR");
        c.backRightName.set("RR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);


        c.powerThreshold.set(0.01);
        c.manualBrakeMode.set(false);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("PP");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.9);
        c.yPodOffset.set(3.6);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18723189189699924);
                Controller secondaryTranslationalForward = Controller.proportional(0.06917718112764946);
                Controller primaryTranslationalLateral = Controller.proportional(0.2782993470558989);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1028241724416451);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009901877062633336));
                c.brake.set(Controller.proportionalFeedforward(0.008416595503238335));

                c.headingFeedback.set(Controller.proportional(2.8688588435685336));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04761952025132773, 0.004881957611424321));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06997101192746781, 0.05986005300180383));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011532201183191049, 0.0011542980661834567));

                c.maxAchievableForwardVelocity.set(100.38138018250847);
                c.maxAchievableStrafeVelocity.set(82.3232498992588);
                c.naturalForwardDeceleration.set(45.835986350090074);
                c.naturalStrafeDeceleration.set(64.93054558349891);
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
