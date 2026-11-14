package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(1.8778218246820406);
        c.yPodOffset.set(-2.1729906337467706);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1974425785837628);
                Controller secondaryTranslationalForward = Controller.proportional(0.07294975702383548);
                Controller primaryTranslationalLateral = Controller.proportional(0.5852026317659119);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21621673553519472);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01671512161594036));
                c.brake.set(Controller.proportionalFeedforward(0.014207853373549306));

                c.headingFeedback.set(Controller.proportional(3.480712538783216));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.053632461218862254, 0.0037403853906236065));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07917214716608789, 0.057255194736108996));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010466245700306454, 0.0016025222791429849));

                c.maxAchievableForwardVelocity.set(60.19046133645846);
                c.maxAchievableStrafeVelocity.set(49.676985361398735);
                c.naturalForwardDeceleration.set(30.49440244712893);
                c.naturalStrafeDeceleration.set(61.91547052501994);
            });

}