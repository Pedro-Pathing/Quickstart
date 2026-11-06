package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kiwi.Kiwi;
import org.firstinspires.ftc.teamcode.kiwi.KiwiConfig;

public class Constants {
    public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {
        c.motorAName.set("MotorA");
        c.motorBName.set("MotorB");
        c.motorCName.set("MotorC");
        c.motorADirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorBDirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorCDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    });

    //Foresight Tuning from the simulator. This definitely won't match the real-world robot!
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.7058736533290159);
                Controller secondaryTranslationalForward = Controller.proportional(0.260801453613681);
                Controller primaryTranslationalLateral = Controller.proportional(0.7062089426590505);
                Controller secondaryTranslationalLateral = Controller.proportional(0.26092533406203294);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01582613564392006));
                c.brake.set(Controller.proportionalFeedforward(0.013452215297332051));

                c.headingFeedback.set(Controller.proportional(0.8443331616844043));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.23754553470190187, 0.004642628916502741));

                c.linearBrakeCoefficients.set(Matrix.diag(0.15680562835551934, 0.2145569720219104));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.004673740969314629, 0.0028396009010192933));

                c.maxAchievableForwardVelocity.set(68.6950004647309);
                c.maxAchievableStrafeVelocity.set(68.6901987579687);
                c.naturalForwardDeceleration.set(18.51205894546473);
                c.naturalStrafeDeceleration.set(18.202745356116683);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Kiwi(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
