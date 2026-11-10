package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kiwi.Kiwi;
import org.firstinspires.ftc.teamcode.kiwi.KiwiConfig;

/**
 * The drive constants for Pedro Pathing: the drivetrain, the Pinpoint localizer and Foresight's gains.
 *
 * These are the real robot's values. The BIOBUZZ sim doesn't need its own set here: its virtual
 * Control Hub applies sim-only values from its vhub/sim-overrides.txt to these public static
 * fields by name, so keep their names (drivetrainConfig, localizerConfig, foresightConfig and
 * foresightTuned) as they are.
 *
 * Values marked TODO are placeholders until the robot is tuned.
 */
public class Constants {
    public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {
        c.motorAName.set("MotorA");
        c.motorBName.set("MotorB");
        c.motorCName.set("MotorC");
        c.motorADirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorBDirection.set(DcMotorSimple.Direction.REVERSE);
        c.motorCDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    /**
     * The Pinpoint. Every value marked TODO is a placeholder, not a measurement: run the Pinpoint
     * tuner (Tuning.pinpointTuner) on the robot and copy its results in.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        // The Pinpoint's name in the robot configuration. Pedro has no default for it (reading it
        // unset throws, which disables Drive), so it must be set. TODO: check it matches the robot's
        // configuration; "pinpoint" is the name the Pinpoint tuner suggests, and the sim's.
        c.name.set("pinpoint");
        // TODO: pod type (goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD), or c.ticksPerUnit for custom pods
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // TODO: pod offsets from the robot's centre of rotation (inches), from the tuner
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        // TODO: encoder directions, from the tuner (pushing forward must increase X, left must increase Y)
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Foresight's gains: NOT YET TUNED on the robot. These came from the simulator and definitely
     * won't match the real robot. Run Tuning.foresightTuner on the robot, paste its results here,
     * then set foresightTuned to true.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // TODO: replace every value below with the robot's tuned values
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

    /**
     * Whether foresightConfig holds gains tuned for the robot it's running on. TeleOpDrive's
     * Foresight heading lock is only used when it is: untuned gains make its brake prediction
     * wrong, so the PID heading lock is used instead. Set it to true once the robot is tuned. The
     * sim sets it to true in its sim-overrides.txt, as these gains came from the sim.
     */
    public static boolean foresightTuned = false;

    /** Builds the follower. */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Kiwi(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
