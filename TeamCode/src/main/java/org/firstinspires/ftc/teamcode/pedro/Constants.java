package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
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
 * There are two sets, chosen by the one switch {@link #target}: SIM (the PC simulator's values) and
 * ROBOT (the real robot's). Change the switch from Panels before an OpMode is initialised: the
 * follower is built from the selected set in create(), so it can't change under a running OpMode.
 * It defaults to ROBOT, so the robot never silently runs on simulator gains.
 *
 * Each set says whether its Foresight gains are tuned ({@link #foresightTuned()}). TeleOpDrive's
 * Foresight heading lock only uses a set that is, and falls back to its PID otherwise.
 */
@Configurable
public class Constants {
    /** Which set of constants to use. */
    public enum Target { SIM, ROBOT }

    /** The selected set. Editable in Panels (as a drop-down of SIM and ROBOT). */
    public static Target target = Target.ROBOT;

    /** The drivetrain: the same names and directions on the simulator and the robot. */
    private static KiwiConfig drivetrain() {
        return new KiwiConfig(c -> {
            c.motorAName.set("MotorA");
            c.motorBName.set("MotorB");
            c.motorCName.set("MotorC");
            c.motorADirection.set(DcMotorSimple.Direction.REVERSE);
            c.motorBDirection.set(DcMotorSimple.Direction.REVERSE);
            c.motorCDirection.set(DcMotorSimple.Direction.REVERSE);
        });
    }

    private static final KiwiConfig SIM_DRIVETRAIN = drivetrain();
    private static final KiwiConfig ROBOT_DRIVETRAIN = drivetrain();

    // The simulator's Pinpoint needs nothing set
    private static final PinpointConfig SIM_LOCALIZER = new PinpointConfig(c -> {
    });

    /**
     * The real robot's Pinpoint. Every value marked TODO is a placeholder, not a measurement: run
     * the Pinpoint tuner (Tuning.pinpointTuner) on the robot and copy its results in.
     */
    private static final PinpointConfig ROBOT_LOCALIZER = new PinpointConfig(c -> {
        // TODO: the Pinpoint's name in the robot configuration, if it isn't the default (c.name)
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

    // Foresight tuning from the simulator. This definitely won't match the real-world robot!
    private static final ForesightConfig SIM_FORESIGHT = new ForesightConfig(
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

    /**
     * The real robot's Foresight gains: NOT YET TUNED. These are the simulator's values, only so
     * Pedro has something to build the follower from. Run Tuning.foresightTuner on the robot, paste
     * its results here, then set ROBOT_FORESIGHT_TUNED to true.
     */
    private static final ForesightConfig ROBOT_FORESIGHT = new ForesightConfig(
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

    /** Set to true once ROBOT_FORESIGHT holds gains tuned on the real robot. */
    private static final boolean ROBOT_FORESIGHT_TUNED = false;

    private static boolean sim() {
        return target == Target.SIM;
    }

    /** The selected set's drivetrain constants. */
    public static KiwiConfig drivetrainConfig() {
        return sim() ? SIM_DRIVETRAIN : ROBOT_DRIVETRAIN;
    }

    /** The selected set's Pinpoint constants. */
    public static PinpointConfig localizerConfig() {
        return sim() ? SIM_LOCALIZER : ROBOT_LOCALIZER;
    }

    /** The selected set's Foresight gains. */
    public static ForesightConfig foresightConfig() {
        return sim() ? SIM_FORESIGHT : ROBOT_FORESIGHT;
    }

    /** Whether the selected set's Foresight gains are tuned for what it describes (SIM: yes, ROBOT: not yet). */
    public static boolean foresightTuned() {
        return sim() || ROBOT_FORESIGHT_TUNED;
    }

    /** Builds the follower from the selected set. */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig()),
                new Kiwi(h, drivetrainConfig()),
                new Foresight(foresightConfig())
        );
    }
}
