package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.helpers.SystemStatus;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

/**
 * Driver control for the TeleOps, using Pedro Pathing 3's manual drive and heading lock.
 *
 * Left stick translates, right stick turns. Releasing the right stick brakes the rotation and then
 * holds the heading the robot settled at. Circle toggles robot/field-centric.
 * Hold the left bumper for slow mode or the right bumper for sprint.
 * Optional auto-aim (startAiming) holds headings given by the OpMode until the driver turns.
 * Gains are live-tunable from Panels.
 *
 * With useForesight, the heading lock works on where the robot would stop rather than where it is
 * now, as Pedro's Foresight does: the heading it would still turn through if it braked now (from
 * Constants' headingBrakeCoefficients, the Foresight tuner's coasting stop, times brakeScale) is
 * taken off the error. It pushes while that stop falls short of the target and brakes once it
 * would overshoot, so it can use a high gain without overshooting. Otherwise it's the PID above.
 * That includes when Constants.foresightTuned is false (on the robot, until it is tuned): the PID
 * is used until the team tunes Foresight, and the telemetry says which is in use.
 */
@Configurable
public class TeleOpDrive {
    // Heading lock gains (turn power per radian of error, and per rad/s of angular velocity for kD).
    // Tuned in the sim's auto-aim test (BiobuzzHub.Aim): retune on the robot.
    public static double kP = 1.2;
    public static double kI = 0.0;
    public static double kD = 0.2;
    public static double kS = 0.03; // Static feedforward to overcome turning friction

    // Foresight heading lock (see the class comment): turn power per radian of the predicted error,
    // kS while that error is over foresightKSDeadbandDegrees, and a scale on the brake model
    public static boolean useForesight = true;
    public static double foresightKP = 4.0;
    public static double foresightKS = 0.03;
    public static double foresightKSDeadbandDegrees = 0.5;
    public static double brakeScale = 0.6;

    // Driver input tuning
    public static double turnDeadband = 0.05;
    public static double turnScale = 1.0;

    // Speed scales: hold left bumper for slow, right bumper for sprint, neither or both for normal
    public static double slowSpeed = 0.25;
    public static double normalSpeed = 0.66;
    public static double sprintSpeed = 1.0;

    /** Angular velocity (rad/s) below which the robot is considered settled and the heading is locked. */
    public static double settleRate = 0.3;

    public enum HeadingState { TURNING, SETTLING, LOCKED, AIMING }

    private final Follower follower;
    private final Controller headingController;

    private HeadingState headingState = HeadingState.LOCKED;
    private double targetHeading = 0.0;
    private boolean fieldCentric = false;

    // From the last update, for telemetry
    private String speedMode = "Normal";
    private double heading = 0.0;
    private double angularVelocity = 0.0;
    private DrivePowers powers = DrivePowers.zero();
    private double speed = normalSpeed;
    private double forwardInput, strafeInput, turnInput;
    /** The heading that counts as "away from the driver" for field-centric (resetFieldHeading()). */
    private double fieldHeadingOffset = 0.0;

    /**
     * Builds the follower at startingPose, or returns null (and notes Drive as disabled in status) if
     * a drive motor or the localizer is missing, so the rest of the robot can still be tested.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Pose startingPose, SystemStatus status) {
        try {
            Follower follower = Constants.create(hardwareMap);
            follower.setPose(startingPose);
            follower.update();
            return follower;
        } catch (RuntimeException e) {
            String reason = e.getMessage() != null ? e.getMessage() : e.toString();
            RobotLog.ww("TeleOpDrive", "Drive disabled: %s", reason);
            status.disable("Drive", reason);
            return null;
        }
    }

    public TeleOpDrive(Follower follower) {
        this.follower = follower;
        // Suppliers let the gains be changed from Panels while running
        headingController = Controller.sum(
                Controller.proportional(() -> kP),
                Controller.integral(() -> kI),
                Controller.derivative(() -> kD),
                Controller.staticFeedforward(() -> kS)
        );
    }

    /** Call from the OpMode's start(): holds the heading the robot starts at. */
    public void start() {
        lockHeading();
    }

    /** Auto-aim: hold the headings update() is given until the driver turns. */
    public void startAiming() {
        headingState = HeadingState.AIMING;
        headingController.reset();
    }

    public boolean isAiming() {
        return headingState == HeadingState.AIMING;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    /** Call every loop, after follower.update(). */
    public void update(Gamepad gamepad) {
        update(gamepad, null);
    }

    /**
     * Call every loop, after follower.update().
     *
     * @param aimHeading the field heading to hold while aiming; only asked for while aiming and the
     *                   driver isn't turning
     */
    public void update(Gamepad gamepad, DoubleSupplier aimHeading) {
        readDriver(gamepad);
        double turn;
        if (headingState == HeadingState.AIMING && !driverTurning()) {
            turn = aimHeading != null ? aimTurn(aimHeading.getAsDouble()) : lockTurn();
        } else {
            turn = driverTurn();
        }
        DrivePowers translation = translation();
        powers = new DrivePowers(translation.forward(), translation.strafe(), turn);
        follower.manual(powers);
    }

    /*
     * The parts of update(), for an OpMode that drives the translation and the turn separately (the
     * two-gamepad TeleOp's Ivy commands): readDriver() once a loop, then translation() and either
     * driverTurn() or aimTurn(), then drive().
     */

    /** Read the driver's sticks, bumpers and circle (robot/field-centric) for this loop. */
    public void readDriver(Gamepad gamepad) {
        if (gamepad.circleWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        heading = follower.pose().heading();
        angularVelocity = follower.twist().omega;

        if (gamepad.left_bumper && !gamepad.right_bumper) {
            speedMode = "Slow";
            speed = slowSpeed;
        } else if (gamepad.right_bumper && !gamepad.left_bumper) {
            speedMode = "Sprint";
            speed = sprintSpeed;
        } else {
            speedMode = "Normal";
            speed = normalSpeed;
        }

        // +forward is forward, +strafe is left and +turn is CCW, so the X axes are negated
        forwardInput = -gamepad.left_stick_y;
        strafeInput = -gamepad.left_stick_x;
        turnInput = -gamepad.right_stick_x;
    }

    /** This loop's translation from the left stick, with the speed mode and field-centric applied (no turn). */
    public DrivePowers translation() {
        DrivePowers translation = new DrivePowers(forwardInput * speed, strafeInput * speed, 0.0);
        return fieldCentric ? ManualDrive.fieldCentric(translation, heading, -fieldHeadingOffset) : translation;
    }

    /** Whether the driver is turning with the right stick this loop. */
    public boolean driverTurning() {
        return Math.abs(turnInput) > turnDeadband;
    }

    /**
     * The driver's turn: the right stick while it's moved, then braking and holding the heading the
     * robot settles at. Leaving AIMING this way settles at wherever the aim left the robot.
     */
    public double driverTurn() {
        if (driverTurning()) {
            headingState = HeadingState.TURNING;
            targetHeading = heading;
            // Speed scaling applies to driver input only, so the heading lock keeps full authority
            return turnInput * turnScale * speed;
        }
        if (headingState != HeadingState.LOCKED) {
            headingState = HeadingState.SETTLING;
            targetHeading = heading;
            if (Math.abs(angularVelocity) < settleRate) {
                lockHeading();
            }
        }
        return lockTurn();
    }

    /** Turn to hold this field heading (auto-aim's). */
    public double aimTurn(double aimHeading) {
        if (headingState != HeadingState.AIMING) {
            headingState = HeadingState.AIMING;
            headingController.reset();
        }
        targetHeading = aimHeading;
        return lockTurn();
    }

    /** Brake and then hold wherever the robot settles, e.g. when the heading lock takes over again after an assist. */
    public void holdCurrentHeading() {
        headingState = HeadingState.SETTLING;
        targetHeading = follower.pose().heading();
        headingController.reset();
    }

    /** Make the robot's current heading "away from the driver" for field-centric driving. */
    public void resetFieldHeading() {
        fieldHeadingOffset = follower.pose().heading();
    }

    /** Drive with this translation and turn. */
    public void drive(DrivePowers translation, double turn) {
        powers = new DrivePowers(translation.forward(), translation.strafe(), turn);
        follower.manual(powers);
    }

    /** The turn power that holds targetHeading, with the Foresight lock or the PID. */
    private double lockTurn() {
        if (usingForesight()) return foresightTurn();
        // While settling the error is zero, so only kD acts and brakes the rotation
        return ManualDrive.headingLock(follower, headingController, DrivePowers.zero(), targetHeading).turn();
    }

    /**
     * Whether the heading lock is the Foresight one: it is switched on and Constants marks its
     * Foresight gains as tuned (Constants.foresightTuned). Untuned gains would
     * make the brake prediction wrong, so the PID is used instead.
     */
    boolean usingForesight() {
        return useForesight && Constants.foresightTuned;
    }

    public void addTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("Heading Lock", usingForesight() ? "Foresight"
                : useForesight ? "PID (Foresight gains not tuned)" : "PID");
        telemetryM.addData("Drive Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetryM.addData("Speed Mode", speedMode);
        telemetryM.addData("Heading State", headingState);
        telemetryM.addData("Target Heading", Math.toDegrees(targetHeading));
        telemetryM.addData("Actual Heading", Math.toDegrees(heading));
        telemetryM.addData("Angular Velocity", Math.toDegrees(angularVelocity));
        telemetryM.addData("Turn Power", powers.turn());
        telemetryM.addData("X", follower.pose().x());
        telemetryM.addData("Y", follower.pose().y());
    }

    /**
     * Turn power from the error left after the robot's predicted stop. While settling the target is
     * where the robot is now, so this brakes it.
     */
    private double foresightTurn() {
        Vector2D brake = Constants.foresightConfig.headingBrakeCoefficients.get();
        double omega = angularVelocity;
        double stop = brakeScale * (brake.x() * omega + brake.y() * omega * omega * Math.signum(omega));
        double error = Angle.normalizeSigned(targetHeading - heading) - stop;
        double turn = foresightKP * error;
        if (Math.abs(error) > Math.toRadians(foresightKSDeadbandDegrees)) {
            turn += Math.signum(error) * foresightKS;
        }
        return Math.max(-1.0, Math.min(1.0, turn));
    }

    private void lockHeading() {
        targetHeading = follower.pose().heading();
        headingController.reset();
        headingState = HeadingState.LOCKED;
    }
}
