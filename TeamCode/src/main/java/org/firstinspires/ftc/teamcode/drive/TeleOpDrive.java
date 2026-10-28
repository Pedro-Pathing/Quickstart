package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
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
 */
@Configurable
public class TeleOpDrive {
    // Heading lock gains (turn power per radian of error, and per rad/s of angular velocity for kD)
    public static double kP = 0.8;
    public static double kI = 0.0;
    public static double kD = 0.08;
    public static double kS = 0.0; // Static feedforward to overcome turning friction

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
        if (gamepad.circleWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        heading = follower.pose().heading();
        angularVelocity = follower.twist().omega;

        double speed;
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
        double forward = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turnInput = -gamepad.right_stick_x;

        // Speed scaling applies to driver input only, so the heading lock keeps full authority
        powers = new DrivePowers(forward * speed, strafe * speed, turnInput * turnScale * speed);

        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(powers, heading);
        }

        if (Math.abs(turnInput) > turnDeadband) {
            // Driver is turning: pass the stick straight through (this also ends auto-aim)
            headingState = HeadingState.TURNING;
            targetHeading = heading;
        } else {
            if (headingState == HeadingState.AIMING) {
                if (aimHeading != null) {
                    targetHeading = aimHeading.getAsDouble();
                }
            } else if (headingState != HeadingState.LOCKED) {
                headingState = HeadingState.SETTLING;
                targetHeading = heading;

                if (Math.abs(angularVelocity) < settleRate) {
                    lockHeading();
                }
            }

            // While settling the error is zero, so only kD acts and brakes the rotation
            powers = ManualDrive.headingLock(follower, headingController, powers, targetHeading);
        }

        follower.manual(powers);
    }

    public void addTelemetry(TelemetryManager telemetryM) {
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

    private void lockHeading() {
        targetHeading = follower.pose().heading();
        headingController.reset();
        headingState = HeadingState.LOCKED;
    }
}
