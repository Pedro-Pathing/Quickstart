package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.FloorBall;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.vision.PoseHistory;

import java.util.List;

/**
 * Drives the robot to collect the nearest ball it's allowed: Pollen, or Nectar of our colour.
 *
 * start() tilts the camera down at the floor, then:
 *  SEARCHING  holds position and picks the nearest ball of ours that the other alliance's Nectar
 *             isn't in the way of (none within half a robot width of the straight line to it, nearer
 *             than it). Their Nectar is never a target.
 *  TURNING    turns on the spot to face the ball.
 *  DRIVING    runs the intake and drives along a line through the ball's position to driveBeyondBall
 *             past it, facing along the line. Each new camera frame updates where the ball is and
 *             re-aims the line, until the ball is within steerUntilDistance; then the line is
 *             fixed, so the robot doesn't turn round as it passes over the ball's spot. Once the
 *             ball passes under the camera, the robot carries on along the line using odometry.
 * It finishes when the intake reports a pickup or the robot passes the end of the line, then
 * runs the intake for intakeAfterSeconds more and tilts the camera back up. If their Nectar comes
 * into the way while turning or driving, it goes back to SEARCHING and picks again. It gives up if
 * nothing suitable is seen for searchTimeoutSeconds or a collect takes over collectTimeoutSeconds.
 *
 * While active it drives the follower with hold(), so the OpMode must not drive it as well.
 * Each camera image is used once, and its balls are turned into field positions with the robot's
 * pose when the image was captured (from the pose history), so the camera's latency doesn't put
 * them out while the robot turns. Call update() every loop, active or not, to keep the history.
 */
@Configurable
public class CollectAssist {
    /** Facing the ball within this, and turning slower than turnSettleRate (rad/s), ends TURNING. */
    public static double aimToleranceDegrees = 4.0;
    public static double turnSettleRate = 0.5;
    /** How far past the ball's centre to drive (m), so the intake sweeps right over it. */
    public static double driveBeyondBall = 0.10;
    /** The robot's half width (m), for checking whether their Nectar is in the way. PLACEHOLDER. */
    public static double robotHalfWidth = 0.23;
    /** A ball seen within this (m) of the target's last field position is taken to be the target. */
    public static double matchRadius = 0.15;
    /** Getting within this (m) of the end of the line, or past it, counts as done. */
    public static double arriveTolerance = 0.04;
    /** Once the ball is this close (m) to the robot's centre, the line stops being re-aimed. */
    public static double steerUntilDistance = 0.35;
    public static double searchTimeoutSeconds = 1.5;
    public static double collectTimeoutSeconds = 4.0;
    /** The intake keeps running this long after a collect, to pull the ball fully in. */
    public static double intakeAfterSeconds = 0.5;

    public enum State { IDLE, SEARCHING, TURNING, DRIVING, FINISHING }

    private static final double METRES_TO_INCHES = 39.3701;

    private final Follower follower;
    private final LimelightCamera camera;
    private final Intake intake;
    private final PoseHistory history = new PoseHistory();
    /** The last camera image used, so each is only used once. */
    private long frame = -1;

    private State state = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private String result = "";

    /** Where the robot was when the collect started, held while searching and turning (Pedro inches). */
    private double holdX, holdY, holdHeading;
    /** The target's kind and last known field position (Pedro inches). */
    private FloorBall.Kind targetKind;
    private double targetX, targetY;
    /** While DRIVING: the line's heading and its end (Pedro inches). */
    private double lineHeading, endX, endY;
    private int ballsSeen = 0;

    public CollectAssist(Follower follower, LimelightCamera camera, Intake intake) {
        this.follower = follower;
        this.camera = camera;
        this.intake = intake;
    }

    /** Start a collect (ignored if the camera can't tilt down). */
    public void start() {
        if (!camera.isFloorAvailable()) {
            result = "no floor camera";
            return;
        }
        Pose pose = follower.pose();
        holdX = pose.x();
        holdY = pose.y();
        holdHeading = pose.heading();
        camera.setMode(LimelightCamera.Mode.FLOOR);
        setState(State.SEARCHING);
        result = "";
    }

    /** Stop now: the intake stops, the camera tilts back up and the OpMode drives again. */
    public void cancel() {
        if (state == State.IDLE) return;
        finish("cancelled");
        intake.stop();
        setState(State.IDLE);
    }

    /** Whether it's driving the robot (the OpMode should leave the follower alone). */
    public boolean isActive() {
        return state != State.IDLE && state != State.FINISHING;
    }

    public State getState() {
        return state;
    }

    /**
     * Call every loop after follower.update().
     *
     * @param balls this loop's camera.findBalls() (null when there's no fresh floor result)
     */
    public void update(List<FloorBall> balls, Alliance alliance) {
        Pose pose = follower.pose();
        history.record(pose);
        if (balls != null) ballsSeen = balls.size();
        // An image already used, or one with no balls in, has nothing new
        if (balls != null && (balls.isEmpty() || balls.get(0).frame == frame)) balls = null;
        // Where the robot was when the image was captured
        Pose seenFrom = pose;
        if (balls != null) {
            frame = balls.get(0).frame;
            Pose then = history.at(balls.get(0).captureNanos);
            if (then != null) seenFrom = then;
        }
        switch (state) {
            case IDLE:
                return;
            case SEARCHING:
                follower.hold(new Pose(holdX, holdY, holdHeading));
                if (balls != null && chooseTarget(balls, alliance, seenFrom)) {
                    setState(State.TURNING);
                } else if (stateTimer.seconds() > searchTimeoutSeconds) {
                    finish("no ball of ours in reach");
                    setState(State.IDLE);
                }
                break;
            case TURNING:
            case DRIVING:
                if (balls != null && !trackTarget(balls, alliance, seenFrom)) {
                    // Their Nectar is now in the way: stop here and pick again
                    holdX = pose.x();
                    holdY = pose.y();
                    holdHeading = pose.heading();
                    intake.stop();
                    setState(State.SEARCHING);
                    break;
                }
                if (state == State.TURNING) {
                    double heading = Math.atan2(targetY - pose.y(), targetX - pose.x());
                    follower.hold(new Pose(holdX, holdY, heading));
                    double error = Math.atan2(Math.sin(heading - pose.heading()), Math.cos(heading - pose.heading()));
                    if (Math.abs(error) < Math.toRadians(aimToleranceDegrees)
                            && Math.abs(follower.twist().omega) < turnSettleRate) {
                        aimLine(pose);
                        intake.run();
                        setState(State.DRIVING);
                    }
                } else {
                    // Re-aim the line at the ball's latest position until it's close
                    if (balls != null && Math.hypot(targetX - pose.x(), targetY - pose.y())
                            > steerUntilDistance * METRES_TO_INCHES) {
                        aimLine(pose);
                    }
                    follower.hold(new Pose(endX, endY, lineHeading));
                    intake.update();
                    // How far short of the end of the line the robot is, along the line (negative once past it)
                    double shortBy = (endX - pose.x()) * Math.cos(lineHeading) + (endY - pose.y()) * Math.sin(lineHeading);
                    boolean arrived = shortBy < arriveTolerance * METRES_TO_INCHES;
                    if (intake.hasPickedUp()) {
                        finish("picked up " + targetKind);
                    } else if (arrived) {
                        finish("reached " + targetKind + " (no pickup detected)");
                    } else if (stateTimer.seconds() > collectTimeoutSeconds) {
                        finish("timed out");
                    }
                }
                break;
            case FINISHING:
                intake.update();
                if (stateTimer.seconds() > intakeAfterSeconds) {
                    intake.stop();
                    setState(State.IDLE);
                }
                break;
        }
    }

    public void addTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("Collect", state);
        telemetryM.addData("Balls Seen", ballsSeen);
        if (state == State.TURNING || state == State.DRIVING) {
            Pose pose = follower.pose();
            telemetryM.addData("Collect Target", targetKind);
            telemetryM.addData("Collect Distance (m)",
                    Math.hypot(targetX - pose.x(), targetY - pose.y()) / METRES_TO_INCHES);
        }
        if (intake.isRunning()) {
            telemetryM.addData("Intake Current (A)", intake.getCurrentAmps());
            telemetryM.addData("Intake Baseline (A)", intake.getBaselineAmps());
        }
        if (!result.isEmpty()) telemetryM.addData("Last Collect", result);
    }

    /** Aim the line from where the robot is through the ball, ending driveBeyondBall past it. */
    private void aimLine(Pose pose) {
        lineHeading = Math.atan2(targetY - pose.y(), targetX - pose.x());
        double beyond = driveBeyondBall * METRES_TO_INCHES;
        endX = targetX + beyond * Math.cos(lineHeading);
        endY = targetY + beyond * Math.sin(lineHeading);
    }

    /** Pick the nearest ball of ours that isn't blocked. False if there isn't one. */
    private boolean chooseTarget(List<FloorBall> balls, Alliance alliance, Pose pose) {
        FloorBall best = null;
        for (FloorBall ball : balls) {
            if (!ball.isOurs(alliance) || isBlocked(ball, balls, alliance)) continue;
            if (best == null || ball.distance() < best.distance()) best = ball;
        }
        if (best == null) return false;
        targetKind = best.kind;
        double[] field = toField(best, pose);
        targetX = field[0];
        targetY = field[1];
        return true;
    }

    /**
     * Update the target's position from a new frame, if it's in view. False if their Nectar is now in
     * the way of it. If the target isn't in view (e.g. it has passed under the camera) its last known
     * position is kept.
     */
    private boolean trackTarget(List<FloorBall> balls, Alliance alliance, Pose pose) {
        FloorBall match = null;
        double matchDistance = matchRadius * METRES_TO_INCHES;
        for (FloorBall ball : balls) {
            if (ball.kind != targetKind) continue;
            double[] field = toField(ball, pose);
            double d = Math.hypot(field[0] - targetX, field[1] - targetY);
            if (d < matchDistance) {
                match = ball;
                matchDistance = d;
            }
        }
        if (match == null) return true;
        if (isBlocked(match, balls, alliance)) return false;
        double[] field = toField(match, pose);
        targetX = field[0];
        targetY = field[1];
        return true;
    }

    /** Is any of their Nectar nearer than the ball and within half a robot width of the line to it? */
    private static boolean isBlocked(FloorBall target, List<FloorBall> balls, Alliance alliance) {
        double length = target.distance();
        if (length < 1e-6) return false;
        double ux = target.x / length, uy = target.y / length;
        for (FloorBall ball : balls) {
            if (!ball.isTheirNectar(alliance)) continue;
            double along = ball.x * ux + ball.y * uy;
            double across = Math.abs(ball.x * uy - ball.y * ux);
            if (along > 0 && along < length && across < robotHalfWidth + ball.kind.diameter / 2) return true;
        }
        return false;
    }

    /** A ball's robot-frame position (m) as a field position (Pedro inches). */
    private static double[] toField(FloorBall ball, Pose pose) {
        double cos = Math.cos(pose.heading()), sin = Math.sin(pose.heading());
        double x = ball.x * METRES_TO_INCHES, y = ball.y * METRES_TO_INCHES;
        return new double[] {pose.x() + x * cos - y * sin, pose.y() + x * sin + y * cos};
    }

    /** End the drive part of a collect: the intake runs on briefly and the camera tilts back up. */
    private void finish(String why) {
        result = why;
        camera.setMode(LimelightCamera.Mode.TAGS);
        setState(State.FINISHING);
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
}
