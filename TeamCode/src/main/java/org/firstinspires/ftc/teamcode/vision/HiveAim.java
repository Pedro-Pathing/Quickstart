package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Chooses the field heading that points the robot at our HIVE, for auto-aim in TeleOp and Autonomous.
 *
 * With tags in view it points straight at the target CELL's open face (the camera's estimate).
 * Before the camera has seen tags it points roughly at our nearest CELL using odometry. Once the
 * camera has given a heading, that heading is kept if the tags drop out, rather than going back to
 * the rough one. Call reset() when a new aim starts.
 *
 * Each camera image gives the target's bearing from where the robot was facing when it was taken,
 * so it's added to the heading then (from the pose history), once per image. Adding it to the
 * heading now instead, on every loop until the next image, would make the aim heading run ahead of
 * the robot as it turns and overshoot. Call update() every loop, aiming or not, to keep the history.
 */
@Configurable
public class HiveAim {
    /** The aim is locked (ready to shoot) within this of the aim heading... */
    public static double lockToleranceDegrees = 2.0;
    /** ...while turning slower than this (rad/s). Shared by TeleOp and the autos. */
    public static double lockSettleRate = 0.3;

    private final PoseHistory history = new PoseHistory();
    private boolean fromVision = false;
    private long frame = -1;
    private double heading = 0.0;

    /** Start a new aim: odometry until the camera sees tags. */
    public void reset() {
        fromVision = false;
        frame = -1;
    }

    /**
     * The heading to aim at this loop.
     *
     * @param target the camera's target this loop, or null if no tags are in view
     * @param pose   the robot's pose this loop (Pedro field frame)
     */
    public double update(Alliance alliance, HiveTarget target, Pose pose) {
        history.record(pose);
        if (target != null) {
            if (target.frame != frame) {
                frame = target.frame;
                fromVision = true;
                heading = target.targetHeading(history.at(target.captureNanos).heading());
            }
        } else if (!fromVision) {
            heading = HiveCells.headingToNearestCell(alliance, pose.x(), pose.y());
        }
        return heading;
    }

    /** Whether a heading error (radians) and turn rate (rad/s) count as locked on. */
    public static boolean isLocked(double headingError, double turnRate) {
        return Math.abs(headingError) < Math.toRadians(lockToleranceDegrees) && Math.abs(turnRate) < lockSettleRate;
    }

    /** Whether this aim has had a heading from the camera yet. */
    public boolean isFromVision() {
        return fromVision;
    }
}
