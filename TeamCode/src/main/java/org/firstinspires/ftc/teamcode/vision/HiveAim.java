package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

/**
 * Chooses the field heading that points the robot at our HIVE, for auto-aim in TeleOp and Autonomous.
 *
 * With tags in view it points straight at the target CELL's open face (the camera's estimate).
 * Before the camera has seen tags it points roughly at our nearest CELL using odometry. Once the
 * camera has given a heading, that heading is kept if the tags drop out, rather than going back to
 * the rough one. Call reset() when a new aim starts.
 */
public class HiveAim {
    private boolean fromVision = false;
    private double heading = 0.0;

    /** Start a new aim: odometry until the camera sees tags. */
    public void reset() {
        fromVision = false;
    }

    /**
     * The heading to aim at this loop.
     *
     * @param target the camera's target this loop, or null if no tags are in view
     * @param pose   the robot's pose this loop (Pedro field frame)
     */
    public double update(Alliance alliance, HiveTarget target, Pose pose) {
        if (target != null) {
            fromVision = true;
            heading = target.targetHeading(pose.heading());
        } else if (!fromVision) {
            heading = HiveCells.headingToNearestCell(alliance, pose.x(), pose.y());
        }
        return heading;
    }

    /** Whether this aim has had a heading from the camera yet. */
    public boolean isFromVision() {
        return fromVision;
    }
}
