package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * What the Autonomous hands over to TeleOp: the alliance it ran for and where the robot finished.
 * Statics live on in the robot controller app between OpModes (until it restarts), so TeleOp can
 * start with the right alliance and pose. Empty if no auto has run since the app started.
 *
 * It also remembers when it was saved, so TeleOp can show the driver how old the pose is and let
 * them throw away a stale one from an earlier match.
 */
public class MatchState {
    private static Alliance alliance;
    private static Pose pose;
    private static long savedNanos;

    private MatchState() {
    }

    /** Call at the end of an auto. */
    public static void save(Alliance endAlliance, Pose endPose) {
        alliance = endAlliance;
        pose = endPose;
        savedNanos = System.nanoTime();
    }

    /** The last auto's alliance, or null if none has run. */
    public static Alliance alliance() {
        return alliance;
    }

    /** Where the last auto finished, or null if none has run. */
    public static Pose pose() {
        return pose;
    }

    /** Seconds since the last save, or NaN if nothing has been saved. */
    public static double ageSeconds() {
        if (pose == null) return Double.NaN;
        return (System.nanoTime() - savedNanos) / 1e9;
    }

    /** Forget the saved state (used by tests; the app clears it by restarting). */
    public static void clear() {
        alliance = null;
        pose = null;
        savedNanos = 0;
    }
}
