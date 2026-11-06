package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * What the Autonomous hands over to TeleOp: the alliance it ran for and where the robot finished.
 * Statics live on in the robot controller app between OpModes (until it restarts), so TeleOp can
 * start with the right alliance and pose. Empty if no auto has run since the app started.
 */
public class MatchState {
    private static Alliance alliance;
    private static Pose pose;

    private MatchState() {
    }

    /** Call at the end of an auto. */
    public static void save(Alliance endAlliance, Pose endPose) {
        alliance = endAlliance;
        pose = endPose;
    }

    /** The last auto's alliance, or null if none has run. */
    public static Alliance alliance() {
        return alliance;
    }

    /** Where the last auto finished, or null if none has run. */
    public static Pose pose() {
        return pose;
    }
}
