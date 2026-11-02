package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Limelight mounting and HIVE AprilTag settings.
 *
 * Robot frame: +X forward, +Y left, +Z up, origin at the robot's centre of rotation on the floor.
 * Lengths are in metres, angles in degrees.
 *
 * TODO: the camera mount values are placeholders until the camera is fitted.
 */
@Configurable
public class LimelightConstants {
    /** Name of the Limelight in the robot configuration. */
    public static String deviceName = "limelight";
    /** Limelight pipeline set up for 36h11 AprilTags (tag size 82.5 mm). */
    public static int aprilTagPipeline = 0;
    public static int pollRateHz = 100;
    /** Results older than this are ignored. */
    public static long maxStalenessMs = 100;

    // Camera lens position on the robot (centred; forward position not yet decided)
    public static double cameraForward = 0.0;
    public static double cameraLeft = 0.0;
    public static double cameraUp = 0.30;

    // Camera orientation on the robot. All zero = level, facing straight forward.
    /** Positive turns the camera to the left. */
    public static double cameraYaw = 0.0;
    /** Positive tilts the camera upwards. 40 keeps the near CELL's tags in view from 0.52 m to 2.74 m. */
    public static double cameraPitch = 40.0;
    /** Right-hand rotation about the camera's forward axis. Usually 0. */
    public static double cameraRoll = 0.0;

    // HIVE AprilTag IDs: four tags under each CELL
    public static final int RED_SCORING_FIRST_TAG = 30;
    public static final int RED_AUDIENCE_FIRST_TAG = 34;
    public static final int BLUE_AUDIENCE_FIRST_TAG = 38;
    public static final int BLUE_SCORING_FIRST_TAG = 42;
    public static final int TAGS_PER_CELL = 4;

    /**
     * How far each tag's face points below horizontal. The tags are on the HIVE's underside, which
     * slopes up 30 degrees towards the near side, so they face 60 degrees downwards towards the robot.
     * Assumes each tag's top edge is its higher edge. 0 = an upright tag.
     */
    public static double tagFacingDownDegrees = 60.0;

    /**
     * Tags lower than this are under the lowered CELL. Those read upside down (top edge lowest), which
     * targetFromTag doesn't handle, so they're ignored. Midway between the raised CELL's tags (1.265 m)
     * and the lowered CELL's (0.902 m).
     */
    public static double minTagHeight = 1.08;

    /**
     * Offset from each tag's centre to the centre of its CELL's open face, indexed by the tag's
     * position in its CELL (tag ID minus the CELL's first tag ID). Each row is
     * {right, out, up}: right as seen when facing the tag, out = away from the HIVE towards the field
     * (negative = into the HIVE), up = vertically upwards.
     *
     * Measured from the field CAD for the raised (up-facing) CELL, the same for all four CELLs:
     * tags at -6.5, -2.75, +2.75 and +6.5 in along the sticker, centres 1.265 m up; the open face
     * centre is 1.484 m up, facing 29.3 degrees above horizontal. The sticker is where the competition
     * manual puts it (TU05, Figure 9-15: its reference holes 9.938 in from the front of the CELL, the
     * tag centres 2.75 in above them). Until 2026-11-02 these were 33 mm too deep into the CELL
     * (out 0.1159, up 0.2356, tags 1.248 m up).
     */
    public static double[][] tagToTargetOffset = {
            { 0.1651, 0.0870, 0.2189},
            { 0.0698, 0.0870, 0.2189},
            {-0.0699, 0.0870, 0.2189},
            {-0.1651, 0.0870, 0.2189}
    };
}
