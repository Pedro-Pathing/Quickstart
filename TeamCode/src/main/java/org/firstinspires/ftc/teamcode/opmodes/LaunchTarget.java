package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;

/**
 * LaunchTarget - Enum defining all possible robot target positions with associated launcher speeds
 *
 * Each target has:
 * - Blue alliance pose
 * - Red alliance pose
 * - Launcher speed (in ticks per second)
 *
 * This makes it easy to add new targets and ensures launcher speeds are consistent
 * across autonomous and teleop modes.
 */
public enum LaunchTarget {
    FRONT_LAUNCH_AF(
            new Pose(8, 13.1, Math.toRadians(220)),          // Blue alliance pose
            new Pose(3.5, -20.9, Math.toRadians(135)),     // Red alliance pose
            RobotConstants.NEAR_LAUNCH_SPEED                                             // Launcher speed
    ),

    SIDE_LAUNCH_AF(
            new Pose(8, 13.1, Math.toRadians(220)),          // Blue alliance pose
            new Pose(3.5, -20.9, Math.toRadians(135)),     // Red alliance pose
            RobotConstants.NEAR_LAUNCH_SPEED                                             // Launcher speed
    ),

    FRONT_LAUNCH_SF(
            new Pose(8, 13.1, Math.toRadians(220)),          // Blue alliance pose
            new Pose(3.5, -20.9, Math.toRadians(135)),     // Red alliance pose
            RobotConstants.NEAR_LAUNCH_SPEED                                             // Launcher speed
    ),
    SIDE_LAUNCH_SF(
            new Pose(8, 13.1, Math.toRadians(220)),          // Blue alliance pose
            new Pose(3.5, -20.9, Math.toRadians(135)),     // Red alliance pose
            RobotConstants.NEAR_LAUNCH_SPEED                                             // Launcher speed
    ),
    // Autonomous end positions (no launcher)
    AUTON_END(
        new Pose(132, 48, Math.toRadians(270)),          // Blue alliance pose
        new Pose(12, 96, Math.toRadians(90)),        // Red alliance pose
        0                                                // No launcher
    ),

    // SF autonomous end positions (Red SF park; Blue pose currently unused)
    AUTON_END_SF(
        new Pose(132, 30, Math.toRadians(90)),          // Blue alliance pose
        new Pose(12, 118, Math.toRadians(270)),          // Red alliance pose
        0                                                // No launcher
    ),
    // AF autonomous end positions (Blue AF park; Red pose currently unused) - same poses as AUTON_END_SF for now
    AUTON_END_AF(
            new Pose(132, 30, Math.toRadians(90)),          // Blue alliance pose
            new Pose(12, 118, Math.toRadians(270)),          // Red alliance pose
            0                                                // No launcher
    ),

    // Home/starting positions
    ALLIANCE_HOME(
        new Pose(132, 48, Math.toRadians(270)),                          // Blue alliance pose
        new Pose(12, 96, Math.toRadians(90)),                           // Red alliance pose
        0                                                // No launcher
    ),

    INIT_POSE(
        new Pose(135, 81, Math.toRadians(180)),         // Blue alliance init position
        new Pose(9, 63, Math.toRadians(0)),      // Red alliance init position (next to the flower in red zone)
        0                                                // No launcher
    );

    private final Pose bluePose;
    private final Pose redPose;
    private final int launcherSpeed;

    /**
     * Constructor for LaunchTarget
     * @param bluePose Pose for Blue alliance
     * @param redPose Pose for Red alliance
     * @param launcherSpeed Launcher speed in ticks per second (0 for non-launch positions)
     */
    LaunchTarget(Pose bluePose, Pose redPose, int launcherSpeed) {
        this.bluePose = bluePose;
        this.redPose = redPose;
        this.launcherSpeed = launcherSpeed;
    }

    /**
     * Get the pose for the specified alliance
     * @param isBlueAlliance true for Blue alliance, false for Red alliance
     * @return The alliance-specific pose
     */
    public Pose getPose(boolean isBlueAlliance) {
        return isBlueAlliance ? bluePose : redPose;
    }

    /**
     * Get the launcher speed for this target
     * @return Launcher speed in ticks per second
     */
    public int getLauncherSpeed() {
        return launcherSpeed;
    }

    /**
     * Check if this target requires the launcher
     * @return true if launcher speed > 0
     */
    public boolean requiresLauncher() {
        return launcherSpeed > 0;
    }
}
