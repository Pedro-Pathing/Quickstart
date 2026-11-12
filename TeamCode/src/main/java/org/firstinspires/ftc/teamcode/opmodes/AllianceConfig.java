package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;

/**
 * Configuration class that holds alliance-specific constants for both TeleOp and Autonomous modes.
 * This allows the same base classes to work for both Blue and Red alliances
 * by simply providing different configuration objects.
 *
 * Now uses LaunchTarget enum to manage poses and launcher speeds consistently.
 */
public class AllianceConfig {
    private final boolean isBlueAlliance;
    private final String targetAprilTagIds;
    private final String allianceName;

    private final int pollenLaunchSpeed;

    private final int nectarLaunchSpeed;

    private final LaunchTarget autonEndTarget;     // Where autonomous parks (AUTON_END, AUTON_END_SF, ...)


    /**
     * Constructor for alliance configuration
     * @param isBlueAlliance true for Blue alliance, false for Red alliance
     * @param targetAprilTagIds AprilTag ID to track for relocalization
     * @param allianceName Name of the alliance (for display purposes)
     */
    private AllianceConfig(boolean isBlueAlliance, String targetAprilTagIds, String allianceName) {
        this.isBlueAlliance = isBlueAlliance;
        this.targetAprilTagIds = targetAprilTagIds;
        this.allianceName = allianceName;
        this.pollenLaunchSpeed = 0;
        this.nectarLaunchSpeed = 0;
        this.autonEndTarget = LaunchTarget.AUTON_END;
    }

    private AllianceConfig(boolean isBlueAlliance, String targetAprilTagIds, String allianceName,
                           int pollenLaunchSpeed, int nectarLaunchSpeed, LaunchTarget autonEndTarget) {
        this.isBlueAlliance = isBlueAlliance;
        this.targetAprilTagIds = targetAprilTagIds;
        this.allianceName = allianceName;
        this.pollenLaunchSpeed = pollenLaunchSpeed;
        this.nectarLaunchSpeed = nectarLaunchSpeed;
        this.autonEndTarget = autonEndTarget;
    }

    // Getters that use LaunchTarget enum
    public Pose getAllianceHomePose() {
        return LaunchTarget.ALLIANCE_HOME.getPose(isBlueAlliance);
    }

    public Pose getInitPose() {
        return LaunchTarget.INIT_POSE.getPose(isBlueAlliance);
    }

    public int getPollenLaunchSpeed(){
        return pollenLaunchSpeed;
    }

    public int getNectarLaunchSpeed() {
        return nectarLaunchSpeed;
    }


    public Pose getAutonEndPose() {
        return autonEndTarget.getPose(isBlueAlliance);
    }


    // New method to get pose for any target
    public Pose getTargetPose(LaunchTarget target) {
        return target.getPose(isBlueAlliance);
    }

    public String getTargetAprilTagIds() {
        return targetAprilTagIds;
    }

    public String getAllianceName() {
        return allianceName;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    /**
     * Creates a Blue Alliance configuration
     */
    public static AllianceConfig createBlueAllianceConfig() {
        return new AllianceConfig(
            true,   // isBlueAlliance
            RobotConstants.BLUE_GOAL_APRILTAGS_AF,     // AprilTag ID
            "Blue"  // Alliance name
        );
    }

    public static AllianceConfig createBlueAllianceConfigAF() {
        return new AllianceConfig(
                true,   // isBlueAlliance
                RobotConstants.BLUE_GOAL_APRILTAGS_AF,     // AprilTag ID
                "Blue",  // Alliance name
                1110,
                RobotConstants.CENTER_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END
        );
    }

    public static AllianceConfig createBlueAllianceConfigSF() {
        return new AllianceConfig(
                true,   // isBlueAlliance
                RobotConstants.BLUE_GOAL_APRILTAGS_SF,     // AprilTag ID
                "Blue",  // Alliance name
                RobotConstants.CENTER_POLLEN_LAUNCH_SPEED,
                RobotConstants.CENTER_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END_SF
        );
    }

    /**
     * Blue AF configuration for launching from the side (side launch speeds)
     */
    public static AllianceConfig createBlueAllianceConfigAFSideLaunch() {
        return new AllianceConfig(
                true,   // isBlueAlliance
                RobotConstants.BLUE_GOAL_APRILTAGS_AF,     // AprilTag ID
                "Blue",  // Alliance name
                RobotConstants.SIDE_POLLEN_LAUNCH_SPEED,
                RobotConstants.SIDE_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END
        );
    }

    /**
     * Creates a Red Alliance configuration
   */

    public static AllianceConfig createRedAllianceConfigAF() {
        return new AllianceConfig(
                false,  // isBlueAlliance
                RobotConstants.RED_GOAL_APRILTAGS_AF,     // AprilTag ID
                "Red" ,  // Alliance name
                RobotConstants.CENTER_POLLEN_LAUNCH_SPEED,
                RobotConstants.CENTER_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END
        );
    }
    public static AllianceConfig createRedAllianceConfigSF() {
        return new AllianceConfig(
                false,  // isBlueAlliance
                RobotConstants.RED_GOAL_APRILTAGS_SF,     // AprilTag ID
                "Red",   // Alliance name
                RobotConstants.CENTER_POLLEN_LAUNCH_SPEED,
                RobotConstants.CENTER_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END_SF
        );
    }

    /**
     * Red AF configuration for launching from the side (side launch speeds)
     */
    public static AllianceConfig createRedAllianceConfigAFSideLaunch() {
        return new AllianceConfig(
                false,  // isBlueAlliance
                RobotConstants.RED_GOAL_APRILTAGS_AF,     // AprilTag ID
                "Red",   // Alliance name
                RobotConstants.SIDE_POLLEN_LAUNCH_SPEED,
                RobotConstants.SIDE_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END
        );
    }

    /**
     * Red SF configuration for launching from the side (side launch speeds)
     */
    public static AllianceConfig createRedAllianceConfigSFSideLaunch() {
        return new AllianceConfig(
                false,  // isBlueAlliance
                RobotConstants.RED_GOAL_APRILTAGS_SF,     // AprilTag ID
                "Red",   // Alliance name
                RobotConstants.SIDE_POLLEN_LAUNCH_SPEED,
                RobotConstants.SIDE_NECTAR_LAUNCH_SPEED,
                LaunchTarget.AUTON_END_SF
        );
    }
}
