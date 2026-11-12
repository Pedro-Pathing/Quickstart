package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Utility class for Limelight3A AprilTag detection.
 *
 * IMPORTANT: Call update() in your main loop to cache fresh data.
 * All getter methods return the last cached data.
 *
 * Design pattern based on official Limelight SDK.
 *
 * Cluster mode: FTC SDK 12.0 added AprilTag clusters (several tags on one game element,
 * e.g. the BioBuzz scoring goals, tags 30-45) to the VisionPortal AprilTagProcessor.
 * The Limelight does its own detection, so the SDK's cluster solver never sees its data.
 * Instead this class uses the SDK's AprilTagLibrary to know which tags belong to which
 * cluster, then combines every visible member into one target (averaged tx/ty/distance),
 * which stays locked on while some of the tags are blocked.
 *
 * The SDK's BioBuzz cluster IDs may not match the tags on your field. Use
 * setTargetCluster(name, "36,37,38,...") to define the cluster from your own tag IDs instead.
 */
public class LimelightUtil {

    // tag36h11 IDs run 0-586; used to count cluster members (the SDK keeps that list package-private)
    private static final int MAX_TAG_ID = 586;

    private Limelight3A limelight;
    private int targetTagId = -1;  // -1 = track any visible tag

    // Cluster tracking (mirrors AprilTagClusterDetection from the SDK)
    private final AprilTagLibrary tagLibrary;
    private final Map<String, Integer> clusterSizes = new HashMap<>();
    private boolean clusterMode = false;
    private String targetClusterName = null;  // null = any cluster
    private Set<Integer> customClusterTagIds = null;  // non-null = cluster defined by these tag IDs, not the library
    private String lastClusterName = null;
    private int lastClusterPercentFound = 0;
    private final List<Integer> lastClusterTagIds = new ArrayList<>();

    // Cached data from last update() call
    private double lastDistanceMeters = 0;
    private double lastHeightMeters = 0;
    private double lastTx = 0;
    private double lastTy = 0;
    private double lastTagYaw = 0;
    private int lastTrackedTagId = -1;
    private boolean hasValidTarget = false;

    private double targetArea = 0;

    private String botPose;

    /**
     * Constructor - initialize Limelight3A
     *
     * @param hardwareMap HardwareMap from OpMode
     */
    public LimelightUtil(HardwareMap hardwareMap) {
        this(hardwareMap, AprilTagGameDatabase.getCurrentGameTagLibrary());
    }

    /**
     * Constructor with a custom tag library (defines which tags form clusters).
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param tagLibrary  library whose clusters are used in cluster mode
     */
    public LimelightUtil(HardwareMap hardwareMap, AprilTagLibrary tagLibrary) {
        this.tagLibrary = tagLibrary;
        for (int id = 0; id <= MAX_TAG_ID; id++) {
            AprilTagClusterMetadata cluster = tagLibrary.lookupCluster(id);
            if (cluster != null) {
                Integer count = clusterSizes.get(cluster.name);
                clusterSizes.put(cluster.name, count == null ? 1 : count + 1);
            }
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);  // Default AprilTag pipeline
            limelight.start();
            System.out.println("Limelight3A initialized successfully");
        } catch (Exception e) {
            System.err.println("Error initializing Limelight3A: " + e.getMessage());
            limelight = null;
        }
    }

    public double getDistance() {
        double cameraAngle = 26.5; // Replace with your camera's mounting angle in degrees
        double cameraHeight = 10.00; // Replace with your camera's height from the floor
        double goalHeight = 53.50;   // Replace with the goal/target height from the floor

        double angleToTarget = cameraAngle + getTy();
        double heightDifference = goalHeight - cameraHeight;

        double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
        return distance;
    }

    public double getDistanceInMeters() {

        return getDistance() * 0.0254;
    }

    public double getDistanceFromTag() {
        double scale = 54070.98;
        double distance = (scale/targetArea);
        return distance;

    }


    /**
     * Set which AprilTag to track.
     *
     * @param tagId Specific tag ID to track, or -1 to track any visible tag
     */
    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }

    /**
     * MUST be called in your main loop to update cached vision data.
     * All getters return data cached from the last update() call.
     */
    public void update() {
        if (limelight == null) {
            hasValidTarget = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasValidTarget = false;
            return;
        }

        // Cache raw angle offsets
        lastTx = result.getTx();
        lastTy = result.getTy();
        targetArea = result.getTa();

        // Get list of detected AprilTags (fiducials)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        lastClusterName = null;
        lastClusterPercentFound = 0;
        lastClusterTagIds.clear();

        if (fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        // Cluster mode: lock onto a whole tag cluster; falls back to single tags if none is visible
        if (clusterMode && updateCluster(fiducials)) {
            return;
        }

        // Find target tag (specific ID or first available)
        LLResultTypes.FiducialResult targetFiducial = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            boolean allowed = targetTagId == -1
                    ? (customClusterTagIds == null || customClusterTagIds.contains(id))
                    : id == targetTagId;
            if (allowed) {
                targetFiducial = fiducial;
                break;
            }
        }

        if (targetFiducial == null) {
            hasValidTarget = false;
            return;
        }

        lastTrackedTagId = targetFiducial.getFiducialId();
        // Use this tag's angles, not the Limelight's primary target
        lastTx = targetFiducial.getTargetXDegrees();
        lastTy = targetFiducial.getTargetYDegrees();

        // Get robot position relative to AprilTag
        Pose3D robotPoseTargetSpace = targetFiducial.getRobotPoseTargetSpace();
        if (robotPoseTargetSpace != null) {
            Position pos = robotPoseTargetSpace.getPosition();

            // X and Z form horizontal plane, Y is vertical
            lastDistanceMeters = Math.sqrt(pos.x * pos.x + pos.z * pos.z);
            lastHeightMeters = pos.y;
            lastTagYaw = robotPoseTargetSpace.getOrientation().getYaw(AngleUnit.DEGREES);

            botPose = robotPoseTargetSpace.toString();

            hasValidTarget = true;
        } else {
            hasValidTarget = false;
        }
    }

    /**
     * Group visible fiducials by cluster, pick the best cluster, and cache its combined
     * pose as the current target.
     *
     * @return true if a cluster target was found and cached
     */
    private boolean updateCluster(List<LLResultTypes.FiducialResult> fiducials) {
        Map<String, List<LLResultTypes.FiducialResult>> visible = new HashMap<>();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            String clusterName;
            if (customClusterTagIds != null) {
                if (!customClusterTagIds.contains(fiducial.getFiducialId())) {
                    continue;
                }
                clusterName = targetClusterName;
            } else {
                AprilTagClusterMetadata cluster = tagLibrary.lookupCluster(fiducial.getFiducialId());
                if (cluster == null) {
                    continue;
                }
                if (targetClusterName != null && !targetClusterName.equals(cluster.name)) {
                    continue;
                }
                clusterName = cluster.name;
            }
            List<LLResultTypes.FiducialResult> members = visible.get(clusterName);
            if (members == null) {
                members = new ArrayList<>();
                visible.put(clusterName, members);
            }
            members.add(fiducial);
        }

        // Best cluster = most tags seen, then largest total area (closest)
        String bestCluster = null;
        List<LLResultTypes.FiducialResult> bestMembers = null;
        double bestArea = 0;
        for (Map.Entry<String, List<LLResultTypes.FiducialResult>> entry : visible.entrySet()) {
            double area = 0;
            for (LLResultTypes.FiducialResult f : entry.getValue()) {
                area += f.getTargetArea();
            }
            int count = entry.getValue().size();
            if (bestMembers == null || count > bestMembers.size()
                    || (count == bestMembers.size() && area > bestArea)) {
                bestCluster = entry.getKey();
                bestMembers = entry.getValue();
                bestArea = area;
            }
        }
        if (bestCluster == null) {
            return false;
        }

        // Average every visible member. Aim lands on the cluster center when all tags are seen;
        // with only some visible it is biased toward the visible ones.
        double sumTx = 0, sumTy = 0, sumDistance = 0, sumHeight = 0, sumYaw = 0;
        int poseCount = 0;
        LLResultTypes.FiducialResult closest = bestMembers.get(0);
        for (LLResultTypes.FiducialResult f : bestMembers) {
            sumTx += f.getTargetXDegrees();
            sumTy += f.getTargetYDegrees();
            lastClusterTagIds.add(f.getFiducialId());
            if (f.getTargetArea() > closest.getTargetArea()) {
                closest = f;
            }
            Pose3D pose = f.getRobotPoseTargetSpace();
            if (pose != null) {
                Position pos = pose.getPosition();
                sumDistance += Math.sqrt(pos.x * pos.x + pos.z * pos.z);
                sumHeight += pos.y;
                sumYaw += pose.getOrientation().getYaw(AngleUnit.DEGREES);
                poseCount++;
            }
        }
        if (poseCount == 0) {
            lastClusterTagIds.clear();
            return false;
        }

        int n = bestMembers.size();
        lastTx = sumTx / n;
        lastTy = sumTy / n;
        targetArea = bestArea;
        lastDistanceMeters = sumDistance / poseCount;
        lastHeightMeters = sumHeight / poseCount;
        lastTagYaw = sumYaw / poseCount;
        lastTrackedTagId = closest.getFiducialId();
        botPose = closest.getRobotPoseTargetSpace() != null ? closest.getRobotPoseTargetSpace().toString() : null;

        lastClusterName = bestCluster;
        Integer size = customClusterTagIds != null ? customClusterTagIds.size() : clusterSizes.get(bestCluster);
        lastClusterPercentFound = (size == null || size == 0) ? 0 : Math.round(100f * n / size);
        hasValidTarget = true;
        return true;
    }

    /**
     * Turn cluster mode on/off. When on, update() prefers a tag cluster from the tag
     * library over a single tag, and falls back to single-tag tracking if no cluster is seen.
     */
    public void setClusterMode(boolean enabled) {
        this.clusterMode = enabled;
    }

    public boolean isClusterMode() {
        return clusterMode;
    }

    /**
     * Only track the cluster with this name (e.g. "RED SCORING", "BLUE SCORING").
     *
     * @param name cluster name from the tag library, or null to track any cluster
     */
    public void setTargetClusterName(String name) {
        this.targetClusterName = name;
        this.customClusterTagIds = null;
    }

    /**
     * @return name of the cluster being targeted, or null if any cluster is accepted
     */
    public String getTargetClusterName() {
        return targetClusterName;
    }

    /**
     * Track a cluster made of the given tag IDs, ignoring the SDK tag library's clusters.
     * Also limits single-tag fallback to these IDs, so the robot never locks onto another goal.
     *
     * @param name   display name for telemetry (e.g. "Red Goal")
     * @param tagIds comma-separated tag IDs, e.g. RobotConstants.RED_GOAL_APRILTAG_IDS
     */
    public void setTargetCluster(String name, String tagIds) {
        Set<Integer> ids = new HashSet<>();
        for (String part : tagIds.split(",")) {
            String trimmed = part.trim();
            if (!trimmed.isEmpty()) {
                ids.add(Integer.parseInt(trimmed));
            }
        }
        this.targetClusterName = name;
        this.customClusterTagIds = ids.isEmpty() ? null : ids;
    }

    /**
     * Names of all clusters in the tag library, e.g. for cycling through targets.
     */
    public List<String> getClusterNames() {
        List<String> names = new ArrayList<>();
        for (AprilTagClusterMetadata cluster : tagLibrary.getAllClusters()) {
            names.add(cluster.name);
        }
        return names;
    }

    /**
     * @return true if the current target is a tag cluster (not a single tag)
     */
    public boolean isTrackingCluster() {
        return hasValidTarget && lastClusterName != null;
    }

    /**
     * @return name of the tracked cluster, or null if not tracking a cluster
     */
    public String getTrackedClusterName() {
        return isTrackingCluster() ? lastClusterName : null;
    }

    /**
     * Same meaning as AprilTagClusterDetection.percentClusterFound.
     *
     * @return percentage (0-100) of the cluster's tags currently visible
     */
    public int getClusterPercentFound() {
        return isTrackingCluster() ? lastClusterPercentFound : 0;
    }

    /**
     * @return IDs of the cluster's tags seen in the last update()
     */
    public List<Integer> getClusterTagIds() {
        return new ArrayList<>(lastClusterTagIds);
    }

    public double getTargetArea(){
        return targetArea;
    }

    public String getBotPose(){
        return botPose;
    }

    /**
     * Get the robot's yaw relative to the AprilTag (from the tag's 3D pose).
     * Unlike tx, this changes when the robot is angled to the tag's surface even if
     * the tag is centered in the image. Check on the robot what it reads when facing
     * the tag squarely (about 0 or about 180) and which way it moves when turning left.
     *
     * @return yaw in degrees, from the last update() with a valid target
     */
    public double getTagYaw() {
        return lastTagYaw;
    }

    /**
     * Check if we currently have a valid AprilTag lock.
     *
     * @return true if a valid tag is being tracked
     */
    public boolean hasTarget() {
        return hasValidTarget;
    }

    /**
     * Get the ID of the currently tracked AprilTag.
     *
     * @return tag ID, or -1 if no valid target
     */
    public int getTrackedTagId() {
        return hasValidTarget ? lastTrackedTagId : -1;
    }

    /**
     * Get horizontal distance to target in meters.
     * This is the distance in the horizontal plane (X-Z).
     *
     * @return distance in meters, or 0 if no valid target
     */
    public double getDistanceToTargetMeters() {
        return lastDistanceMeters;
    }

    /**
     * Get vertical distance to target in meters.
     * Positive = target is above camera
     * Negative = target is below camera
     *
     * @return height difference in meters, or 0 if no valid target
     */
    public double getHeightToTargetMeters() {
        return lastHeightMeters;
    }

    /**
     * Convert distance from meters to inches.
     *
     * @param meters distance in meters
     * @return distance in inches
     */
    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    /**
     * Get horizontal distance in inches.
     *
     * @return distance in inches, or 0 if no valid target
     */
    public double getDistanceToTargetInches() {
        return metersToInches(lastDistanceMeters);
    }

    /**
     * Get vertical distance in inches.
     *
     * @return height in inches, or 0 if no valid target
     */
    public double getHeightToTargetInches() {
        return metersToInches(lastHeightMeters);
    }

    /**
     * Get horizontal angle offset (for aiming).
     * Positive = target to the right
     * Negative = target to the left
     *
     * @return angle in degrees (-29.8 to 29.8)
     */
    public double getTx() {
        return lastTx;
    }

    /**
     * Get vertical angle offset.
     * Positive = target above crosshair
     * Negative = target below crosshair
     *
     * @return angle in degrees (-24.85 to 24.85)
     */
    public double getTy() {
        return lastTy;
    }

    /**
     * Check if target is horizontally centered (within tolerance).
     *
     * @param tolerance tolerance in degrees
     * @return true if aligned
     */
    public boolean isHorizontallyAligned(double tolerance) {
        return Math.abs(lastTx) <= tolerance;
    }

    /**
     * Check if target is vertically centered (within tolerance).
     *
     * @param tolerance tolerance in degrees
     * @return true if aligned
     */
    public boolean isVerticallyAligned(double tolerance) {
        return Math.abs(lastTy) <= tolerance;
    }

    /**
     * Switch Limelight pipeline.
     *
     * @param pipeline pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Check if Limelight hardware is connected.
     *
     * @return true if Limelight is available and connected
     */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /**
     * Get a formatted status string for telemetry display.
     * One-liner with all important info.
     *
     * @return formatted status string
     */
    public String getStatusString() {
        if (limelight == null) {
            return "Limelight: Not connected";
        }
        if (!hasValidTarget) {
            return "Limelight: No target";
        }
        if (isTrackingCluster()) {
            return String.format(Locale.US, "Cluster %s (%d%%) | Distance: %.2fm | Height: %.2fm | tx: %.1f°",
                    lastClusterName, lastClusterPercentFound, lastDistanceMeters, lastHeightMeters, lastTx);
        }
        return String.format(Locale.US, "Tag %d | Distance: %.2fm | Height: %.2fm | tx: %.1f°",
                lastTrackedTagId, lastDistanceMeters, lastHeightMeters, lastTx);
    }

    /**
     * Get the underlying Limelight3A object for direct access if needed.
     *
     * @return Limelight3A instance
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Stop Limelight processing.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}