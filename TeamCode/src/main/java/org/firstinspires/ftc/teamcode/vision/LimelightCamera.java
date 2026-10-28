package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.helpers.Devices;

import static org.firstinspires.ftc.teamcode.vision.LimelightConstants.*;

/**
 * Finds the centre of our HIVE's target CELL using the AprilTags under its CELLs.
 *
 * Only our alliance's HIVE tags are used. If tags from both of its CELLs are in view, the largest
 * tag decides which CELL is the target, and every visible tag of that CELL is averaged.
 *
 * Per tag:
 * 1. The tag's position in camera space is moved into the robot frame using the camera mount
 *    constants, giving where the tag is.
 * 2. The camera's position in the tag's own frame shows which side of the tag the camera is on, and
 *    from that, which way the tag faces in the robot frame. The tag's angles are not used, because
 *    the SDK labels Limelight's camera-space angles inconsistently. The tags' downward tilt comes
 *    from tagFacingDownDegrees.
 * 3. tagToTargetOffset then moves from the tag to the centre of the CELL's open face.
 *
 * If the Limelight isn't in the configuration (or won't start), vision is disabled and findTarget()
 * always returns null, as if no tags were in view.
 */
public class LimelightCamera {
    private final Limelight3A limelight;  // null when vision is disabled
    private Alliance alliance;

    public LimelightCamera(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        limelight = Devices.get(hardwareMap, Limelight3A.class, deviceName, l -> {
            l.setPollRateHz(pollRateHz);
            l.pipelineSwitch(aprilTagPipeline);
            l.start();
        });
    }

    /** Whether the Limelight was found and started. */
    public boolean isAvailable() {
        return limelight != null;
    }

    /** The Limelight's name in quotes if it's missing, otherwise "". */
    public String missingDevices() {
        return Devices.missing(limelight, deviceName);
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    /**
     * The current target, or null if none of our HIVE's tags are in view or the latest result is
     * too old (or vision is disabled). Call once per loop and pair it with the robot heading from the same loop.
     */
    public HiveTarget findTarget() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > maxStalenessMs) return null;

        // The largest of our alliance's tags decides which CELL is the target
        LLResultTypes.FiducialResult largest = null;
        HiveTarget.Cell cell = null;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            HiveTarget.Cell tagCell = cellOf(tag.getFiducialId());
            if (tagCell == null || !isRaised(tag)) continue;
            if (largest == null || tag.getTargetArea() > largest.getTargetArea()) {
                largest = tag;
                cell = tagCell;
            }
        }
        if (largest == null) return null;

        // Average the target estimate from every visible tag of that CELL, weighted by tag area
        double sumX = 0, sumY = 0, sumZ = 0, sumWeight = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (cellOf(tag.getFiducialId()) != cell || !isRaised(tag)) continue;
            double[] estimate = targetFromTag(tag, tag.getFiducialId() - firstTag(cell));
            if (estimate == null) continue;
            double weight = tag.getTargetArea();
            sumX += estimate[0] * weight;
            sumY += estimate[1] * weight;
            sumZ += estimate[2] * weight;
            sumWeight += weight;
            count++;
        }
        if (count == 0 || sumWeight <= 0) return null;

        return new HiveTarget(cell, sumX / sumWeight, sumY / sumWeight, sumZ / sumWeight,
                count, largest.getFiducialId(), result.getStaleness());
    }

    /** Estimate of the open face's centre in the robot frame from one tag, or null if it has no 3D pose. */
    private double[] targetFromTag(LLResultTypes.FiducialResult tag, int index) {
        Position tagInCamera = metres(tag.getTargetPoseCameraSpace());
        Position cameraInTag = metres(tag.getCameraPoseTargetSpace());
        if (tagInCamera == null || cameraInTag == null || tagInCamera.z <= 0) return null;

        // 1. Tag position in the robot frame
        double[] tagInRobot = cameraToRobot(tagInCamera.x, tagInCamera.y, tagInCamera.z);

        // 2. Which way the tag faces. Target space: +X right as seen facing the tag (horizontal),
        // +Y down the tag's face, +Z out of the tag. With the tag tilted to face down by tilt, the
        // horizontal distance of the camera in front of the tag is z*cos(tilt) - y*sin(tilt).
        double tilt = Math.toRadians(tagFacingDownDegrees);
        double inFront = cameraInTag.z * Math.cos(tilt) - cameraInTag.y * Math.sin(tilt);
        // The camera is at angle beta from the tag's horizontal facing direction, towards the tag's right
        double beta = Math.atan2(cameraInTag.x, inFront);
        double toCameraX = cameraForward - tagInRobot[0];
        double toCameraY = cameraLeft - tagInRobot[1];
        double length = Math.hypot(toCameraX, toCameraY);
        if (length < 1e-6) return null;
        toCameraX /= length;
        toCameraY /= length;
        // The tag's "out" direction is the tag-to-camera direction turned back by beta (clockwise)
        double outX = toCameraX * Math.cos(beta) + toCameraY * Math.sin(beta);
        double outY = -toCameraX * Math.sin(beta) + toCameraY * Math.cos(beta);
        // The tag's "right" as seen facing it is its "out" direction turned 90 degrees anticlockwise
        double rightX = -outY;
        double rightY = outX;

        // 3. Move from the tag to the centre of the open face
        double[] offset = tagToTargetOffset[index];
        return new double[] {
                tagInRobot[0] + offset[0] * rightX + offset[1] * outX,
                tagInRobot[1] + offset[0] * rightY + offset[1] * outY,
                tagInRobot[2] + offset[2]
        };
    }

    /** Is this tag under the raised CELL (upright as seen from the field)? */
    private boolean isRaised(LLResultTypes.FiducialResult tag) {
        Position p = metres(tag.getTargetPoseCameraSpace());
        return p != null && p.z > 0 && cameraToRobot(p.x, p.y, p.z)[2] > minTagHeight;
    }

    /**
     * Converts a point from camera space (+X right, +Y down, +Z out of the lens) to the robot frame,
     * using the camera mount position and orientation.
     */
    private static double[] cameraToRobot(double cx, double cy, double cz) {
        // Camera axes to robot axes for a level, forward-facing camera
        double x = cz, y = -cx, z = -cy;

        double roll = Math.toRadians(cameraRoll);
        double pitch = Math.toRadians(cameraPitch);
        double yaw = Math.toRadians(cameraYaw);

        // Roll about the forward axis
        double y1 = y * Math.cos(roll) - z * Math.sin(roll);
        double z1 = y * Math.sin(roll) + z * Math.cos(roll);
        // Pitch: positive tilts the forward axis upwards
        double x2 = x * Math.cos(pitch) - z1 * Math.sin(pitch);
        double z2 = x * Math.sin(pitch) + z1 * Math.cos(pitch);
        // Yaw: positive turns the forward axis to the left
        double x3 = x2 * Math.cos(yaw) - y1 * Math.sin(yaw);
        double y3 = x2 * Math.sin(yaw) + y1 * Math.cos(yaw);

        return new double[] {x3 + cameraForward, y3 + cameraLeft, z2 + cameraUp};
    }

    private static Position metres(Pose3D pose) {
        if (pose == null || pose.getPosition() == null) return null;
        return pose.getPosition().toUnit(DistanceUnit.METER);
    }

    /** Which of our HIVE's CELLs a tag is under, or null if it isn't one of ours. */
    private HiveTarget.Cell cellOf(int id) {
        int scoring = firstTag(HiveTarget.Cell.SCORING);
        int audience = firstTag(HiveTarget.Cell.AUDIENCE);
        if (id >= scoring && id < scoring + TAGS_PER_CELL) return HiveTarget.Cell.SCORING;
        if (id >= audience && id < audience + TAGS_PER_CELL) return HiveTarget.Cell.AUDIENCE;
        return null;
    }

    /** ID of the first of the four tags under one of our CELLs. */
    private int firstTag(HiveTarget.Cell cell) {
        if (alliance == Alliance.RED) {
            return cell == HiveTarget.Cell.SCORING ? RED_SCORING_FIRST_TAG : RED_AUDIENCE_FIRST_TAG;
        }
        return cell == HiveTarget.Cell.SCORING ? BLUE_SCORING_FIRST_TAG : BLUE_AUDIENCE_FIRST_TAG;
    }
}
