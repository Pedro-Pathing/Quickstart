package org.firstinspires.ftc.teamcode.pedroPathing.subSystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/** Handles stabilized AprilTag detection and path generation using a low-pass filter. */
public class AprilTagAligner {

    private final AprilTagProcessor aprilTag;
    private final Follower follower;
    private final ElapsedTime pathUpdateTimer = new ElapsedTime();

    private boolean isAligning = false;
    private boolean hasFilteredTag = false;
    private double filteredX, filteredY, filteredYaw;
    private double lastRawX, lastRawY, lastRawYaw;
    private int stableFrames = 0;
    private Pose lastTargetPose = null;

    public AprilTagAligner(AprilTagProcessor aprilTag, Follower follower) {
        this.aprilTag = aprilTag;
        this.follower = follower;
        pathUpdateTimer.reset();
    }

    /** Periodic update to check for tags and refresh the alignment path if needed. */
    public void update() {
        if (!isAligning) return;
        if (pathUpdateTimer.seconds() < ALIGN_UPDATE_SECONDS) return;

        AprilTagDetection targetTag = findTargetTag();
        if (targetTag != null) {
            processTagDetection(targetTag);
        }
    }

    /** Searches for the desired AprilTag ID in the current frame. */
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    /** Applies a low-pass filter and stability check to raw tag data. */
    private void processTagDetection(AprilTagDetection tag) {
        double rawX = tag.ftcPose.y;
        double rawY = -tag.ftcPose.x;
        double rawYaw = tag.ftcPose.yaw;

        if (!hasFilteredTag) {
            filteredX = rawX; filteredY = rawY; filteredYaw = rawYaw;
            hasFilteredTag = true;
            stableFrames = 1;
        } else {
            double xJump = Math.abs(rawX - lastRawX);
            double yJump = Math.abs(rawY - lastRawY);
            double yawJump = Math.abs(rawYaw - lastRawYaw);

            if (xJump > MAX_TRANSLATION_JUMP || yJump > MAX_TRANSLATION_JUMP || yawJump > MAX_YAW_JUMP_DEG) {
                // if (Math.abs(rawX - lastRawX) > MAX_TRANSLATION_JUMP || Math.abs(rawYaw - lastRawYaw) > MAX_YAW_JUMP_DEG) {
                stableFrames = 0;
            } else {
                filteredX = FILTER_ALPHA * rawX + (1.0 - FILTER_ALPHA) * filteredX;
                filteredY = FILTER_ALPHA * rawY + (1.0 - FILTER_ALPHA) * filteredY;
                filteredYaw = FILTER_ALPHA * rawYaw + (1.0 - FILTER_ALPHA) * filteredYaw;
                stableFrames++;
            }
        }

        lastRawX = rawX; lastRawY = rawY; lastRawYaw = rawYaw;

        if (stableFrames >= REQUIRED_STABLE_FRAMES) {
            generateAndFollowPath();
        }
    }

    /** Calculates a global field pose and commands the follower to drive to the standoff point. */
    private void generateAndFollowPath() {
        Pose currentPose = follower.getPose();
        double robotHeading = currentPose.getHeading();

        double tagLocalX = filteredX + CAMERA_FORWARD_OFFSET;
        double tagLocalY = filteredY + CAMERA_LEFT_OFFSET;

        double tagFieldX = currentPose.getX() + (tagLocalX * Math.cos(robotHeading)) - (tagLocalY * Math.sin(robotHeading));
        double tagFieldY = currentPose.getY() + (tagLocalX * Math.sin(robotHeading)) + (tagLocalY * Math.cos(robotHeading));

        double finalHeading = robotHeading + Math.toRadians(filteredYaw - CAMERA_YAW_OFFSET);

        double standoffX = tagFieldX - (DESIRED_DISTANCE * Math.cos(finalHeading));
        double standoffY = tagFieldY - (DESIRED_DISTANCE * Math.sin(finalHeading));

        double targetX = standoffX - (CAMERA_FORWARD_OFFSET * Math.cos(finalHeading)) + (CAMERA_LEFT_OFFSET * Math.sin(finalHeading));
        double targetY = standoffY - (CAMERA_FORWARD_OFFSET * Math.sin(finalHeading)) - (CAMERA_LEFT_OFFSET * Math.cos(finalHeading));

        Pose targetPose = new Pose(targetX, targetY, finalHeading);

        if (shouldUpdatePath(targetPose)) {
            Path path = new Path(new BezierLine(currentPose, targetPose));
            path.setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading());
            
            follower.setMaxPower(ALIGN_MAX_POWER);
            follower.followPath(path, true);
            lastTargetPose = targetPose;
        }
        pathUpdateTimer.reset();
    }

    /** Determines if the target has moved enough to warrant a new path calculation. */
    private boolean shouldUpdatePath(Pose newTarget) {
        if (lastTargetPose == null) return true;
        double dx = Math.abs(newTarget.getX() - lastTargetPose.getX());
        double dy = Math.abs(newTarget.getY() - lastTargetPose.getY());
        double dh = Math.abs(Math.atan2(Math.sin(newTarget.getHeading() - lastTargetPose.getHeading()), 
                                        Math.cos(newTarget.getHeading() - lastTargetPose.getHeading())));
        return dx > PATH_UPDATE_XY_INCHES || dy > PATH_UPDATE_XY_INCHES || dh > Math.toRadians(PATH_UPDATE_HEADING_DEG);
    }

    /** Activates the alignment state. */
    public void startAlignment() {
        isAligning = true;
        hasFilteredTag = false;
        stableFrames = 0;
        pathUpdateTimer.reset();
    }

    /** Cancels alignment and returns the follower to TeleOp drive mode. */
    public void stopAlignment() {
        if (isAligning) {
            follower.startTeleopDrive();
            follower.setMaxPower(1.0);
            isAligning = false;
            lastTargetPose = null;
            hasFilteredTag = false;
        }
    }

    /** Returns true if the robot is currently executing an alignment path. */
    public boolean isAligning() { return isAligning; }

    /** Returns the current target field pose for dashboard visualization. */
    public Pose getLastTargetPose() { return lastTargetPose; }
}
