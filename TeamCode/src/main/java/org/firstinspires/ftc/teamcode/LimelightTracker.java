package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

/**
 * Handles communication with the Limelight 3A to acquire target data.
 */
public class LimelightTracker {
    private Limelight3A limelight;
    private final int targetTagId;
    private double currentTx = 0.0;
    private boolean isTargetFound = false;
    private int detectedTagID = -1;

    /**
     * @param hardwareMap The OpMode's hardware map.
     * @param llName The name of the Limelight in the configuration.
     * @param pipelineId The ID of the Limelight pipeline to use.
     * @param targetTagId The specific AprilTag ID to track (e.g., 24).
     */
    public LimelightTracker(HardwareMap hardwareMap, String llName, int pipelineId, int targetTagId) {
        this.targetTagId = targetTagId;
        limelight = hardwareMap.get(Limelight3A.class, llName);
        limelight.pipelineSwitch(pipelineId);
        limelight.start();
    }

    /**
     * Updates the latest Limelight data. Call this once per loop.
     */
    public void update() {
        LLResult result = limelight.getLatestResult();
        isTargetFound = false;
        currentTx = 0.0;
        detectedTagID = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // If the specific target is not found, default to the first tag found for general info
                detectedTagID = fiducials.get(0).getFiducialId();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        currentTx = fiducial.getTargetXDegrees();
                        isTargetFound = true;
                        detectedTagID = fiducial.getFiducialId();
                        break;
                    }
                }
            }
        }
    }

    public double getTx() {
        return currentTx;
    }

    public boolean isTargetFound() {
        return isTargetFound;
    }

    public int getDetectedTagID() {
        return detectedTagID;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
