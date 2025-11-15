package org.firstinspires.ftc.teamcode.experimental;

import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

/**
 * Handles communication with the Limelight 3A to acquire target data.
 */
public class LimelightTracker {
    private final Limelight3A limelight;
    private double currentTx = 0.0;
    private boolean isTargetFound = false;
    private int detectedTagID = -1;

    /**
     */
    public LimelightTracker(Robot robot) {
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Robot.current_pipeline_id);
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
                    if (fiducial.getFiducialId() == Robot.current_tag_id) {
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