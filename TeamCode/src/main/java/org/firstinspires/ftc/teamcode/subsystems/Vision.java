package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.io.OutputStream;
import java.net.Socket;
import java.util.List;

public class Vision implements Subsystem {

    //-------------Hardware------------
    private Limelight3A limelight;
    //HUSKY

    //-------------Software-------------
    private LLResult latest;
    private int pipeline = 0;

    private double lastFieldX = 0;
    private double lastFieldY = 0;
    private double lastHeading = 0;
    private long lastTime = 0;
    private double lastPredictedX = 0;
    private double lastPredictedY = 0;

    private final double LOOK_AHEAD_MS = 50; // Predict x ms into the future

    private final double CAMERA_HEIGHT_INCHES = 0;
    private final double TARGET_HEIGHT_INCHES = 0; // Estimated height of the pollen object
    private final double CAMERA_PITCH_DEGREES = 0; // Camera angled downwards x degrees
    private final double CAMERA_FORWARD_OFFSET = 0; // Distance from robot center to camera lens

    private double smoothedVelocityX = 0;
    private double smoothedVelocityY = 0;
    private final double VELOCITY_SMOOTHING_FACTOR = 0.3; // Lower = smoother but more lag (0.0 to 1.0)
    private long lastCaptureTime = 0;

    //------------Pictures--------------
    private int snapshotCount = 0;
    private static final String LAPTOP_IP = "192.168.43.9";
    private static final int PORT = 6000;

    private Socket socket;
    private OutputStream out;
    private String connectionStatus = "Disconnected"; //Status tracker

    //--------Constructor--------
    private Vision(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");
    }
    private static Vision instance = null;

    public static Vision getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Vision(hardwareMap);
        }
        return instance;
    }

    //---------Methods-----------
    private void limelightInit() {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    private void limelightUpdate() {
        latest = limelight.getLatestResult();
    }

    private void logFeedToLogger() {
        //This will be used for PsiKit later
    }

    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    public double getTx() {
        return (hasTarget() ? latest.getTxNC() : 0);
    }

    public double getTy() {
        return (hasTarget() ? latest.getTyNC() : 0);
    }

    public double getArea() {
        return (hasTarget() ? latest.getTa() : 0);
    }

    public Pose getPredictiveInterceptPose(Pose robotPose) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            resetTrackingHistory();
            return null;
        }
        long currentCaptureTime = (long) result.getTimestamp(); // Or derive from latency if timestamp isn't exposed
        if (currentCaptureTime == lastCaptureTime) {
            return new Pose(lastPredictedX, lastPredictedY, lastHeading);
        }

        List<LLResultTypes.DetectorResult> targets = result.getDetectorResults();
        LLResultTypes.DetectorResult chosenPollen = findBestPollenCluster(targets);

        if (chosenPollen == null) {
            resetTrackingHistory();
            return null;
        }

        // 1. Calculate relative coordinates in inches based on Limelight angles
        double currentXAngle = chosenPollen.getTargetXDegrees();
        double currentYAngle = chosenPollen.getTargetYDegrees();

        double totalPitchRadians = Math.toRadians(CAMERA_PITCH_DEGREES + currentYAngle);

        // Prevent division by zero if target is perfectly on the horizon
        if (Math.abs(totalPitchRadians) < 0.001) totalPitchRadians = 0.001;

        double forwardDistance = (CAMERA_HEIGHT_INCHES - TARGET_HEIGHT_INCHES) / Math.tan(totalPitchRadians);
        double lateralDistance = forwardDistance * Math.tan(Math.toRadians(currentXAngle));

        double robotRelativeForward = forwardDistance + CAMERA_FORWARD_OFFSET;
        double robotRelativeLateral = lateralDistance;

        // Extract robot positions and heading (Pedro Pathing heading is in RADIANS natively)
        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double robotHeadingRad = robotPose.heading();

        // 2. Vector Translation: Convert Robot Space to absolute Field Space coordinates
        // Forward projects along cosine of heading, lateral projects along sine (standard 2D rotation)
        double currentFieldX = robotX + (robotRelativeForward * Math.cos(robotHeadingRad) - robotRelativeLateral * Math.sin(robotHeadingRad));
        double currentFieldY = robotY + (robotRelativeForward * Math.sin(robotHeadingRad) + robotRelativeLateral * Math.cos(robotHeadingRad));

        // 3. Velocity Prediction Math
        long currentTime = System.currentTimeMillis();

        double predictedFieldX = currentFieldX;
        double predictedFieldY = currentFieldY;


        if (lastTime != 0) {
            long deltaTimeMs = currentTime - lastTime;
            if (deltaTimeMs > 0) {
                // Calculate raw instantaneous velocity
                double rawVelocityX = (currentFieldX - lastFieldX) / (double) deltaTimeMs;
                double rawVelocityY = (currentFieldY - lastFieldY) / (double) deltaTimeMs;

                // Apply Exponential Moving Average (EMA) to filter out camera jitter
                smoothedVelocityX = (rawVelocityX * VELOCITY_SMOOTHING_FACTOR) + (smoothedVelocityX * (1 - VELOCITY_SMOOTHING_FACTOR));
                smoothedVelocityY = (rawVelocityY * VELOCITY_SMOOTHING_FACTOR) + (smoothedVelocityY * (1 - VELOCITY_SMOOTHING_FACTOR));

                predictedFieldX = currentFieldX + (smoothedVelocityX * LOOK_AHEAD_MS);
                predictedFieldY = currentFieldY + (smoothedVelocityY * LOOK_AHEAD_MS);
            }
        }

        //Find optimal heading
        double deltaX = predictedFieldX - robotX;
        double deltaY = predictedFieldY - robotY;
        double targetHeadingRad = Math.atan2(deltaY, deltaX);

        // Save current values to track movement velocity on the next frame iteration
        lastFieldX = currentFieldX;
        lastFieldY = currentFieldY;
        lastTime = currentTime;
        lastCaptureTime = currentCaptureTime;
        lastHeading = targetHeadingRad;

        lastPredictedX = predictedFieldX;
        lastPredictedY = predictedFieldY;
        // Return a fresh Pedro Pathing Pose.
        // Note: Target heading can remain 0 or be set to match the robot's heading preference.
        return new Pose(predictedFieldX, predictedFieldY, targetHeadingRad);
    }

    private LLResultTypes.DetectorResult findBestPollenCluster(List<LLResultTypes.DetectorResult> objects) {
        LLResultTypes.DetectorResult bestPollen = null;
        double highestScore = Double.NEGATIVE_INFINITY;

        final double NECTAR_PROXIMITY_THRESHOLD_PIXELS = 120;
        final double POLLEN_AREA_MULTIPLIER = 10.0;
        final double NECTAR_PENALTY_WEIGHT = 25.0;

        for (LLResultTypes.DetectorResult obj : objects) {
            if (obj.getClassName().equals("pollen")) {
                double score = obj.getTargetArea() * POLLEN_AREA_MULTIPLIER;

                double pollenPixelX = obj.getTargetXPixels();
                double pollenPixelY = obj.getTargetYPixels();

                // 2. Scan the rest of the visible list to find nearby nectar threats
                for (LLResultTypes.DetectorResult checkObj : objects) {
                    if (checkObj.getClassName().equals("nectar")) {

                        double nectarPixelX = checkObj.getTargetXPixels();
                        double nectarPixelY = checkObj.getTargetYPixels();

                        // Calculate straight-line pixel distance (Euclidean Distance Formula)
                        double deltaX = pollenPixelX - nectarPixelX;
                        double deltaY = pollenPixelY - nectarPixelY;
                        double pixelDistance = Math.hypot(deltaX, deltaY);

                        // 3. Apply penalty if nectar is contaminating this specific pollen cluster
                        if (pixelDistance <= NECTAR_PROXIMITY_THRESHOLD_PIXELS) {
                            // The closer or larger the nectar is, the higher the penalty
                            double nectarThreatSize = checkObj.getTargetArea();
                            score -= (nectarThreatSize * NECTAR_PENALTY_WEIGHT);
                        }
                    }
                }

                // 4. Track the single highest scoring (cleanest/largest) target
                if (score > highestScore) {
                    highestScore = score;
                    bestPollen = obj;
                }
            }
        }

        return bestPollen;
    }

    private void resetTrackingHistory() {
        lastFieldX = 0;
        lastFieldY = 0;
        lastTime = 0;
        lastCaptureTime = 0;
        smoothedVelocityX = 0;
        smoothedVelocityY = 0;
        lastHeading = 0;
    }

    public int getCurrentTagId()
    {
        if (!hasTarget() || latest.getFiducialResults().isEmpty()) {
            return -1;
        }
        return latest.getFiducialResults().get(0).getFiducialId();
    }
    public Pose3D getLatestBotPose()
    {
        if (latest == null || !latest.isValid()) {
            return null;
        }
        try {
            Pose3D mt2 = latest.getBotpose_MT2();
            if (mt2 != null) {
                return mt2;
            }
        } catch (Throwable ignored) {
        }
        try {
            return latest.getBotpose();
        } catch (Throwable ignored) {
            return null;
        }
    }
    //-------------Snapshot Methods---------------
    public void startSnapshotServer() {
        connectionStatus = "Attempting to connect to " + LAPTOP_IP + "...";
        new Thread(() -> {
            try {
                socket = new Socket(LAPTOP_IP, PORT);
                out = socket.getOutputStream();
                connectionStatus = "CONNECTED to Laptop!";
            } catch (Exception e) {
                connectionStatus = "FAILED: " + e.getMessage();
            }
        }).start();
    }

    public void takeSnapshot() {
        snapshotCount++;
        String snapName = "dataset_snap_" + snapshotCount;
        limelight.captureSnapshot(snapName);

        new Thread(() -> {
            try {
                if (out != null) {
                    out.write(("SNAP:" + snapName + "\n").getBytes());
                    out.flush();
                }
            } catch (Exception e) {
                connectionStatus = "SEND ERROR: " + e.getMessage();
            }
        }).start();
    }

    public void stopSnapshotServer() {
        try {
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
        } catch (Exception ignored) {
            System.out.println("An error occurred...");
        }
    }

    public void stopLimelightAndServers() {
        limelight.stop();
        stopSnapshotServer();
    }

    // Getters for Telemetry
    public String getConnectionStatus() { return connectionStatus; }
    public int getSnapshotCount() { return snapshotCount; }

    //---------Interface Methods---------
    @Override
    public void init() {
        limelightInit();
        startSnapshotServer();
    }

    @Override
    public void update() {
        limelightUpdate();
    }

    @Override
    public void stop() {
        stopLimelightAndServers();
    }

}
