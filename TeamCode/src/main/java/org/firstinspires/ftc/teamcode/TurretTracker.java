package org.firstinspires.ftc.teamcode; // make sure this aligns with class location
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;


public class TurretTracker {
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;
    private static final int PIPELINE_ID = 2;
    private long lastLoopTime = 0;
    private Robot robot;

    private final int RED_GOAL_X = 144;
    private final int RED_GOAL_Y = 144;

    private final int BLUE_GOAL_X = 144;
    private final int BLUE_GOAL_Y = 144;
    // Track using coordinates
    private final Pose TARGET_COORDINATES = new Pose(RED_GOAL_X, RED_GOAL_Y); // e.g., (180, 84) for a field corner tag

    public TurretTracker(Robot robot) {
        // Constructor (can be left empty)
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        this.robot = robot;
    }

    public void start() {
        limelight.pipelineSwitch(PIPELINE_ID);
        limelight.start();
        lastLoopTime = System.nanoTime();
    }

    public void update(Pose currentRobotPose) {
            // Calculate loop time for derivative
            long currentTime = System.nanoTime();
            double dt = (currentTime - lastLoopTime) / 1e9; // seconds
            lastLoopTime = currentTime;
            dt = Math.max(dt, 0.001); // Prevent division by zero

            // -------------------- LIMELIGHT DATA --------------------
            LLResult result = limelight.getLatestResult();
            boolean trackingTag = false;
            double tx = 0.0;
            int detectedTagID = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                            tx = fiducial.getTargetXDegrees();
                            detectedTagID = fiducial.getFiducialId();
                            trackingTag = true;
                            break;
                        }
                    }
                }
            }

            // -------------------- TURRET CONTROL (Uses New Class) --------------------
            double turretPower = robot.turret.calculatePower(tx, trackingTag, currentRobotPose, dt);

            if (trackingTag || !robot.turret.isTrackingLost()) {
                robot.turret.setTurretPower(turretPower);

                // --- TELEMETRY ---
                String status = trackingTag ? "🎯 VISION LOCKED" : "🧠 ODOMETRY PREDICTING";
                robot.telemetry.addData("Turret Status", status);
                robot.telemetry.addData("Error Source", trackingTag ? "Vision" : "Prediction");
                robot.telemetry.addData("Error (Deg)", "%.2f°", robot.turret.getLastError());
                robot.telemetry.addData("Turret Power", "%.3f", turretPower);

            } else {
                // Target never seen or lost for too long (TurretTracker.isTrackingLost() is true)
                robot.turret.setTurretPower(0.0);
                robot.telemetry.addData("Turret Status", "🛑 LOST/IDLE");
            }
    }

    public void update_odometry_tracking(Pose currentRobotPose) {
        double turretPower;
        double errorToUse;
        double odometryError = 0.0;
        double deltaX = TARGET_COORDINATES.getX() - currentRobotPose.getX();
        double deltaY = TARGET_COORDINATES.getY() - currentRobotPose.getY();

        // Calculate the absolute angle to the target
        double absoluteTargetAngle = Math.atan2(deltaY, deltaX);

        // Calculate the required turret angle relative to the robot's heading
        // This is the error we need to drive to zero
        odometryError = Math.toDegrees(absoluteTargetAngle - currentRobotPose.getHeading());

        // Normalize the error to be between -180 and 180 degrees
        if (odometryError > 180) {
            odometryError -= 360;
        } else if (odometryError < -180) {
            odometryError += 360;
        }

        // Calculate loop time for derivative
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero

        // B. ODOMETRY MODE (New logic)
        // Run the PID calculation directly with the odometryError
        // You would need to expose a simple PID function from TurretTracker or calculate here.
        // Assuming a simplified PID function for this example:
        errorToUse = odometryError;
        turretPower = robot.turret.calculateSimplePIDPower(errorToUse, dt);

        // Set power and update telemetry
        robot.turret.setTurretPower(turretPower);
        robot.telemetry.addData("Turret Mode", "ODOMETRY");
        robot.telemetry.addData("Odometry Error", "%.2f°", odometryError);
    }

}
