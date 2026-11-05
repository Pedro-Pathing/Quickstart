package org.firstinspires.ftc.teamcode.hardwareControl.sensors.limeLight;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.limeLight.LimeLightConstants;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class LimeLightController {
    private boolean initialized = false;


    // Private static instance (eager initialization)
    private static final LimeLightController INSTANCE = new LimeLightController();
    private Telemetry telemetry;
    private final int APRILTAG_PIPELINE = 0;

    private Limelight3A limelight;


    // Private constructor to prevent instantiation
    private LimeLightController() {
        // Initialize hardware, state, or configuration here

    }

    // Public method to access the singleton instance
    public static LimeLightController getInstance() {
        return INSTANCE;
    }

    private static void setupConstants(){
        try {
            Class.forName(LimeLightConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //complains about unhandled exceptions if you take out try-catch
        }
    }
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) {
            return;
        }
        setupConstants();
        this.telemetry  = telemetry;
        initLimeLight01(hardwareMap);

        initialized = true;
    }

    public void reset() {

    }

    private void initLimeLight01(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    // Example method
    public Pose getCurrentRobotPose() {
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

//        limelight.updateRobotOrientation(pinpointYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D robotPose = result.getBotpose();

            double xPose = (robotPose.getPosition().x * 39.37008);  //convert meters to inches
            double yPose = (robotPose.getPosition().y * 39.37008);  //convert meters to inches
            double heading = robotPose.getOrientation().getYaw();

//            telemetry.addData("RobotPoseX", xPose);
//            telemetry.addData("RobotPoseY", yPose);
//            telemetry.addData("RobotHeading", heading);
            return new Pose(xPose, yPose, heading);
        }
        return null;
    }


    public Set<Integer> getPresentFiducialIds() {
        LLResult result = limelight.getLatestResult();

        if (!result.isValid()) {
            return Collections.emptySet();
        }

        Set<Integer> ids = new HashSet<>();
        for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
            ids.add(fr.getFiducialId());
        }
        return ids;
    }

    public void update(){

    }

    public void resumeStreaming(){
        limelight.start();
    }

    public void stopStreaming(){
        limelight.stop();
    }

}

/*usage Example*/

