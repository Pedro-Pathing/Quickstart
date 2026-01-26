package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTagLimelightTest extends OpMode{

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0); // april tag #11 pipeline
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();

    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null & llresult.isValid()){
            Pose3D botPose = llresult.getBotpose();
            //telemetry.addData('Tx', llresult.getTx());
            //telemetry.addData('Tx', llresult.getTy());
            //telemetry.addData('Ta', llresult.getTa());
            //telemetry.addData("BotPose", botPose.toString());
            //telemetry.addData("Yaw", botPose.getOrientation().getYaw());

        }


    }
}
