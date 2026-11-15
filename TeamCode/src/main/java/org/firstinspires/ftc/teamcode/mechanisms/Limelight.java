package org.firstinspires.ftc.teamcode.mechanisms;
//coolefwvv
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Limelight")
public class Limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Pipeline 0 = aprilTag

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Status", "AprilTag Detected");
            telemetry.addData("Tx", "%.2f", llResult.getTx());
            telemetry.addData("Ty", "%.2f", llResult.getTy());
            telemetry.addData("Ta", "%.3f", llResult.getTa());

            Pose3D botpose = llResult.getBotpose_MT2();
            if (botpose != null) {
                double botx = botpose.getPosition().x;
                double boty = botpose.getPosition().y;

                telemetry.addData("botx", "%.2f", botx);
                telemetry.addData("boty", "%.2f", boty);
            }

        } else {
            telemetry.addData("Status", "No AprilTag detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}