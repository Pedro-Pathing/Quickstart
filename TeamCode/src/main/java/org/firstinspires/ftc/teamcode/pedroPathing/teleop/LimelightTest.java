package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "LimelightTest",group = "Test")
public class LimelightTest extends OpMode {
    private Limelight3A ll;
    private IMU imu;

    @Override
    public void init() {
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientation));
    }

    @Override
    public void start() {
        ll.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles ypra = imu.getRobotYawPitchRollAngles();
        ll.updateRobotOrientation(ypra.getYaw(AngleUnit.DEGREES));
        LLResult LLResult = ll.getLatestResult();
        if (LLResult.isValid() && LLResult != null) {
            Pose3D botpose = LLResult.getBotpose_MT2();
            telemetry.addData("X", botpose.getPosition().x);
            telemetry.addData("Y", botpose.getPosition().y);
            telemetry.addData("Heading", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
        }
    }
}