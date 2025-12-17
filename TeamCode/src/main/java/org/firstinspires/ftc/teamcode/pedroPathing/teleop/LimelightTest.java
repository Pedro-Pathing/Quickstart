package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "LimelightTest",group = "Test")
public class LimelightTest extends OpMode {
    private PanelsField panelsField;

    private PanelsTelemetry panelsTelemetry;
    private Limelight3A ll;
    private IMU imu;

    private Odometry odo;

    @Override
    public void init() {
        panelsField.setField(Field.PEDRO_PATHING);
        panelsField = new PanelsField();
        panelsTelemetry = new PanelsTelemetry(panelsField);
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(1);
        ///odo = hardwareMap.get(Odometry.class, "odo");
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
        ///Pose2d initPose = odo.getPose();
        ///double heading = Math.toDegrees(odo.getPose().getHeading());
        ///ll.updateRobotOrientation(heading);
        YawPitchRollAngles ypra = imu.getRobotYawPitchRollAngles();
        ll.updateRobotOrientation(ypra.getYaw(AngleUnit.DEGREES));
        LLResult LLResult = ll.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose_MT2();
            telemetry.addData("X", botpose.getPosition().x);
            telemetry.addData("Y", botpose.getPosition().y);
            telemetry.addData("Heading", botpose.getOrientation().getYaw(AngleUnit.DEGREES));

            double dx = botpose.getPosition().x - odo.getPose().getX();
            double dy = botpose.getPosition().y - odo.getPose().getY();
            double dist = Math.hypot(dx,dy);
        }
        telemetry.update();
    }
}