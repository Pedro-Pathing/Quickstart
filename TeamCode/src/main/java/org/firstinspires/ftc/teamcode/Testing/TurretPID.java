package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(group = "testing", name="TurretPidTuning")
public class TurretPID extends OpMode {
    public static double P=0.01,I,D=0.0005,F = 0,Pos=100;
    static TelemetryManager telemetryM;
    private Follower follower;
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry dashboardT = dashboard.getTelemetry();
    public static Pose startingPose = new Pose(72, 72, 0);
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        Robot.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Turret.faceGoal(follower.getPose().getX(), follower.getPose().getX(), follower.getHeading(), true, 0);
//        telemetryM.addData("targetPosition", Pos);
//        telemetryM.addData("currentPosition", turret.getCurrentPosition());
//        telemetryM.addData("error", Pos-turret.getCurrentPosition());
//        telemetryM.update(telemetry);
        dashboardT.addData("targetPosition", Pos);
        dashboardT.addData("currentPosition", turret.getCurrentPosition());
        dashboardT.addData("error", Pos-turret.getCurrentPosition());
        dashboardT.update();
    }
}
