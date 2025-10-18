package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.opmode.TeleOp_Solo.startingPose;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.ArrayList;

public class Robot {

    public TelemetryManager telemetry;
    public HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();

    public RobotData data = new RobotData();

    public Configuration names = new Configuration();

    public Follower follower;


    public CameraSubsystem cameraSubsystem;

    public IntakeSubsystem intakeSubsystem;

    public ShooterSubsystem shooterSubsystem;


    public ArrayList<RE_SubsystemBase> subsystems;
    public Configuration config = new Configuration();

    private static Robot instance = null;

    public void initialize(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.follower = PedroPathingConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetry.update();

        if (!Globals.IS_AUTO) follower.startTeleopDrive();

        subsystems = new ArrayList<>();

//        cameraSubsystem = new CameraSubsystem(this.hardwareMap, names.limelight);

        intakeSubsystem = new IntakeSubsystem(this.hardwareMap, names.intakeroller);

        shooterSubsystem = new ShooterSubsystem(this.hardwareMap, names.shootroller, names.stoproller);
    }

    public CameraSubsystem.Obelisk getObelisk() {
        return cameraSubsystem.getObelisk();
    }



    public void write() {
        this.data.write(telemetry);
    }

    public void periodic() {
        for (RE_SubsystemBase subsystem : subsystems) {
            subsystem.periodic();
        }

        this.follower.update();
    }

    public void updateData() {
        for (RE_SubsystemBase subsystem : subsystems) {
            subsystem.updateData();
        }
        this.data.currentPose = this.follower.getPose();
    }

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

}
