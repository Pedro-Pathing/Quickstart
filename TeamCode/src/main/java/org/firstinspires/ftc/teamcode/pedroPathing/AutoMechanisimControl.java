package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

@Autonomous(name = "Attempt to use mechanisms while driving", group = "Auto")
public class AutoMechanisimControl extends LinearOpMode {
    Follower follower;


    //Mechanisms
    DcMotorEx shooter;
    DcMotor intake;
    DcMotor feeder;
    //Pose path targets
    ArrayList<Pose> poseTargets = new ArrayList<>();
    int currentState = 0;

    //Constants. We will test to Find the correct ones
    static final double SHOOTER_RPM = 4200;
    static final double RPM_Tolerance = 75;
    static final double Feeder_Power  = 0.6;
    static final double Intake_Power  = 0.2;

    static final int TOTAL_BALLS = 3;

    int ballsShot = 0;
    boolean feedingBall = false;

    ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void runOpMode() {


        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Pose startPose = new Pose (0,0, Math.toRadians(90));
    }

}
