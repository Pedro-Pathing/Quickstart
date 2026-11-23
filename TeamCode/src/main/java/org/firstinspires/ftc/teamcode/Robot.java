package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


public class Robot {
    public Flywheel flywheel;
    public Intake intake;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public Drivetrain drivetrain;

    public Follower follower;
    public Gamepad gamepad2;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad2 = opMode.gamepad2;

        flywheel = new Flywheel(this);
        intake = new Intake(this);
        drivetrain = new Drivetrain(this);
    }

}
