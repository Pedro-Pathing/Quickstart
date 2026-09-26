package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp")
public class teleOp extends NextOpMode {
    public final Robot robot;
    CommandGamepad driver1 = new CommandGamepad(gamepad1);
    CommandGamepad driver2 = new CommandGamepad(gamepad2);
    public teleOp(Robot robot) {
        super(robot);
        this.robot = robot;
    }


    @Override
    public void start() {
        robot.intake.start(driver2);
        robot.flywheel.start(driver2);
        robot.drivetrain.startDrive(gamepad1);
    }

    @Override
    public void periodic() {

    }
}
