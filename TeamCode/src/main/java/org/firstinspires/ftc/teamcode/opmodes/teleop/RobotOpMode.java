package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@Disabled
@TeleOp
public class RobotOpMode extends OpMode {

    protected Robot robot;
    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);

        schedule(
                robot.drivetrain.periodic(),
                robot.flywheel.periodic(),
                robot.intake.periodic()
        );
    }
    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

    }
}
