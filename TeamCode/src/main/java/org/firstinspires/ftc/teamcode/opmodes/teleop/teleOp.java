package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp")
public class teleOp extends NextOpMode {
    public final Robot robot;
    protected HardwareMap hardwareMap;
    private final CommandGamepad driver1 = new CommandGamepad(gamepad1);
    private final CommandGamepad driver2 = new CommandGamepad(gamepad2);
    private final Follower follower;
    public teleOp(Robot robot) {
        super(robot);
        this.robot = robot;
        follower = Constants.create(hardwareMap);
    }


    @Override
    public void start() {
        robot.intake.start(driver2);
        robot.flywheel.start(driver2);
        robot.drivetrain.startDrive(gamepad1, follower);
    }

    @Override
    public void periodic() {

    }
}
