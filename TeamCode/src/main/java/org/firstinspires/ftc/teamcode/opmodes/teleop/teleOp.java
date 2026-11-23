package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


@Disabled
@TeleOp
public class teleOp extends OpMode {
    public Flywheel flywheel;
    public DcMotorEx intake;
    public Drivetrain drivetrain;


    @Override
    public void init() {
        Scheduler.reset();

        drivetrain = new Drivetrain(this);
        flywheel = new Flywheel(this);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    @Override
    public void start() {
        super.start();

        schedule(
                drivetrain.periodic(),
                flywheel.periodic(),
                intake()
        );
    }

    @Override
    public void loop() {
        if (gamepad2.dpadUpWasPressed()) {
            flywheel.adjustTarget(100);
        } if (gamepad2.dpadDownWasPressed()) {
            flywheel.adjustTarget(-100);
        } if (gamepad2.circleWasPressed()) {
            flywheel.toggle();
        } else if (gamepad2.dpadLeftWasPressed()) {
            flywheel.home();
        } if (gamepad2.dpadRightWasPressed()) {
            flywheel.reverse();
        }
        Scheduler.execute();
    }
    public Command intake() {
        return Command.build()
                .setExecute(()->{
                    intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    telemetry.addData("intake power", gamepad2.left_trigger - gamepad2.right_trigger);
                }).requiring(intake);
    }

}
