package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    public double intakePower = 0;

    private MotorEx intake = new MotorEx("motorExp1")
            .brakeMode()
            .zeroed();

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
    }

    Command myLambdaCommand= new LambdaCommand()
            .setStart(() -> {
                // Runs on start
            })
            .setUpdate(() -> {
                // Runs on update
            })
            .setStop(interrupted -> {
                // Runs on stop
            })
            .setIsDone(() -> true) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("My Command"); // sets the name of the command; optional
}
