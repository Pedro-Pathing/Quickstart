package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private static double intakePower = 0;
    private final MotorEx intake = new MotorEx("motor3");

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
        Logger.add("Intake", "power: " + intakePower);
    }

    private static void setIntakePower(double newPower) {
        intakePower = newPower;
    }

    public static Command setIntakePowerCommand(double newPower) {
        return new InstantCommand(() -> setIntakePower(newPower));
    }
}
