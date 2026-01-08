package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    private static double intakePower = 0;

    private MotorEx intake = new MotorEx("motorExp2");

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
        Logger.add("Intake", Logger.Level.DEBUG, "power: " + intakePower);
    }

    private static void setIntakePower(double newPower) {
        intakePower = newPower;
    }

    public static Command setIntakePowerCommand(double newPower) {
        return new InstantCommand(() -> setIntakePower(newPower));
    }
}
