package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    public static double intakePower = 0;

    private MotorEx intake = new MotorEx("motorExp2");

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
        Logger.add("Intake", Logger.Level.DEBUG, "power: " + intakePower);
    }

    public static void setIntakePower(double newPower) {
        intakePower = newPower;
    }
}
