package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    public static double transitionPower = 0;

    private CRServoEx intakeServo = new CRServoEx("servo5");

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intakeServo.setPower(transitionPower);
    }
    public static void setTransitionPower(double newPower) {
        new InstantCommand(() -> transitionPower = newPower).schedule();
    }
}
