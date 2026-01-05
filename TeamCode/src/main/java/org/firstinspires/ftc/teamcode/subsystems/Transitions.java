package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    public static double outtakePosition = 0;
    private ServoEx outtakeServo = new ServoEx("servo1");

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        outtakeServo.to(outtakePosition);
        ActiveOpMode.telemetry().addLine("Outtake Transition Position: " + outtakeServo.getPosition());
    }
    public static void setOuttakePosition(double newPower) {
        outtakePosition = newPower;
    }
}
