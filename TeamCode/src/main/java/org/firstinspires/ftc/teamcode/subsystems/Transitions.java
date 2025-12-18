package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    public static double intakePower = 0;
    public static double outtakePower = 0;

    private CRServoEx intakeServo = new CRServoEx("servo5");
    private CRServoEx outtakeServo = new CRServoEx("servo4");


    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intakeServo.setPower(intakePower);
        outtakeServo.setPower(outtakePower);
    }
    public static void setIntakePower(double newPower) {
        intakePower = newPower;
    }
    public static void setOuttakePower(double newPower) {
        intakePower = newPower;
    }
}
