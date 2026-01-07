package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    public static double outtakePosition = 0.75;
    private Servo outtakeServo;

    @Override
    public void initialize() {
        outtakeServo = ActiveOpMode.hardwareMap().servo.get("servoExp0");
    }

    @Override
    public void periodic() {
        outtakeServo.setPosition(outtakePosition);
        Logger.add("Transition", Logger.Level.DEBUG, "position: " + outtakeServo.getPosition());
    }

    public static void setOuttakePosition(double newPosition) {
        outtakePosition = newPosition;
    }
}
