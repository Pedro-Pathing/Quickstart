package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Transitions implements Subsystem {

    public static final Transitions INSTANCE = new Transitions();
    public static double DOWN_POS = 0.95;
    public static double UP_POS = 0.55;
    private static double outtakePosition = DOWN_POS;
    private Servo outtakeServo;

    @Override
    public void initialize() {
        outtakeServo = ActiveOpMode.hardwareMap().servo.get("servo0");
        outtakeServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        outtakeServo.setPosition(outtakePosition);
        Logger.add("Transition", Logger.Level.DEBUG, "position: " + outtakeServo.getPosition());
    }

    private static void setOuttakePosition(double newPosition) {
        outtakePosition = newPosition;
    }

    public static Command setOuttakePositionCommand(double newPosition) {
        return new InstantCommand(() -> setOuttakePosition(newPosition));
    }
}