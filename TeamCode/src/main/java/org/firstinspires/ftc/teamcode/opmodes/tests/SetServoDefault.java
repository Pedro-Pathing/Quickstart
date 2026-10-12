package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SETSERVO", group = "Test")
public class SetServoDefault extends OpMode {

    private Servo turretL;
    private Servo turretR;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private double rightOffset = -0.008;
    private double center = 0.5;

    @Override
    public void init() {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);

        if (current.a && !previous.a){
            turretL.setPosition(center);
            turretR.setPosition(center + rightOffset);
        }
    }
}
