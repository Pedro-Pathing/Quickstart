package org.firstinspires.ftc.teamcode.friends.teleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    Servo servo;
    int servoPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "Servo");
        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(servoPosition);
            servo.setDirection(Servo.Direction.FORWARD);

            telemetry.addData("Position", servoPosition);
        }
    }
}