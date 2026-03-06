package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Configurable
public class LightTest extends LinearOpMode {
    public static double color = 0.28;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftLight = hardwareMap.get(Servo.class, "light_ll");
        Servo centerLight = hardwareMap.get(Servo.class, "light_cl");
        Servo rightLight = hardwareMap.get(Servo.class, "light_rl");
        waitForStart();
        while (!isStopRequested()){
            leftLight.setPosition(color);
            centerLight.setPosition(color);
            rightLight.setPosition(color);
        }
    }
}
