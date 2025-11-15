package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

@TeleOp(name = "RGBLight", group = "Examples")
public class RgbLightTest extends OpMode {


    public Servo rgbLight;
    public Timer buttonTimer;
    private double cur_val = 0.0;

    @Override
    public void init() {
        rgbLight = hardwareMap.get(Servo.class, "light");
    }

    @Override
    public void start() {
        rgbLight.setPosition(0.0);
        buttonTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && buttonTimer.getElapsedTime() > 250) {
            cur_val += 0.1;
            buttonTimer.resetTimer();
        }
        if (gamepad1.dpad_down && buttonTimer.getElapsedTime() > 250) {
            cur_val -= 0.1;
            buttonTimer.resetTimer();
        }
        telemetry.addData("CurLight value: ", cur_val);
        telemetry.update();
    }
}
