package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Sensorange I2C Encoder Test (Panels)")
public class SensorangeI2CTest extends LinearOpMode {

    private Servo crServo;
    private I2cDeviceSynchSimple encoder;

    private static final I2cAddr ENCODER_ADDR = I2cAddr.create8bit(0x40);


    @Override
    public void runOpMode() throws InterruptedException {

        crServo = hardwareMap.get(Servo.class, "sensorangeServo");

        encoder = hardwareMap.get(I2cDeviceSynchSimple.class, "sensorangeEncoder");
        encoder.setI2cAddress(ENCODER_ADDR);

        telemetry.addLine("Sensorange I2C Encoder Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            double pwm = (power + 1) / 2.0;

            crServo.setPosition(pwm);

            byte[] data = encoder.read(0x00, 2);
            int raw = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);

            telemetry.addData("Power", power);
            telemetry.addData("PWM", pwm);
            telemetry.addData("Encoder", raw);
            telemetry.update();
        }
    }
}