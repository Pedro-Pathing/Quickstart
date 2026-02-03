package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="testmotor", group="FTC")
public class testmotor extends OpMode {

    //    private CRServo l1, l2;
    private DcMotor motor;

    String last;


    @Override
    public void init() {
        last = "";


        motor = hardwareMap.get(DcMotor.class, "motor");


    }


    @Override
    public void loop() {


        // Управление intake на gamepad1 и gamepad2 (те же кнопки)
        if (gamepad1.left_trigger > 0 || gamepad2.right_bumper) {
            motor.setPower(-0.
            );

        } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            motor.setPower(0.7);

        } else {
            motor.setPower(0.0);

        }




    }


}
