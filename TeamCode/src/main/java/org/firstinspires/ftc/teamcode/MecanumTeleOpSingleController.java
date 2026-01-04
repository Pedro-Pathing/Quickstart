package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@TeleOp
public class MecanumTeleOpSingleController extends LinearOpMode {
//hi
    @Override
    public void runOpMode() throws InterruptedException {

        //initialize motors
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor intake1 = hardwareMap.dcMotor.get("intake1");
        DcMotor intake2 = hardwareMap.dcMotor.get("intake2");
        DcMotor outtake = hardwareMap.dcMotor.get("outtake");




        //set motor actions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //------------ Mecanum Drive ------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.2;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower((y - x + rx) / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backRight.setPower((y + x - rx) / denominator);

            if(gamepad1.a) {
                intake1.setDirection(DcMotorSimple.Direction.FORWARD);
                intake2.setDirection(DcMotorSimple.Direction.FORWARD);
                intake1.setPower(1);
                intake2.setPower(1);
            }else{
                intake1.setPower(0);
                intake2.setPower(0);
            }if(gamepad1.b) {
                intake1.setDirection(DcMotorSimple.Direction.REVERSE);
                intake2.setDirection(DcMotorSimple.Direction.REVERSE);
                intake1.setPower(1);
                intake2.setPower(1);
            }else{
                intake1.setPower(0);
                intake2.setPower(0);
            }
            if(gamepad1.left_bumper){
                outtake.setDirection(DcMotorSimple.Direction.FORWARD);
                outtake.setPower(1);
            }else{
                outtake.setPower(0);
            }
        }
    }
}