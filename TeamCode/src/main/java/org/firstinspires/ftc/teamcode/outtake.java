package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class outtake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor outtakeMotor = hardwareMap.dcMotor.get("intake_motor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //set motor actions
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            if (gamepad1.a) {
                outtakeMotor.setPower(1);
                outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                outtakeMotor.setPower(0);
            }


        }
    }
}
