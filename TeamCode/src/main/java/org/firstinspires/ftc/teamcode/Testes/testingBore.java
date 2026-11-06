package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Test_Bore")
public class testingBore extends LinearOpMode {

    public void runOpMode(){

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        while (opModeIsActive()){


            double posBore = motor.getCurrentPosition();

            telemetry.addData("GIRO PELO BORE", posBore);
            telemetry.update();
        }
    }
}
