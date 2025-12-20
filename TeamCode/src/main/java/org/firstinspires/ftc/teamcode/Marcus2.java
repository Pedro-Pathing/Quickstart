package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Marcus2 extends LinearOpMode{
    @Override
    public void runOpMode(){
        Roti move = new Roti();
        Intake Itk = new Intake();
        move.Init(hardwareMap);
        Itk.Init(hardwareMap);
        while(!isStopRequested()&&opModeIsActive())
        {
            waitForStart();
            while(!isStopRequested()&&opModeIsActive())
            {
                move.Movement(gamepad1);
                Itk.take(gamepad1);
                Itk.climb(gamepad1);
            }
        }
    }
}