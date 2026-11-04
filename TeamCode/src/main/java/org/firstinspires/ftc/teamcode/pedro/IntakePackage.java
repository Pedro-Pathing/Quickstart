package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IntakePackage", group = "Linear OpMode")
public class IntakePackage extends LinearOpMode {

    private DcMotor intake;
    int intakePower = 0;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.startWasPressed()) {
                intakePower = 6000;
            }
            if (gamepad1.backWasPressed()) {
                intakePower = 0;
            }

            intake.setPower(intakePower);
            telemetry.addData("Intake Power","%4.2f", intake.getPower());
            telemetry.update();
        }
    }
}
