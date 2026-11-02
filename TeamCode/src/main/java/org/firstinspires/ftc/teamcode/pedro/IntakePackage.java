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
    int intakeVelocity = 0;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger_pressed) {
                intakeVelocity = 6000;
            }
            if (gamepad1.left_trigger_pressed) {
                intakeVelocity = -6000;
            }
            if (gamepad1.atRest()) {
                intakeVelocity = 0;
            }

            intake.setPower(intakeVelocity);
            telemetry.addData("Launch Velocity","%4.2f", intake.getPower());
            telemetry.update();
        }
    }
}