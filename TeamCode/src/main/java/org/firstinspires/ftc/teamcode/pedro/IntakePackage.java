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

    private DcMotorEx launcher;
    int launcherVelocity = 0;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger_pressed) {
                launcherVelocity = 6000;
            }
            if (gamepad1.left_trigger_pressed) {
                launcherVelocity = -6000;
            }
            if (gamepad1.atRest()) {
                launcherVelocity = 0;
            }

            launcher.setVelocity(launcherVelocity);
            telemetry.addData("Launch Velocity","%4.2f", launcher.getVelocity());
            telemetry.update();
        }
    }
}