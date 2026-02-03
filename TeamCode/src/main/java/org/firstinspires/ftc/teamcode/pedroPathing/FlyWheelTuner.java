package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelTuner extends OpMode {
    public DcMotorEx shut1;
    public DcMotorEx shut2;
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double CurlTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    boolean previousY = false;
    boolean previousB = false;
    boolean previousDpadLeft = false;
    boolean previousDpadRight = false;
    boolean previousDpadUp = false;
    boolean previousDpadDown = false;

    @Override
    public void init() {
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");

        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut1.setDirection(DcMotorSimple.Direction.FORWARD);

        shut2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shut2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);

        telemetry.addLine("брат код работате bilordahouston🇺🇸");
    }

    @Override
    public void loop() {

        if (gamepad1.y && !previousY) {
            if (CurlTargetVelocity == highVelocity) {
                CurlTargetVelocity = lowVelocity;
            } else {
                CurlTargetVelocity = highVelocity;
            }
        }
        previousY = gamepad1.y;


        if (gamepad1.b && !previousB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        previousB = gamepad1.b;


        if (gamepad1.dpad_left && !previousDpadLeft) {
            F -= stepSizes[stepIndex];
        }
        previousDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !previousDpadRight) {
            F += stepSizes[stepIndex];
        }
        previousDpadRight = gamepad1.dpad_right;

        // Настройка P (D-Pad Up/Down)
        if (gamepad1.dpad_up && !previousDpadUp) {
            P += stepSizes[stepIndex];
        }
        previousDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !previousDpadDown) {
            P -= stepSizes[stepIndex];
        }
        previousDpadDown = gamepad1.dpad_down;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);

        shut1.setVelocity(CurlTargetVelocity);
        shut2.setVelocity(CurlTargetVelocity);

        double curVelocity1 = shut1.getVelocity();
        double curVelocity2 = shut2.getVelocity();
        double curVelocity = (curVelocity1 + curVelocity2) / 2.0;
        double error = CurlTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", CurlTargetVelocity);
        telemetry.addData("Current Velocity Shut1", "%.2f", curVelocity1);
        telemetry.addData("Current Velocity Shut2", "%.2f", curVelocity2);
        telemetry.addData("Current Velocity Avg", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}


