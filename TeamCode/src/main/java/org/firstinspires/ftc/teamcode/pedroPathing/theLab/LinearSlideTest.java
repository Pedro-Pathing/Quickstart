package org.firstinspires.ftc.teamcode.pedroPathing.theLab;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LinearSlideTest")
@Disabled
public class LinearSlideTest extends LinearOpMode {

    private DcMotor containerMotor;

    static final double TICKS_PER_REV = 537.6;

    private int linearPos;

    static final int POSITION_RETRACTED = 0;
    static final int POSITION_MAX = -(int)(TICKS_PER_REV * 2.7);

    @Override
    public void runOpMode() throws InterruptedException {

        containerMotor = hardwareMap.get(DcMotor.class, "containerMotor");
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        containerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        containerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        containerMotor.setTargetPosition(POSITION_RETRACTED);
        containerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Ready! Make sure mechanisms are fully lowered, then press START.");
        telemetry.update();

        containerMotor.setPower(0.3);

        waitForStart();

        while (opModeIsActive()) {
            linearPos = containerMotor.getCurrentPosition();

            if (gamepad1.dpad_up) {
                if (linearPos < 1021.44) {
                    containerMotor.setTargetPosition(POSITION_MAX);
                } else {
                    containerMotor.setTargetPosition(linearPos);
                }
            }

            if (gamepad1.dpad_down) {
                containerMotor.setTargetPosition(POSITION_RETRACTED);
            }

            if (gamepad1.y) {
                containerMotor.setTargetPosition(linearPos);
            }

            telemetry.addLine("=== SLIDE MONITOR ===");
            telemetry.addData("Slide Target", containerMotor.getTargetPosition());
            telemetry.addData("Slide Current", linearPos);
            telemetry.update();
        }
    }
}
