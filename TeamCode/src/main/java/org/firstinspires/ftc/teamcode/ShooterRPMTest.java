package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Shooter RPM Test", group = "Examples")
public class ShooterRPMTest extends OpMode {
    public int currentRPM = 500;

    public DcMotorEx shooterMotor;
    public RevBlinkinLedDriver shooterLight;

    public Robot robot;
    private Timer buttontimer;


    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLight = hardwareMap.get(RevBlinkinLedDriver.class, "shooterLight");
        buttontimer = new Timer();
        buttontimer.resetTimer();
    }

    @Override
    public void start() {
        buttontimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad2.y && buttontimer.getElapsedTime() > 500) {
            currentRPM += 25;
            buttontimer.resetTimer();
        } else if (gamepad2.a && buttontimer.getElapsedTime() > 500) {
            currentRPM -= 25;
            buttontimer.resetTimer();
        }

        if (gamepad2.b) {
            shooterMotor.setVelocity(currentRPM);
        }
        else if (gamepad2.x) {
            shooterMotor.setVelocity(0);
        }
        if (currentRPM > 0 && shooterMotor.getVelocity() >= currentRPM) {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

        if (gamepad2.dpad_up) {
            if (shooterMotor.getVelocity() >= currentRPM) {
                robot.intake.startTransferOnly();
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
        } else if (gamepad2.left_bumper) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopTransfer();
            robot.intake.stopIntake();
        }


        telemetry.addData("Target RPM", currentRPM);
        telemetry.update();
    }
}

