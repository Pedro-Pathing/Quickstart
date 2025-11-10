package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control (CRServo).
 *
 * Author:
 *   Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 *   3.1, 10/2025
 */
@TeleOp(name = "RobotTeleop", group = "Examples")
public class RobotTeleop extends OpMode {

    private Follower follower;
    private static final double DEAD_ZONE = 0.1;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;

    private CRServo turretCR;
    private static final double TURRET_POWER = 0.45;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0); // start stopped

        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
    }

    @Override
    public void start() {

        follower.startTeleopDrive();
        follower.setMaxPower(1.0);

    }

    @Override
    public void loop() {

        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        // NOTE: rotation is negated to match PedroPathing's TeleOp example (prevents reversed/odd rotation behavior)
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(
                yInput * powerScale,  // forward/backward
                xInput * powerScale,  // strafe
                turnInput * powerScale, // rotation (negated)
                true                   // robot-centric
        );

        follower.update();

        // Shooter RPM presets
        if (gamepad2.b) {
            shooterMotor.setVelocity(1420);
        } else if (gamepad2.a) {
            shooterMotor.setVelocity(0);
        } else if (gamepad2.x) {
            shooterMotor.setVelocity(1035);
        } else if (gamepad2.y) {
            shooterMotor.setVelocity(-1200);
        }

        // Turret control (fixed: check gamepad2 on both dpad sides)
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            turretCR.setPower(TURRET_POWER); // rotate right
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            turretCR.setPower(-TURRET_POWER); // rotate left
        } else {
            turretCR.setPower(0.0); // stop when no dpad pressed or both pressed
        }

        // Intake / transfer
        if (gamepad2.right_bumper) {
            intakeMotor.setPower(0.5);
        } else if (gamepad2.left_bumper) {
            intakeMotor.setPower(-0.5);
        } else {
            intakeMotor.setPower(0.0);
        }

        if (gamepad2.dpad_up) {
            transferMotor.setPower(0.75);
        }
        else if (gamepad2.dpad_down) {
            transferMotor.setPower(-0.75);
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }
        else {
            transferMotor.setPower(0.0);
        }

        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        telemetry.addData("Shooter RPM", shooterMotor.getVelocity());
        telemetry.addData("Turret Power", turretCR.getPower());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        if (turretCR != null) turretCR.setPower(0.0);
    }
}
