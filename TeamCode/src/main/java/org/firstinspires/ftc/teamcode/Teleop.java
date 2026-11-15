package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
/**
 *
 * 
 * Teleop
 */
@TeleOp
public class Teleop extends LinearOpMode {
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_RPM = 4500.0;
    private static final double SHOOTER_REVERSE_RPM = 1000.0;
    private static final double SHOOTER_STOPPED_RPM = 30.0;

    private static final double SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN = 1.0;
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double TRANSFER_TICKS_PER_REV = 145.1;
    private static final double TRANSFER_FEED_RPM = 300.0;
    private static final double TRANSFER_REVERSE_RPM = 300.0;
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double INTAKE_TICKS_PER_REV = 537.7;
    private static final double INTAKE_RPM = 312.0;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transferMotor.setDirection(TRANSFER_DIRECTION);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(INTAKE_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double shooterTargetTPS = SHOOTER_RPM * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN * SHOOTER_TICKS_PER_REV / 60.0;
        double shooterReverseTPS = SHOOTER_REVERSE_RPM * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN * SHOOTER_TICKS_PER_REV / 60.0;
        double shooterStoppedTPS = SHOOTER_STOPPED_RPM * SHOOTER_TICKS_PER_REV / 60.0;

        double transferFeedTPS = TRANSFER_FEED_RPM * TRANSFER_TICKS_PER_REV / 60.0;
        double transferReverseTPS = TRANSFER_REVERSE_RPM * TRANSFER_TICKS_PER_REV / 60.0;

        double intakeTPS = INTAKE_RPM * INTAKE_TICKS_PER_REV / 60.0;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("P1: Drive");
        telemetry.addLine("P2: LT Intake (also runs Transfer + reverses Flywheel), RT Shoot, RB Reverse Flywheel, B Transfer, X Transfer Reverse");
        telemetry.update();

        try {
            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

                frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
                backLeftMotor.setPower((rotY - rotX + rx) / denominator);
                frontRightMotor.setPower((rotY - rotX - rx) / denominator);
                backRightMotor.setPower((rotY + rotX - rx) / denominator);

                // Intake
                boolean intakeHeld = gamepad2.left_trigger > 0.05;
                boolean outakeHeld = gamepad2.left_bumper  > 0.05;
                boolean intakeActive = false;
                if (intakeHeld && !outakeHeld) {
                    intakeMotor.setVelocity(intakeTPS);
                    intakeActive = true;
                } else if (outakeHeld && !intakeHeld) {
                    intakeMotor.setVelocity(-intakeTPS);
                
                } else {
                    intakeMotor.setPower(0.0);
                }

                // Transfer: manual B/X takes priority, otherwise runs with the intake
                if (gamepad2.b && !gamepad2.x) {
                    transferMotor.setVelocity(transferFeedTPS);
                } else if (gamepad2.x && !gamepad2.b) {
                    transferMotor.setVelocity(-transferReverseTPS);
                } else if (intakeActive) {
                    transferMotor.setVelocity(transferFeedTPS);
                } else {
                    transferMotor.setPower(0.0);
                }

                // Shooter: RT shoots (priority), RB or intake reverses once flywheel has stopped
                boolean shooterHeld = gamepad2.right_trigger > 0.05;
                boolean shooterReverseHeld = gamepad2.right_bumper || intakeActive;
                boolean flywheelSpinningForward = shooterMotor.getVelocity() > shooterStoppedTPS;
                boolean reverseActive = false;

                if (shooterHeld) {
                    shooterMotor.setVelocity(shooterTargetTPS);
                } else if (shooterReverseHeld && !flywheelSpinningForward) {
                    // Only reverse once the flywheel has fully stopped
                    shooterMotor.setVelocity(-shooterReverseTPS);
                    reverseActive = true;
                } else {
                    shooterMotor.setPower(0.0);
                }

                double shooterMeasuredRPM = shooterMotor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV
                        / SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN;
                double transferMeasuredRPM = transferMotor.getVelocity() * 60.0 / TRANSFER_TICKS_PER_REV;
                double intakeMeasuredRPM = intakeMotor.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV;

                telemetry.addData("Shooter Mode",
                        shooterHeld ? "SHOOT"
                                : (reverseActive ? "REVERSE"
                                : (shooterReverseHeld ? "WAITING TO STOP" : "OFF")));
                telemetry.addData("Shooter Target RPM", shooterHeld ? String.format("%.0f", SHOOTER_RPM) : "OFF");
                telemetry.addData("Shooter Measured RPM", "%.0f", shooterMeasuredRPM);
                telemetry.addData("Transfer Measured RPM", "%.0f", transferMeasuredRPM);
                telemetry.addData("Intake Measured RPM", "%.0f", intakeMeasuredRPM);
                telemetry.update();
            }
        } finally {
            shooterMotor.setPower(0.0);
            transferMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
            frontLeftMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
        }
    }
}