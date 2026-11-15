package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {
    // true = field-centric drive (needs IMU), false = robot-centric (handy for debugging strafing)
    private static final boolean FIELD_CENTRIC = true;

    // Drivetrain (encoders on the drive motors, speed capped at DRIVE_MAX_RPM at full stick)
    private static final double DRIVE_TICKS_PER_REV = 537.7; // set to your drive motor's encoder ticks per rev
    private static final double DRIVE_MAX_RPM = 250.0;

    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_RPM = 4500.0;
    private static final double SHOOTER_REVERSE_RPM = 500.0;
    private static final double SHOOTER_STOPPED_RPM = 30.0;

    private static final double SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN = 1.0;
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double TRANSFER_TICKS_PER_REV = 145.1;
    private static final double TRANSFER_FEED_RPM = 500.0;
    private static final double TRANSFER_REVERSE_RPM = 500.0;
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static final double INTAKE_TICKS_PER_REV = 537.7;
    private static final double INTAKE_RPM = 312.0;
    private static final double OUTTAKE_RPM = 312.0;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Drive: standard mecanum setup (left side reversed)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transferMotor.setDirection(TRANSFER_DIRECTION);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setDirection(INTAKE_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double driveMaxTPS = DRIVE_MAX_RPM * DRIVE_TICKS_PER_REV / 60.0;

        double shooterTargetTPS = SHOOTER_RPM * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN * SHOOTER_TICKS_PER_REV / 60.0;
        double shooterReverseTPS = SHOOTER_REVERSE_RPM * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN * SHOOTER_TICKS_PER_REV / 60.0;
        double shooterStoppedTPS = SHOOTER_STOPPED_RPM * SHOOTER_TICKS_PER_REV / 60.0;

        double transferFeedTPS = TRANSFER_FEED_RPM * TRANSFER_TICKS_PER_REV / 60.0;
        double transferReverseTPS = TRANSFER_REVERSE_RPM * TRANSFER_TICKS_PER_REV / 60.0;

        double intakeTPS = INTAKE_RPM * INTAKE_TICKS_PER_REV / 60.0;
        double outtakeTPS = OUTTAKE_RPM * INTAKE_TICKS_PER_REV / 60.0;

        // IMU orientation: logo LEFT, USB UP (adjust to match how the hub is mounted)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        telemetry.addLine("P1: Drive (Options = reset heading)");
        telemetry.addLine("P2: LT Intake, LB Outtake, RT Shoot, RB Reverse Flywheel, B Transfer, X Transfer Reverse");
        telemetry.update();

        try {
            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {

                // Mecanum drive
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // Reset heading (Options on gamepad 1) while the robot faces away from you
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x;
                double rotY = y;
                if (FIELD_CENTRIC) {
                    // Rotate the movement direction counter to the bot's rotation
                    rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                }

                rotX = rotX * 1.1; // Counteract imperfect strafing

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setVelocity(frontLeftPower * driveMaxTPS);
                backLeftMotor.setVelocity(backLeftPower * driveMaxTPS);
                frontRightMotor.setVelocity(frontRightPower * driveMaxTPS);
                backRightMotor.setVelocity(backRightPower * driveMaxTPS);

                // Intake
                boolean intakeHeld = gamepad2.left_trigger > 0.05;
                boolean intakeActive = false;
                if (intakeHeld && !gamepad2.left_bumper) {
                    intakeMotor.setVelocity(intakeTPS);
                    intakeActive = true;
                } else if (gamepad2.left_bumper && !intakeHeld) {
                    intakeMotor.setVelocity(-outtakeTPS);
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

                telemetry.addData("Heading (deg)", "%.0f", Math.toDegrees(botHeading));
                telemetry.addData("Drive FL/BL", "%.2f / %.2f", frontLeftPower, backLeftPower);
                telemetry.addData("Drive FR/BR", "%.2f / %.2f", frontRightPower, backRightPower);
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