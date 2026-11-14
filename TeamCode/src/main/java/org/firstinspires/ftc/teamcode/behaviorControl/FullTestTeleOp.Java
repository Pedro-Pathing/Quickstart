package org.firstinspires.ftc.teamcode.behaviorControl;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 *
 * Controls (gamepad1):
 *   Left stick          - drive forward/back and strafe
 *   Right stick X       - rotate
 *   Right bumper        - spin up launcher (closed-loop velocity control)
 *   Right trigger       - windmill servo spins continuously while held
 *   Left trigger        - intake in
 *   Left bumper         - intake reverse (takes priority over left trigger)
 */


@TeleOp(name = "FullTestTeleOp")
public class FullTestTeleOp extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * Launcher velocity in encoder ticks per second (28 ticks per revolution).
     * Max for a 6000 RPM motor is about 2800 ticks/sec, so 1120 is 40% (about 2400 RPM).
     * Change LAUNCHER_TARGET_VELOCITY to adjust shot speed.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1120;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // Right trigger must be pulled past this value to count as "pressed".
    public final double TRIGGER_THRESHOLD = 0.1;

    // Stick values smaller than this are ignored (fixes stick drift).
    // If the robot still creeps, raise this (try 0.10 or 0.15).
    public final double STICK_DEADBAND = 0.08;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double intakePower;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Adjust these based on your first test drive so left stick forward = robot forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Closed-loop velocity control for the launcher.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        // Servo directions (set before the initial power command).
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize the servos to stopped.
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Drive: forward stick is negative, so invert it. Deadband removes stick drift.
        mecanumDrive(
                -deadband(gamepad1.left_stick_y),
                deadband(gamepad1.left_stick_x),
                deadband(gamepad1.right_stick_x));

        // Intake: left trigger = in, left bumper = reverse.
        // (Right trigger is reserved for the windmill servo.)
        if (gamepad1.left_bumper) {
            intakePower = -1;
        } else {
            intakePower = gamepad1.left_trigger;
        }
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // Launcher and windmill.
        launch();

        telemetry.addData("Drive", "LF (%.2f) RF (%.2f) LB (%.2f) RB (%.2f)",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Stick raw", "LY (%.3f) LX (%.3f) RX (%.3f)",
                gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Intake power", "%.2f", intakePower);
        telemetry.addData("Launcher velocity", "%.0f (target %d)",
                launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Windmill power", "%.2f", windmillServo.getPower());
    }

    @Override
    public void stop() {
        // Make sure everything is off when the OpMode ends.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        intake.setPower(0);
        launcher.setVelocity(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    // Returns 0 for small stick values, otherwise the value unchanged.
    double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        // Right bumper: spin the launcher up to target velocity. Release: spin down.
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        // Right trigger: windmill servo spins continuously while held, independent of the launcher.
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            windmillServo.setPower(1);
        } else {
            windmillServo.setPower(0);
        }
    }
}