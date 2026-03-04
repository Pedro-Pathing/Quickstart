package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Robot: Final", group = "Main")
public class FinalRobotCode extends LinearOpMode {

    /* Hardware Members */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor rotate, intake, shooter;
    private Servo kicker, pusher, hood;
    private Limelight3A limelight;

    /* Limelight Tuning Constants */
    private static final double kP = 0.02;
    private static final double MAX_ROTATE_POWER = 0.25;
    private static final double TX_DEADBAND = 1.2;
    private static final double GAIN = 0.2; // Low-pass filter gain
    private double smoothedTx = 0;
    private int targetTagID = 20;

    /* State tracking for toggle/button debouncing */
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // 2. Initialize Mechanisms
        rotate  = hardwareMap.get(DcMotor.class, "rotate");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood"); // TODO add the hood servo to the config
        kicker  = hardwareMap.get(Servo.class, "kicker");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Initialize Vision
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized - Ready to Fly");
        telemetry.update();

        waitForStart();

        /*
        * all button mappings:
        * Left stick: forward and backward, strafe left and right
        * right stick, rotate (change heading) left and right
        * a: intake
        * b: shooter
        * x: kicker on intake
        * y: pusher on shooter
        * bumpers:  limelight targettagID
        * dpad up/down: override limelight rotation
        * right trigger then bumpers: change hood angle */

        while (opModeIsActive()) {

            // --- SECTION 1: MECANUM DRIVE ---
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double lf = axial + lateral + yaw;
            double rf = axial - lateral - yaw;
            double lb = axial - lateral + yaw;
            double rb = axial + lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);


            // --- SECTION 2: LIMELIGHT TARGETING (ROTATE MOTOR) ---
            LLResult result = limelight.getLatestResult();

            // Cycle target ID with bumpers
            if (gamepad1.left_bumper && !lastLeftBumper) targetTagID = Math.max(20, targetTagID - 1);
            if (gamepad1.right_bumper && !lastRightBumper) targetTagID = Math.min(25, targetTagID + 1);
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            double rotatePower = 0.0;
            boolean tagFound = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetTagID) {
                        tagFound = true;
                        // Apply Smoothing Filter
                        smoothedTx = (result.getTx() * GAIN) + (smoothedTx * (1.0 - GAIN));

                        if (Math.abs(smoothedTx) > TX_DEADBAND) {
                            rotatePower = smoothedTx * kP;
                        }
                        break;
                    }
                }
            } else {
                smoothedTx = 0; // Reset filter if vision lost
            }

            // Constraints and Manual Overrides for Rotation
            rotatePower = Math.max(-MAX_ROTATE_POWER, Math.min(MAX_ROTATE_POWER, rotatePower));
            if (gamepad1.dpad_up) rotatePower = 0.3;      // Manual override up
            else if (gamepad1.dpad_down) rotatePower = -0.3; // Manual override down

            rotate.setPower(rotatePower);



            // --- SECTION 3: MECHANISMS (INTAKE, SHOOTER, SERVOS) ---

            // Intake (Motor) - Spin while A is pressed
            if (gamepad1.a) intake.setPower(-0.8);
            else intake.setPower(0);

            // Shooter (Motor) - Spin while B is pressed
            if (gamepad1.b) shooter.setPower(1.0);
            else shooter.setPower(0);

            // Kicker (Servo) - Continuous rotation while X is pressed
            // (Assumes continuous rotation servo where 1.0 is full speed one way)
            if (gamepad1.x) kicker.setPosition(1.0);
            else kicker.setPosition(0.5); // 0.5 is usually "Stop" for CR Servos

            // Pusher (Servo) - Continuous rotation while Y is pressed
            if (gamepad1.y) pusher.setPosition(1);
            else pusher.setPosition(0.5); // reset by moving down:

            // Hood (Servo) - Moves up and down (ideally we want limelight to do this)
            if ((gamepad1.right_trigger) > 0.0) { // right_trigger returns float
                if (gamepad1.right_bumper) {
                    hood.setPosition(hood.getPosition() + 0.1);
               } else if (gamepad1.left_bumper) {
                    hood.setPosition(hood.getPosition() - 0.1);
                }
            } // Marwan added this in anticipation of the new variable-hood shooter


            /*
            * if (gamepad.b) {
            *
            * }
            *
            *
            * */


            // --- SECTION 4: TELEMETRY ---
            telemetry.addData("Target ID", targetTagID);
            telemetry.addData("Vision Locked", tagFound);
            telemetry.addData("Rotate Power", "%.2f", rotatePower);
            telemetry.addData("Drive", "LF:%.2f RF:%.2f", lf, rf);
            telemetry.update();
        }
    }
}