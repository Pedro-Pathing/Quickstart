package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Robot: Smooth Tracking Fix", group = "Main")
public class FinalRobotCodetest extends LinearOpMode {

    /* --- HARDWARE --- */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor rotate, intake, shooter;
    private Servo kicker, pusher;
    private Limelight3A limelight;

    /* --- CONSTANTS --- */
    private static final double TICKS_PER_REV = 288.0;
    private static final double GEAR_RATIO = 1.6;
    private static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private static final double TICKS_PER_INCH = 1800.0;
    private static final double TRACK_WIDTH_TICKS = 15000.0;

    // --- TUNING (Reduced to stop oscillation) ---
    private double kP = 0.006;  // Very gentle correction speed
    private double kF = 0.04;   // Very low friction boost (was 0.15)
    private double deadband = 1.5; // Ignore errors smaller than 1.5 degrees
    private int targetTagID = 22;

    /* --- VARIABLES --- */
    private double robotX = 0, robotY = 0, robotHeading = 0;
    private double tagFieldX = 0, tagFieldY = 0;
    private boolean tagInitialized = false;

    // Toggle State Memory
    private boolean lastLB = false, lastRB = false;
    private boolean lastA = false, lastB = false, lastX = false;
    private boolean intakeOn = false, shooterOn = false, kickerOn = false;

    @Override
    public void runOpMode() {
        // Init Hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // IMPORTANT: Ensure these are plugged into the ports matching your config
        // If these return 0, your ghost tracking will not work.
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rotate = hardwareMap.get(DcMotor.class, "rotate");
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        pusher = hardwareMap.get(Servo.class, "pusher");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. ODOMETRY ---
            // These encoders MUST count up/down when you turn the robot chassis
            double lPos = leftFrontDrive.getCurrentPosition();
            double rPos = rightFrontDrive.getCurrentPosition();

            robotHeading = (rPos - lPos) / TRACK_WIDTH_TICKS * 360.0;
            double avgDist = ((lPos + rPos) / 2.0) / TICKS_PER_INCH;
            robotX = avgDist * Math.cos(Math.toRadians(robotHeading));
            robotY = avgDist * Math.sin(Math.toRadians(robotHeading));
//test
            // --- 2. DRIVE ---
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            leftFrontDrive.setPower(axial + lateral + yaw);
            rightFrontDrive.setPower(axial - lateral - yaw);
            leftBackDrive.setPower(axial - lateral + yaw);
            rightBackDrive.setPower(axial + lateral - yaw);

            // --- 3. TRACKING LOGIC ---
            LLResult result = limelight.getLatestResult();
            double currentTurretDeg = rotate.getCurrentPosition() / TICKS_PER_DEG;

            double rotatePower = 0;
            double trackingError = 0;
            boolean seeTagRightNow = false;

            // A. LIVE VISION
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == targetTagID) {
                        seeTagRightNow = true;

                        double xCam = fr.getCameraPoseTargetSpace().getPosition().x;
                        double zCam = fr.getCameraPoseTargetSpace().getPosition().z;
                        double distCM = Math.sqrt(xCam*xCam + zCam*zCam) * 100.0;

                        double tx = result.getTx();
                        double absAngle = Math.toRadians(robotHeading + currentTurretDeg + tx);
                        double distInches = distCM / 2.54;

                        tagFieldX = robotX + (distInches * Math.cos(absAngle));
                        tagFieldY = robotY + (distInches * Math.sin(absAngle));
                        tagInitialized = true;

                        trackingError = tx;
                        break;
                    }
                }
            }

            // B. GHOST TRACKING
            if (tagInitialized && !seeTagRightNow) {
                double angleToTag = Math.toDegrees(Math.atan2(tagFieldY - robotY, tagFieldX - robotX));
                double targetTurretAngle = angleToTag - robotHeading;

                trackingError = targetTurretAngle - currentTurretDeg;

                // Normalize error
                while (trackingError > 180)  trackingError -= 360;
                while (trackingError < -180) trackingError += 360;
            }

            // --- 4. MOTOR POWER (Anti-Oscillation) ---
            if (Math.abs(trackingError) > deadband) {
                rotatePower = trackingError * kP;

                // Lower friction boost to prevent shaking
                // Only apply if power is very low to prevent "kicking"
                if (Math.abs(rotatePower) < 0.15) {
                    if (rotatePower > 0) rotatePower += kF;
                    else rotatePower -= kF;
                }
            } else {
                rotatePower = 0;
            }

            rotate.setPower(Range.clip(rotatePower, -0.35, 0.35));

            // --- 5. CONTROLS ---
            if (gamepad1.left_bumper && !lastLB) targetTagID--;
            if (gamepad1.right_bumper && !lastRB) targetTagID++;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            if (gamepad1.a && !lastA) intakeOn = !intakeOn;
            lastA = gamepad1.a;
            intake.setPower(intakeOn ? -0.8 : 0);

            if (gamepad1.b && !lastB) shooterOn = !shooterOn;
            lastB = gamepad1.b;
            shooter.setPower(shooterOn ? 1.0 : 0);

            if (gamepad1.x && !lastX) kickerOn = !kickerOn;
            lastX = gamepad1.x;
            kicker.setPosition(kickerOn ? 1.0 : 0.5);
            pusher.setPosition(gamepad1.y ? 1.0 : 0.5);

            // --- 6. CRITICAL DEBUGGING ---
            telemetry.addLine(seeTagRightNow ? "VISION: LOCKED" : "VISION: SEARCHING");
            telemetry.addData("Motor Power", rotatePower);
            telemetry.addData("Error", "%.2f", trackingError);

            telemetry.addLine("--- ODOMETRY CHECK ---");
            // IF THESE ARE 0 WHEN YOU TURN, GHOST TRACKING WILL FAIL
            telemetry.addData("Left Encoder", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Right Encoder", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Robot Heading", "%.1f°", robotHeading);

            telemetry.update();
        }
    }
}