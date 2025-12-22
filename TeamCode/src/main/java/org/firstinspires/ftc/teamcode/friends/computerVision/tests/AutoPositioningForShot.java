package org.firstinspires.ftc.teamcode.friends.computerVision.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import android.util.Size;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp( name = "AutoPositioning")
public class AutoPositioningForShot extends LinearOpMode {

    // Change this for the distance we are going to shoot at
        final double DESIRED_DISTANCE = 18.0;

        //These constants will hopefully autocorrect any minor errors when the robot is trying to align with the tag
        //  Drive = Error * Gain. Smaller = More control or larger for a faster response.
        final double SPEED_GAIN =  0.02 ;   // So if u want 50% power for a 25cm error 0.5/25 = 0.02 (Forward movement)
        final double STRAFE_GAIN =  0.015 ;   //Same thing as above but for sideways control 0.25 / 25.0 = 0.015
        final double TURN_GAIN   =  0.01  ;   // Turn Control

        final double MAX_AUTO_SPEED = 0.5;   //  This is the maximum forward speed at which the robot approaches the desired target
        final double MAX_AUTO_STRAFE= 0.5;
        final double MAX_AUTO_TURN  = 0.3;

        private DcMotor leftFrontDrive   = null;
        private DcMotor rightFrontDrive  = null;
        private DcMotor leftBackDrive    = null;
        private DcMotor rightBackDrive   = null;

        private static final boolean USE_WEBCAM = true;  // Its true if we are using a webcam or false if not. I coded both cause i wasnt sure
        private static final int DESIRED_TAG_ID = -1;     // Its currently set for any tag
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    @Override public void runOpMode()
        {
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            double  drive           = 0.5;        // Desired forward power/speed (-1 to +1)
            double  strafe          = 0.5;        // Desired strafe power/speed (-1 to +1)
            double  turn            = 0.3;        // Desired turning power/speed (-1 to +1)

            // For starting th april tag tagging
            initAprilTag();

            // The hardware variables for the autonomous driving
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "FLM");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "FRM");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "BLM");
            rightBackDrive = hardwareMap.get(DcMotor.class, "BRM");


            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur (learnt that from a MarkRober video)

            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            while (opModeIsActive())
            {
                targetFound = false;
                // Used to hold the data for a detected AprilTag
                AprilTagDetection desiredTag = null;

                // Step through the list of detected tags and look for a matching tag. love my for loops
                // Put the tag for shooting here
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null)
                            && ((DESIRED_TAG_ID >= 0) ||  (detection.id == DESIRED_TAG_ID))  ){
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }

                // Tell Jonny  what we see, and what to do cause Jonny is gonna be a useless driver
                if (targetFound) {
                    telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f centimetres", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData(">","Drive using joystick to find target\n");
                }

                // If Left Bumper is being pressed and  we have found the desired target, Drive to target Automatically .
                if (gamepad1.left_bumper && targetFound) {

                    // Determine heading, range and Yaw (rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;

                    // Use the speed and turn to calculate how we want the robot to move.
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
                    drive  = -gamepad1.left_stick_y  / 1.5;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Add the correct axes motions to the drivetrain cause i have no idea how it works
                moveRobot(drive, strafe, turn);
                idle();
            }
        }

        /**
         * Positive X is forward
         * Positive Y is strafe left
         * Positive Yaw is counter-clockwise
         */
        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            double leftFrontPower    =  x -y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double leftBackPower     =  x +y -yaw;
            double rightBackPower    =  x -y +yaw;

            //  wheel powers are to be less than 1.0 otherwise we will al die when the robot moves randomly
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Sends the powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

        /**
         * start the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                    .setLensIntrinsics(1439.42, 1439.42, 970.514, 537.613)
                    .build();
            // These are placeholder values we will need to calibrate either using a chessboard or roughly guessing after figuring out the camera features

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                        .setCameraResolution(new Size(1920,1080))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams for some reason
        */

        private void    setManualExposure(int exposureMS, int gain) { //Figure out some values
            // Wait for the camera to be open, then use the control

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls which focuses onto the image
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
    }


