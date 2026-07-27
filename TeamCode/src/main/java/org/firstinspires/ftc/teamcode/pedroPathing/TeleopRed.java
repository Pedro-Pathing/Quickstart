package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*@TeleOp(name = "Field Centric TeleOp Red")
public class TeleopRed extends LinearOpMode {
    private slideConstants slideConstants;
    //private AprilTagAligner aprilTagAligner;
    private ArtifactHandlingSystem artifactHandlingSystem;
    //private ColorDetection colorDetection;
    private DriveTrain driveTrain;
    private RobotControls robotControls;

    @Override
    public void runOpMode() throws InterruptedException {
        //this.artifactHandlingSystem = new ArtifactHandlingSystem(this);
        this.slideConstants = new slideConstants(hardwareMap);
        this.robotControls = new RobotControls(this);
        this.driveTrain = new DriveTrain(this);
        //this.colorDetection = new ColorDetection(this);
        //this.aprilTagAligner = new AprilTagAligner(this, this.driveTrain, 24);
        configureMotorModes();

        waitForStart(); // Motor stays completely dead-still here

        this.slideConstants.start(); // Arms the encoder targeting loop safely
        this.driveTrain.resetYaw();

        while (opModeIsActive()) {
            try {
                this.robotControls.updateControls();
                //this.aprilTagAligner.updateDetection();
                double linearPos = this.slideConstants.getCurrentPosition();

                if (gamepad1.dpad_up) {
                    // Safety check adjusted for negative encoder bounds (-1021.44)
                    if (linearPos > -1021.44) {
                        this.slideConstants.extendToHigh();
                    }
                }

                if (gamepad1.dpad_down) {
                    this.slideConstants.extendToBottom();
                }
                if (this.gamepad1.options || this.gamepad1.back) {
                    this.driveTrain.resetYaw();
                }
                if (this.robotControls.alignRobot) {
                    //this.aprilTagAligner.turnToTag();
                } else {
                    this.driveTrain.driveFieldCentric(-this.gamepad1.left_stick_y, this.gamepad1.left_stick_x, this.gamepad1.right_stick_x, this.driveTrain.getHeading());
                    //this.aprilTagAligner.stop();
                }
                //handleSubsystems();
                //if (this.robotControls.DEBUG) {
                    //displayTelemetry();
                //}
            } finally {
                if (this.robotControls != null) {
                    //this.aprilTagAligner.close();
                }
            }
        }
    }

    private void handleSubsystems() {
        //this.artifactHandlingSystem.updateLaunchVelocityForRange(this.aprilTagAligner.getLastRangeInches(), this.aprilTagAligner.getTargetedTagId());
        //this.artifactHandlingSystem.shootingSystem(this.robotControls.shootArtifact, this.robotControls.motorBrake);
        //this.artifactHandlingSystem.flapSystem(this.robotControls.flapArtifact);
        //this.artifactHandlingSystem.manageIntakeWithAutoFeed(this.robotControls.intakeArtifact, this.robotControls.rejectIntakeArtifact, this.robotControls.shootArtifact > 0.1f, this.robotControls.shootArtifact > 0.1f, this.robotControls.shootArtifact > 0.1f);
        //this.artifactHandlingSystem.checkMotorHealth();
        //this.colorDetection.setRGBIndicator();
        //this.colorDetection.setOuttakeIndicatorWithVelocity(this.artifactHandlingSystem.getLaunchVelocity(), this.artifactHandlingSystem.getActualVelocity());
    }

    private void configureMotorModes() {
        //this.artifactHandlingSystem.configureMotorModes();
        this.driveTrain.configureMotorModes();
    }

    private void displayTelemetry() {
        this.telemetry.addData("Field Heading", "%.2f Deg", Double.valueOf(Math.toDegrees(this.driveTrain.getHeading())));
        // Displays the slide encoder value clearly to the drivers
        this.telemetry.addData("Slide Ticks", this.slideConstants.getCurrentPosition());
        this.driveTrain.displayTelemetry();
        this.artifactHandlingSystem.displayTelemetry();
        //this.colorDetection.displayTelemetry();
        this.robotControls.displayTelemetry();
        this.telemetry.update();
    }
}*/
