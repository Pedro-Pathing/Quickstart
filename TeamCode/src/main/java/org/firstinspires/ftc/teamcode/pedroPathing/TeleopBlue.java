package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* JADX INFO: loaded from: classes7.dex */
/*@TeleOp(name = "Field Centric TeleOp Blue")
public class TeleopBlue extends LinearOpMode {
    private AprilTagAligner aprilTagAligner;
    private ArtifactHandlingSystem artifactHandlingSystem;
    private ColorDetection colorDetection;
    private DriveTrain driveTrain;
    private RobotControls robotControls;

    @Override // com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
    public void runOpMode() throws InterruptedException {
        this.artifactHandlingSystem = new ArtifactHandlingSystem(this);
        this.robotControls = new RobotControls(this);
        this.driveTrain = new DriveTrain(this);
        this.colorDetection = new ColorDetection(this);
        this.aprilTagAligner = new AprilTagAligner(this, this.driveTrain, 20);
        configureMotorModes();
        waitForStart();
        this.driveTrain.resetYaw();
        while (opModeIsActive()) {
            try {
                this.robotControls.updateControls();
                this.aprilTagAligner.updateDetection();
                if (this.gamepad1.options) {
                    this.driveTrain.resetYaw();
                }
                if (this.robotControls.alignRobot) {
                    this.aprilTagAligner.turnToTag();
                } else {
                    this.driveTrain.driveFieldCentric(-this.gamepad1.left_stick_y, this.gamepad1.left_stick_x, this.gamepad1.right_stick_x, this.driveTrain.getHeading());
                    this.aprilTagAligner.stop();
                }
                handleSubsystems();
                if (this.robotControls.DEBUG) {
                    displayTelemetry();
                }
            } finally {
                if (this.aprilTagAligner != null) {
                    this.aprilTagAligner.close();
                }
            }
        }
    }

    private void handleSubsystems() {
        this.artifactHandlingSystem.updateLaunchVelocityForRange(this.aprilTagAligner.getLastRangeInches(), this.aprilTagAligner.getTargetedTagId());
        this.artifactHandlingSystem.shootingSystem(this.robotControls.shootArtifact, this.robotControls.motorBrake);
        this.artifactHandlingSystem.flapSystem(this.robotControls.flapArtifact);
        this.artifactHandlingSystem.manageIntakeWithAutoFeed(this.robotControls.intakeArtifact, this.robotControls.rejectIntakeArtifact, this.robotControls.shootArtifact > 0.1f, this.colorDetection.isArtifactAtBack(), this.colorDetection.getArtifactCount());
        this.colorDetection.setRGBIndicator();
    }

    private void configureMotorModes() {
        this.artifactHandlingSystem.configureMotorModes();
        this.driveTrain.configureMotorModes();
    }

    private void displayTelemetry() {
        this.driveTrain.displayTelemetry();
        this.telemetry.update();
    }
}*/
