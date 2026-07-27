package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.videoio.Videoio;

import java.util.List;

/* JADX INFO: loaded from: classes7.dex */
public class AprilTagAligner {
    private AprilTagDetection currentDetection;
    private final int desiredTagId;
    private final DriveTrain driveTrain;
    private final LinearOpMode linearOpMode;
    private final VisionPortal visionPortal;
    private final SimplePIDController turnController = new SimplePIDController(0.01d, LynxServoController.apiPositionFirst, 5.0E-4d);
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean controllersPrimed = false;
    private final AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();

    public AprilTagAligner(LinearOpMode linearOpMode, DriveTrain driveTrain, int desiredTagId) {
        this.linearOpMode = linearOpMode;
        this.driveTrain = driveTrain;
        this.desiredTagId = desiredTagId;
        this.visionPortal = new VisionPortal.Builder().addProcessor(this.aprilTagProcessor).setCameraResolution(new Size(640, Videoio.CAP_PROP_XI_CC_MATRIX_01)).setCamera((CameraName) linearOpMode.hardwareMap.get(WebcamName.class, "Webcam 1")).build();
        this.loopTimer.reset();
    }

    public void updateDetection() {
        List<AprilTagDetection> detections = this.aprilTagProcessor.getDetections();
        this.currentDetection = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == this.desiredTagId) {
                this.currentDetection = detection;
                return;
            }
        }
    }

    public void turnToTag() {
        double turnCommand;
        if (this.currentDetection == null) {
            this.driveTrain.driveFieldCentric(LynxServoController.apiPositionFirst, LynxServoController.apiPositionFirst, LynxServoController.apiPositionFirst, this.driveTrain.getHeading());
            return;
        }
        if (!this.controllersPrimed) {
            this.turnController.reset();
            this.controllersPrimed = true;
        }
        double dt = this.loopTimer.seconds();
        this.loopTimer.reset();
        double turnError = this.currentDetection.ftcPose.bearing;
        double turnCommand2 = this.turnController.calculate(turnError, dt);
        if (Math.abs(turnError) > 0.25d) {
            turnCommand = turnCommand2 + (Math.signum(turnCommand2) * 0.12d);
        } else {
            turnCommand = LynxServoController.apiPositionFirst;
        }
        double appliedTurn = Range.clip(turnCommand, -0.4d, 0.4d);
        this.driveTrain.driveFieldCentric(LynxServoController.apiPositionFirst, LynxServoController.apiPositionFirst, appliedTurn, this.driveTrain.getHeading());
    }

    public void stop() {
        this.controllersPrimed = false;
    }

    public void close() {
        this.visionPortal.close();
    }

    public double getLastRangeInches() {
        return this.currentDetection != null ? this.currentDetection.ftcPose.range : LynxServoController.apiPositionFirst;
    }

    public int getTargetedTagId() {
        if (this.currentDetection != null) {
            return this.currentDetection.id;
        }
        return -1;
    }

    public void displayTelemetry() {
        if (this.currentDetection != null) {
            this.linearOpMode.telemetry.addData("Tag ID Found", Integer.valueOf(this.currentDetection.id));
            this.linearOpMode.telemetry.addData("Bearing Error", "%.2f deg", Double.valueOf(this.currentDetection.ftcPose.bearing));
            this.linearOpMode.telemetry.addData("Range to Tag", "%.2f in", Double.valueOf(this.currentDetection.ftcPose.range));
            return;
        }
        this.linearOpMode.telemetry.addData("AprilTag", "Searching for ID %d...", Integer.valueOf(this.desiredTagId));
    }

    public void align(boolean requested) {
    }

    public void setRangeHoldEnabled(boolean enabled) {
    }
}
