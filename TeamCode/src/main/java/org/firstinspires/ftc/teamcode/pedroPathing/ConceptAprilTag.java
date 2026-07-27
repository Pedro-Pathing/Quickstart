package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/* JADX INFO: loaded from: classes7.dex */
@TeleOp(group = "Concept", name = "Concept: AprilTag")
@Disabled
public class ConceptAprilTag extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override // com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
    public void runOpMode() {
        initAprilTag();
        this.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        this.telemetry.addData(">", "Touch START to start OpMode");
        this.telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();
                this.telemetry.update();
                if (this.gamepad1.dpad_down) {
                    this.visionPortal.stopStreaming();
                } else if (this.gamepad1.dpad_up) {
                    this.visionPortal.resumeStreaming();
                }
                sleep(20L);
            }
        }
        this.visionPortal.close();
    }

    private void initAprilTag() {
        this.aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera((CameraName) this.hardwareMap.get(WebcamName.class, "Webcam"));
        builder.addProcessor(this.aprilTag);
        this.visionPortal = builder.build();
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        this.telemetry.addData("# AprilTags Detected", Integer.valueOf(currentDetections.size()));
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                this.telemetry.addLine(String.format("\n==== (ID %d) %s", Integer.valueOf(detection.id), detection.metadata.name));
                this.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", Double.valueOf(detection.ftcPose.x), Double.valueOf(detection.ftcPose.y), Double.valueOf(detection.ftcPose.z)));
                this.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", Double.valueOf(detection.ftcPose.pitch), Double.valueOf(detection.ftcPose.roll), Double.valueOf(detection.ftcPose.yaw)));
                this.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", Double.valueOf(detection.ftcPose.range), Double.valueOf(detection.ftcPose.bearing), Double.valueOf(detection.ftcPose.elevation)));
            } else {
                this.telemetry.addLine(String.format("\n==== (ID %d) Unknown", Integer.valueOf(detection.id)));
                this.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", Double.valueOf(detection.center.x), Double.valueOf(detection.center.y)));
            }
        }
        this.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        this.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        this.telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
