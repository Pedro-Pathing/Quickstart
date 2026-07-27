package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/* JADX INFO: loaded from: classes7.dex */
public class SimpleAprilTagHelper {
    private AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
    private AprilTagDetection lastDetection;
    private VisionPortal visionPortal;

    public SimpleAprilTagHelper(HardwareMap hardwareMap, String webcamName) {
        this.visionPortal = new VisionPortal.Builder().addProcessor(this.aprilTagProcessor).setCameraResolution(new Size(1280, 720)).setCamera((CameraName) hardwareMap.get(WebcamName.class, webcamName)).setStreamFormat(VisionPortal.StreamFormat.MJPEG).enableLiveView(true).build();
//        FtcDashboard.getInstance().startCameraStream(this.visionPortal, 30.0d);
    }

    public void updateDetection() {
        List<AprilTagDetection> detections = this.aprilTagProcessor.getDetections();
        this.lastDetection = null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == 21) {
                this.lastDetection = detection;
                return;
            }
        }
    }

    public int getTagId() {
        if (this.lastDetection != null) {
            return this.lastDetection.id;
        }
        return -1;
    }

    public void close() {
        this.visionPortal.close();
    }
}
