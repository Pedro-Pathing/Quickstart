package org.firstinspires.ftc.teamcode.friends.computerVision.tests.CV_Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Locale;

// Step 1: Get the april tag detections without webcam

public class AprilTagCam {
    ArrayList<AprilTagDetection> detectionsList = new ArrayList<>();
    ArrayList<AprilTagDetection> currentDetections;
    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public void initApriltag(){
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam"), aprilTag);
    }
    public void processCameraOutput(){
        currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.isEmpty()){
            lookForAprilTags();
            return;
        }

        for(AprilTagDetection detection : currentDetections){
            if(detection.metadata == null) continue;
            else{
                processAprilTag(detection);
                detectionsList.add(detection);
            }
        }
    }
    public void processAprilTag(AprilTagDetection detection){
        telemetry.addLine(String.format(Locale.ENGLISH ,"\n==== (ID %d) %s", detection.id, detection.metadata.name));
        telemetry.addLine(String.format(Locale.ENGLISH ,"XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
    }
    public void lookForAprilTags(){

    } // SERVOS
    public void checkDetections(){
        if(!(detectionsList.isEmpty())){
            for(AprilTagDetection detection : detectionsList){
                telemetry.addLine(String.format(Locale.ENGLISH ,"\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.ENGLISH ,"XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            }
        }
    }
}