package org.firstinspires.ftc.teamcode.friends.computerVision.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagWebCamExample extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void init(){
        aprilTagWebCam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        // Update the vision portal
        aprilTagWebCam.update();
        AprilTagDetection id20 = aprilTagWebCam.getTagByID(20);
        aprilTagWebCam.displayDetectionTelemetry(id20);
    }
}
