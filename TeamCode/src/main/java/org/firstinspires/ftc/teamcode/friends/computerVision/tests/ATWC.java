package org.firstinspires.ftc.teamcode.friends.computerVision.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class ATWC extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void init(){
        aprilTagWebCam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        aprilTagWebCam.update();
        AprilTagDetection id21 = aprilTagWebCam.getTagByID(21);
        aprilTagWebCam.displayDetectionTelemetry(id21);
    }
}
