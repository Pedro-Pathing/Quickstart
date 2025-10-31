package org.firstinspires.ftc.teamcode.friends.computerVision.tests.BRUH;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
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
        AprilTagDetection id21 = aprilTagWebCam.getTagByID(21);
        aprilTagWebCam.displayDetectionTelemetry(id21);
    }
}
