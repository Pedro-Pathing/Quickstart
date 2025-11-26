package org.firstinspires.ftc.teamcode.friends.computerVision.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class ATWC extends OpMode {

    AprilTagWebCam ATWC = new AprilTagWebCam();

    @Override
    public void init(){
        ATWC.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        ATWC.update();
        AprilTagDetection id21 = ATWC.getTagByID(21);
        ATWC.displayDetectionTelemetry(id21);
        ATWC.stop();
    }
}
