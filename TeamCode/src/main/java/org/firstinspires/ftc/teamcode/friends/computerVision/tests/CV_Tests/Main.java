package org.firstinspires.ftc.teamcode.friends.computerVision.tests.CV_Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends LinearOpMode {

    AprilTagWebCam aprilTag = new AprilTagWebCam();

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag.initApriltag();

        waitForStart();

        if(opModeIsActive()){
            aprilTag.processCameraOutput();
            aprilTag.checkDetections();
        }
    }
}