package org.firstinspires.ftc.teamcode.friends.computerVision.tests.CV_Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends LinearOpMode {
    AprilTagCam aprilTag = new AprilTagCam();

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag.initApriltag();

        waitForStart();

        if(isStopRequested()) return;

        if(opModeIsActive()){
            aprilTag.processCameraOutput();
            aprilTag.checkDetections();
        }
    }
}