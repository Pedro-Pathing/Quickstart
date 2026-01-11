package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.Arrays;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Curve;
import com.pedropathing.paths.PathBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroAutoTest")
public class PedroAutoTest extends LinearOpMode {

    Follower follower;

    @Override
    public void runOpMode() {

        // Create the follower
        follower = Constants.createFollower(hardwareMap);

        //Set starting pose (x, y, heading)
        follower.setPose(new Pose(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;

        // Define paths as Pose arrays

        // Path 1: Forward and right
        Pose[] path1 = new Pose[] {
                new Pose(0, 0, 0),
                new Pose(67, 0, 0),
                new Pose(67, 67, Math.toRadians(90))
        };

        // Path 2: Backward and turn
        Pose[] path2 = new Pose[] {
                new Pose(67, 67, Math.toRadians(90)),
                new Pose(0, 67, Math.toRadians(180)),
                new Pose(0, 0, Math.toRadians(270))
        };

        // Looping through each path
        Pose[][] allPaths = new Pose[][] { path1, path2 };

        for (int i = 0; i < allPaths.length; i++) {
            Pose[] path = allPaths[i];

            telemetry.addLine("Starting Path " + (i + 1));
            telemetry.update();

            //ChatGPT told me to add this cause a curve couldnt be instantiated so couldnt tell you how it works
            com.pedropathing.paths.PathBuilder builder = follower.pathBuilder();
            for (int j = 1; j < path.length; j++) {
                builder = builder
                        .addPath(new com.pedropathing.geometry.BezierLine(path[j - 1], path[j]))
                        .setLinearHeadingInterpolation(
                                path[j - 1].getHeading(),
                                path[j].getHeading()
                        );
            }

            follower.followPath(builder.build());
            //End of chatgpt bit.



            // Update loop while following path
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                updateTelemetry();
            }
        }

        telemetry.addLine("All Paths Complete!");
        telemetry.update();
        sleep(1000);
    }

    // Helper
    private void updateTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("X", String.format("%.2f", pose.getX()));
        telemetry.addData("Y", String.format("%.2f", pose.getY()));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(pose.getHeading())));
        telemetry.update();
    }
}

