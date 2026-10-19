package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Small square for verifying a tuned Pedro follower; initializes drive and Pinpoint only. */
@Autonomous(name = "Pedro Auto Nav", group = "Pedro Pathing")
public final class PedroAutoNav extends LinearOpMode {
  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    Pose[] route = {
      new Pose(0, 0, 0), new Pose(24, 0, 0), new Pose(24, 24, Math.PI / 2),
      new Pose(0, 24, Math.PI), new Pose(0, 0, 0)
    };
    follower.setStartingPose(route[0]);
    follower.setMaxPower(0.35);
    try {
      telemetry.addLine("Pedro 24-inch square at 35% power. Run localization and tuning first.");
      OdometryTelemetry.add(telemetry);
      telemetry.update();
      waitForStart();
      if (isStopRequested()) {
        return;
      }
      follower.setPose(route[0]);
      for (int i = 1; i < route.length && opModeIsActive(); i++) {
        Path path = new Path(new BezierLine(route[i - 1], route[i]));
        path.setLinearHeadingInterpolation(route[i - 1].getHeading(), route[i].getHeading());
        follower.followPath(path, false);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && timer.seconds() < 10.0) {
          follower.update();
          Pose pose = follower.getPose();
          telemetry.addData("Segment", "%d / %d", i, route.length - 1);
          telemetry.addData("Pose", "X %.1f, Y %.1f in; heading %.1f deg",
              pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
          telemetry.addData("Pose Source", "Pedro: Pinpoint (2 pods + onboard IMU)");
          OdometryTelemetry.add(telemetry);
          telemetry.update();
          idle();
        }
        if (follower.isBusy()) {
          telemetry.addData("Status", "Stopped: segment %d timed out or OpMode stopped", i);
          OdometryTelemetry.add(telemetry);
          telemetry.update();
          break;
        }
      }
    } finally {
      follower.breakFollowing();
    }
  }
}
