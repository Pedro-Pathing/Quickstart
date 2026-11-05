package org.firstinspires.ftc.teamcode.tests;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

@Autonomous(name = "Forward Left Return Test", group = "Tests")
public class ForwardLeftReturnTest extends OpMode {

  private Follower follower;

  private final PoseFactory p = PoseFactory.degrees();

  private final Pose startPose = p.of(0, 0, 0);
  private final Pose forwardPose = p.of(20, 0, 0);
  private final Pose leftPose = p.of(20, 10, 0);

  private Path forwardPath;
  private Path leftPath;
  private Path returnPath;

  private int step = 0;

  @Override
  public void init() {
    config.reload(); // pick up a pushConfig on re-init
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    follower = Constants.create(hardwareMap);
    follower.setPose(startPose);

    forwardPath = line(startPose, forwardPose).linear(startPose, forwardPose);
    leftPath = line(forwardPose, leftPose).linear(forwardPose, leftPose);
    returnPath = line(leftPose, startPose).linear(leftPose, startPose);

    telemetry.addLine("Ready: forward 20, left 10, return to start.");
    telemetry.update();
  }

  @Override
  public void start() {
    step = 1;
    follower.follow(forwardPath);
  }

  @Override
  public void loop() {
    follower.update();

    switch (step) {
      case 1:
        if (!follower.isBusy()) {
          follower.follow(leftPath);
          step = 2;
        }
        break;

      case 2:
        if (!follower.isBusy()) {
          follower.follow(returnPath);
          step = 3;
        }
        break;

      case 3:
        if (!follower.isBusy()) {
          step = 4;
        }
        break;

      default:
        break;
    }

    telemetry.addData("Step", step);
    telemetry.addData("X", follower.pose().x());
    telemetry.addData("Y", follower.pose().y());
    telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
    telemetry.addData("Finished", step == 4);
    telemetry.update();
  }
}
