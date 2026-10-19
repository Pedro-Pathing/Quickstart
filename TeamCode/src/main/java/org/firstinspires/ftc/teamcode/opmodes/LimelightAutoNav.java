package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.localization.PoseEstimate;
import org.firstinspires.ftc.teamcode.localization.RobotLocalization;
import org.firstinspires.ftc.teamcode.navigation.HolonomicPoseController;
import org.firstinspires.ftc.teamcode.navigation.PoseControlOutput;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.telemetry.LocalizationTelemetry;

@Autonomous(name = "Limelight Auto Nav", group = "Localization")
public class LimelightAutoNav extends LinearOpMode {
  private RobotLocalization localization;
  private MecanumDrive drive;
  private HolonomicPoseController poseController;
  private LocalizationTelemetry localizationTelemetry;

  private static final Waypoint[] WAYPOINTS = {
    new Waypoint(24, 0, 0),
    new Waypoint(24, 24, 90),
    new Waypoint(0, 24, 180),
    new Waypoint(0, 0, 0)
  };

  @Override
  public void runOpMode() {
    localization = new RobotLocalization(hardwareMap);
    drive = new MecanumDrive(hardwareMap);
    poseController = HolonomicPoseController.createDefault();
    localizationTelemetry = new LocalizationTelemetry();
    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    localization.resetPose(0, 0, 0);
    localization.start();
    try {
      telemetry.addData("Status", "Initialized");
      telemetry.addData("LL Connected", localization.isLimelightConnected());
      OdometryTelemetry.add(telemetry);
      telemetry.update();

      waitForStart();

      for (Waypoint waypoint : WAYPOINTS) {
        if (!opModeIsActive()) {
          break;
        }
        driveToPosition(waypoint);
      }
    } finally {
      drive.stop();
      localization.stop();
    }
  }

  private void driveToPosition(Waypoint target) {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime settleTimer = new ElapsedTime();
    boolean settling = false;

    poseController.reset();

    while (opModeIsActive()
        && timer.seconds() < RobotConfig.Autonomous.WAYPOINT_TIMEOUT_SECONDS) {
      localization.update();

      PoseEstimate currentPose = localization.getPoseEstimate();
      PoseControlOutput output = poseController.calculate(currentPose, target);

      boolean atTarget =
          output.getDistanceError() < RobotConfig.Autonomous.POSITION_TOLERANCE_INCHES
              && Math.abs(output.getHeadingErrorDegrees())
                  < RobotConfig.Autonomous.HEADING_TOLERANCE_DEGREES;

      if (atTarget) {
        if (!settling) {
          settling = true;
          settleTimer.reset();
        }
        if (settleTimer.seconds() > RobotConfig.Autonomous.SETTLE_TIME_SECONDS) {
          break;
        }
      } else {
        settling = false;
      }

      drive.driveRobotCentric(output.getDrive(), output.getStrafe(), output.getTurn());

      telemetry.addData(
          "Target",
          "(%.1f, %.1f) @ %.1f°",
          target.getX(),
          target.getY(),
          target.getHeadingDegrees());
      telemetry.addData("Distance", "%.2f in", output.getDistanceError());
      telemetry.addData("Heading Error", "%.1f°", output.getHeadingErrorDegrees());
      telemetry.addData("PID Drive", "%.3f", output.getDrive());
      telemetry.addData("PID Strafe", "%.3f", output.getStrafe());
      telemetry.addData("PID Turn", "%.3f", output.getTurn());
      telemetry.addData("Settling", settling);
      localizationTelemetry.add(telemetry, localization, target);
      OdometryTelemetry.add(telemetry);
      telemetry.update();
    }

    drive.stop();
  }
}
