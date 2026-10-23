package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls;
import org.firstinspires.ftc.teamcode.control.DualGamepadDriveControls.DriveCommand;
import org.firstinspires.ftc.teamcode.control.PositionHoldControls;
import org.firstinspires.ftc.teamcode.control.PositionHoldControls.Transition;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/**
 * Driver TeleOp that resists pushes: whenever nobody is driving, the robot holds the field pose
 * where it stopped and drives itself back there if it is pushed from any direction or turned.
 */
@TeleOp(name = "TeleOp: Hold Position", group = "Competition")
public final class HoldPositionTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    Follower follower = Constants.createFollower(hardwareMap);
    GoBildaPinpointDriver pinpoint =
        hardwareMap.get(GoBildaPinpointDriver.class, RobotConfig.Hardware.PINPOINT);
    PositionHoldControls hold =
        new PositionHoldControls(
            RobotConfig.PositionHold.SETTLED_SPEED_INCHES_PER_SECOND,
            Math.toRadians(RobotConfig.PositionHold.SETTLED_TURN_DEGREES_PER_SECOND),
            RobotConfig.PositionHold.SETTLE_TIMEOUT_SECONDS);
    ElapsedTime clock = new ElapsedTime();

    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("Keep still during Pinpoint calibration.");
    telemetry.addLine("Left stick: drive/strafe | Triggers: rotate");
    telemetry.addLine("Release all drive inputs: robot holds its position against pushes");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    waitForStart();

    try {
      if (isStopRequested()) {
        return;
      }
      follower.setStartingPose(new Pose(0, 0, 0));
      follower.setPose(new Pose(0, 0, 0));
      follower.startTeleopDrive(true);
      clock.reset();

      while (opModeIsActive()) {
        DriveCommand command = DualGamepadDriveControls.read(gamepad1, gamepad2);
        Pose pose = follower.getPose();
        Transition transition =
            hold.update(
                command.isActive(), pose.getX(), pose.getY(), pose.getHeading(), clock.seconds());

        if (transition == Transition.START_HOLD) {
          // Pedro's translational and heading PIDs drive the robot back to this pose when pushed.
          follower.holdPoint(new Pose(hold.getHoldX(), hold.getHoldY(), hold.getHoldHeading()));
        } else if (transition == Transition.START_DRIVER) {
          follower.startTeleopDrive(true);
        }

        if (hold.getMode() != PositionHoldControls.Mode.HOLDING) {
          // Zero input while settling brakes the drivetrain until the robot stops.
          // Pedro uses positive left strafe and counterclockwise turn; the mapper uses right/CW.
          follower.setTeleOpDrive(
              command.getDrive(), -command.getStrafe(), -command.getRotate(), true);
        }
        follower.update();

        pose = follower.getPose();
        telemetry.addData("Mode", hold.getMode());
        telemetry.addData("Active Driver", "Gamepad %d", command.getGamepadNumber());
        telemetry.addData(
            "Drive / Strafe / Rotate",
            "%.2f / %.2f / %.2f",
            command.getDrive(),
            command.getStrafe(),
            command.getRotate());
        telemetry.addData("Field Position", "X %.2f in, Y %.2f in", pose.getX(), pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        if (hold.getMode() == PositionHoldControls.Mode.HOLDING) {
          telemetry.addData(
              "Hold Target",
              "X %.2f in, Y %.2f in, %.1f°",
              hold.getHoldX(),
              hold.getHoldY(),
              Math.toDegrees(hold.getHoldHeading()));
          telemetry.addData(
              "Pushed Off By",
              "%.2f in, %.1f°",
              hold.distanceFromHold(pose.getX(), pose.getY()),
              Math.toDegrees(hold.headingErrorFromHold(pose.getHeading())));
        }
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        OdometryTelemetry.add(telemetry);
        telemetry.update();
        idle();
      }
    } finally {
      follower.breakFollowing();
    }
  }
}
