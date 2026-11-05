package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

@Configurable
@TeleOp(name = "Casablanca Test", group = "Test")
public class CasablancaTest extends OpMode {

  /**
   * Configurable toggle to bypass Casablanca safety/friction/heading-lock and cubic joystick
   * response, feeding raw gamepad sticks directly into Pedro Pathing.
   */
  public static boolean rawDirectDrive = false;

  public static boolean fieldcentric = false;

  private Follower follower;
  private FieldManager field;
  private List<LynxModule> allHubs;
  private Sentinel sentinel;
  private Casablanca casablanca;

  private double targetX;
  private double targetY;

  /** Set in init: the tuners' open lane beside the HIVE, on BLUE's side. (72, 72) is inside it. */
  private Pose startPose;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT

    // Initialize Field Manager for Dashboard visualization
    field = PanelsField.INSTANCE.getField();
    if (field != null) {
      field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    // Initialize LynxModule manual bulk caching
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    // Configure zero-power brake behavior on drive motors
    List<DcMotorEx> motors = hardwareMap.getAll(DcMotorEx.class);
    for (DcMotorEx motor : motors) {
      motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    sentinel = new Sentinel(Alliance.BLUE);
    casablanca = new Casablanca(sentinel);
    startPose = sentinel.field().fromRedFrame(config.casablanca.heading_tuner.calibration_pose);

    follower = Constants.create(hardwareMap);
    follower.setPose(startPose);

    // Draw the line to BLUE's starting-upward CELL, matching the alliance this test drives as.
    var opening = sentinel.field().upOpening(HiveCell.A);
    targetX = opening.x();
    targetY = opening.y();
  }

  @Override
  public void start() {
    follower.manual(0, 0, 0);
    casablanca.reset();
  }

  @Override
  public void loop() {
    // Clear bulk cache once per loop
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    follower.update();

    handleDrive();
    drawField();

    telemetry.update();
  }

  private void handleDrive() {
    Pose currentPose = follower.pose();

    if (rawDirectDrive) {
      // Direct raw drive: bypass cubic curve and Casablanca processing entirely
      double forwardRaw = -gamepad1.left_stick_y;
      double strafeRaw = -gamepad1.left_stick_x;
      double turnRaw = -gamepad1.right_stick_x;

      follower.manual(
          fieldcentric
              ? new DrivePowers(forwardRaw, strafeRaw, turnRaw)
              : ManualDrive.fieldCentric(
                  new DrivePowers(forwardRaw, strafeRaw, turnRaw), follower.pose().heading()));

      telemetry.addData("Mode", "[RAW DIRECT DRIVE (BYPASSED)]");
      telemetry.addData("Raw Stick Input", "F:%.2f S:%.2f T:%.2f", forwardRaw, strafeRaw, turnRaw);
    } else {
      // TeleOp drive: exact cubic joystick curves and teleop.max_speed scaling as TeleOpBase
      double maxSpeed = config.teleop.max_speed;
      double forward = Math.clamp(-Math.pow(gamepad1.left_stick_y, 3), -maxSpeed, maxSpeed);
      double strafe = Math.clamp(-Math.pow(gamepad1.left_stick_x, 3), -maxSpeed, maxSpeed);
      double turn = Math.clamp(-Math.pow(gamepad1.right_stick_x, 3), -maxSpeed, maxSpeed);

      // Adjust inputs via Casablanca safety, friction, and heading-lock controller
      double[] adjusted =
          casablanca.adjustDriveInput(
              currentPose,
              follower.velocity().toVector2D(),
              follower.velocity().omega,
              strafe,
              forward,
              turn,
              -gamepad1.right_stick_x);

      // Casablanca returns [strafe, forward, turn] -> Follower expects [forward, strafe, turn]
      follower.manual(
          fieldcentric
              ? new DrivePowers(adjusted[1], adjusted[0], adjusted[2])
              : ManualDrive.fieldCentric(
                  new DrivePowers(adjusted[1], adjusted[0], adjusted[2]),
                  follower.pose().heading()));

      telemetry.addData("Mode", "[TELEOP DRIVE + CASABLANCA]");
      telemetry.addData("Cubic Input", "F:%.2f S:%.2f T:%.2f", forward, strafe, turn);
      telemetry.addData(
          "Adjusted Output", "F:%.2f S:%.2f T:%.2f", adjusted[1], adjusted[0], adjusted[2]);
      telemetry.addData(
          "RotSafety",
          "safe=%b lookaheadRad=%.4f angVel=%.4f",
          casablanca.getLastRotationSafe(),
          casablanca.getLastLookaheadRad(),
          casablanca.getLastAngularVelocityUsed());

      telemetry.addData(
          "Protect flags",
          "depth=%b side=%b repulsePwr=%.2f",
          Casablanca.enableDepthProtection,
          Casablanca.enableSideProtection,
          Casablanca.wallRepulsionPower);
      var zone = casablanca.getLastProtectedZone();
      var bounds = casablanca.getLastRobotBounds();
      if (zone != null && bounds != null) {
        telemetry.addData(
            "Zone",
            "x[%.1f,%.1f] y[%.1f,%.1f]",
            zone.getMinX(),
            zone.getMaxX(),
            zone.getMinY(),
            zone.getMaxY());
        telemetry.addData(
            "Bot bounds",
            "x[%.1f,%.1f] y[%.1f,%.1f]",
            bounds.getMinX(),
            bounds.getMaxX(),
            bounds.getMinY(),
            bounds.getMaxY());
        telemetry.addData(
            "Gap to zone",
            "dX=%.1f dY=%.1f",
            Math.max(zone.getMinX() - bounds.getMaxX(), bounds.getMinX() - zone.getMaxX()),
            Math.max(zone.getMinY() - bounds.getMaxY(), bounds.getMinY() - zone.getMaxY()));
      }
      telemetry.addData(
          "Protect state",
          "fadeX=%.2f fadeY=%.2f depthScale=%.2f sideScale=%.2f rep(d/s)=%b/%b",
          casablanca.getLastLaneFadeX(),
          casablanca.getLastLaneFadeY(),
          casablanca.getLastDepthScale(),
          casablanca.getLastSideScale(),
          casablanca.getLastDepthRepulsion(),
          casablanca.getLastSideRepulsion());

      if (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1) {
        double inputMag = Math.hypot(forward, strafe);
        double outputMag = Math.hypot(adjusted[1], adjusted[0]);
        telemetry.addData("Magnitude", "In:%.2f Out:%.2f", inputMag, outputMag);
        if (inputMag > 0.1 && outputMag < 0.05) {
          telemetry.addData("WARNING", "Goal zone collision repulsion active!");
        }
      }
    }

    telemetry.addData(
        "Robot Pose",
        "X:%.2f in | Y:%.2f in | H:%.2f deg",
        currentPose.x(),
        currentPose.y(),
        Math.toDegrees(currentPose.heading()));

    telemetry.addData(
        "Robot Velocity",
        "X:%.2f in | Y:%.2f in | H:%.2f deg",
        follower.velocity().vx,
        follower.velocity().vy,
        Math.toDegrees(follower.velocity().omega));
  }

  private void drawField() {
    if (field != null) {
      DrawingUtil.drawFieldZones(field, sentinel);
      DrawingUtil.drawRobotOnField(
          field,
          follower.pose().x(),
          follower.pose().y(),
          follower.pose().heading(),
          0.0,
          targetX,
          targetY);
    }
  }
}
