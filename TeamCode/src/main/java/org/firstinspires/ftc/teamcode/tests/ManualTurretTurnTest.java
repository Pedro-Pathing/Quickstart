package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Configurable
@TeleOp(name = "Manual Turret Turn Tester", group = "Test")
public class ManualTurretTurnTest extends LinearOpMode {
  private Turret turret;
  private Follower follower;
  public static int turn_degrees = 60;

  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    follower = Constants.create(hardwareMap);
    follower.setPose(new Pose(0, 0, 0));
    turret = new Turret(hardwareMap, telemetry, follower);

    waitForStart();

    while (opModeIsActive()) {
      follower.update();

      // Zeroing off-centre moves where the travel limits sit; if the stops are then unreachable or
      // the encoder wrap lands mid-travel, driving to turn_degrees could hit a stop at speed.
      String zeroProblem = turret.getEncoderRangeWarning();
      if (gamepad1.dpad_down) {
        turret.zeroAtCurrentPosition();
      } else if (gamepad1.dpad_right && zeroProblem == null) {
        turret.setTargetTurnAngle(turn_degrees);
      }
      if (zeroProblem != null) {
        telemetry.addLine("!! Zero refused for driving: " + zeroProblem);
        telemetry.addLine("!! Centre the turret and zero again (DPad down).");
      }

      turret.updateTurret(follower.pose());

      telemetry.addData("Turret Angle", turret.getCurrentTurnAngle());
      telemetry.addData("Target Angle", turret.getTargetTurnAngle());
      telemetry.update();
    }
  }
}
