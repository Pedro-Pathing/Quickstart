package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp
@Disabled
public class AtanAlignTest extends OpMode {

  private Follower follower;

  public static double GOAL_X = 129.0;
  public static double GOAL_Y = 132.0;

  public static double X = 45;
  public static double Y = 45;

  private FieldManager field;

  private final Pose initialPose = new Pose(72, 72, 0);

  @Override
  public void init() {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    follower = Constants.create(hardwareMap);
    follower.setPose(initialPose);
    follower.update();

    field = PanelsField.INSTANCE.getField();
    field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
  }

  @Override
  public void start() {}

  @Override
  public void loop() {
    if (gamepad1.bWasPressed()) {
      follower.manual(0, 0, 0);
    }
    if (!follower.holding()) {
      follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
    follower.update();
    Pose currentPose = follower.pose();

    if (gamepad1.aWasPressed() && !follower.isBusy()) {

      double deltaX = GOAL_X - currentPose.x();
      double deltaY = GOAL_Y - currentPose.y();
      double targetHeading = Math.atan2(deltaY, deltaX);

      follower.hold(new Pose(currentPose.x(), currentPose.y(), targetHeading));
    }
  }
}
