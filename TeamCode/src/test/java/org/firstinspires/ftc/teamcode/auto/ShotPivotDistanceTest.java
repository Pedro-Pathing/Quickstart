package org.firstinspires.ftc.teamcode.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

public class ShotPivotDistanceTest {
  @Test
  public void initialSolveAndAutoWindowUseTheTurretPivot() {
    double savedForward = config.turret.pivot_forward_in;
    double savedLeft = config.turret.pivot_left_in;
    config.turret.pivot_forward_in = 4;
    config.turret.pivot_left_in = 2;
    try {
      for (Alliance alliance : Alliance.values()) {
        FieldModel field = new FieldModel(alliance);
        HiveTracker hive = new HiveTracker(3, 1000, 500, () -> 0);
        ShotController controller =
            new ShotController(
                mock(Shooter.class),
                mock(Turret.class),
                mock(Intake.class),
                Pose::zero,
                Velocity::zero,
                null,
                field,
                hive,
                null);
        try {
          for (HiveCell cell : HiveCell.values()) {
            hive.restore(cell, cell == HiveCell.A ? 0 : 1);
            var target = controller.currentTarget();
            for (double heading : new double[] {0, Math.PI / 2, Math.PI}) {
              Pose centre = new Pose(target.x() - 30, target.y() - 20, heading);
              Pose pivot = Turret.pivotPose(centre);
              double expected = Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
              assertEquals(expected, controller.distanceToTarget(centre), 1e-12);
              for (BallType ball : BallType.values()) {
                var initial = AllianceAutoBase.solveInitialShot(centre, target, ball);
                assertEquals(expected, initial.distanceInches(), 1e-12);
                assertEquals(ball, initial.ballType());
                assertEquals(target.epoch(), initial.targetEpoch());
              }
            }
          }
        } finally {
          controller.shutdown();
        }
      }
    } finally {
      config.turret.pivot_forward_in = savedForward;
      config.turret.pivot_left_in = savedLeft;
    }
  }
}
