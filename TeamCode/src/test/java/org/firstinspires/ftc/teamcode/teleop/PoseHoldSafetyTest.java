package org.firstinspires.ftc.teamcode.teleop;

import static org.mockito.Mockito.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.Test;

public class PoseHoldSafetyTest {
  @Test
  public void inPlaceHoldRejectsIntermediateHiveAndWallCollisions() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      var redFrame = new FieldModel(Alliance.RED).hiveFrame();
      double half = config.sentinel.robot_width / 2;
      double endpoint = half * (Math.cos(Math.toRadians(35)) + Math.sin(Math.toRadians(35)));
      double radius = Math.hypot(half, half);
      double distance = (endpoint + radius) / 2;
      for (boolean hive : new boolean[] {true, false}) {
        TeleOp op = new TeleOp();
        op.follower = mock(Follower.class);
        op.sentinel = new Sentinel(field);
        double x = hive ? redFrame.getMinX() - distance : distance;
        double y = hive ? redFrame.centre().y : 25;
        Pose from = field.fromRedFrame(new Pose(x, y, Math.toRadians(35)));
        Pose target = field.fromRedFrame(new Pose(x, y, Math.toRadians(55)));
        when(op.follower.pose()).thenReturn(from);
        op.holdPoseIfClear(target);
        verify(op.follower).stop();
        verify(op.follower, never()).hold(any());
      }
    }
  }

  @Test
  public void holdsRefuseHiveCrossingsAndAllowClearApproachesForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      TeleOp op = new TeleOp();
      op.follower = mock(Follower.class);
      op.sentinel = new Sentinel(alliance);
      FieldModel field = new FieldModel(alliance);
      Pose from = field.fromRedFrame(new Pose(58, 27, Math.PI / 2));
      Pose blocked = field.fromRedFrame(new Pose(58, 114, Math.PI / 2));
      Pose clear = field.fromRedFrame(new Pose(30, 27, Math.PI / 2));
      when(op.follower.pose()).thenReturn(from);
      op.holdPoseIfClear(blocked);
      verify(op.follower).stop();
      verify(op.follower, never()).hold(blocked);
      op.holdPoseIfClear(clear);
      verify(op.follower).hold(clear);
      op.holdPoseIfClear(from);
      verify(op.follower).hold(from);
    }
  }
}
