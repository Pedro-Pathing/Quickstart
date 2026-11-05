package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.Map;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

public final class OpModeUtil {

  /**
   * INIT telemetry for config trouble: a pushed config that was ignored (and why), and every
   * ConfigSanity problem with the values in use. Call from every OpMode's init_loop.
   */
  public static void reportConfig(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
    String rejection = org.firstinspires.ftc.teamcode.config.ConfigLoader.overrideRejection();
    if (rejection != null) {
      telemetry.addLine("!! Pushed config ignored: " + rejection);
    }
    for (String problem : ConfigSanity.problems()) {
      telemetry.addLine("!! Config: " + problem);
    }
  }

  /**
   * HIVE state and inventory saved by AUTO are only restored by a TELEOP initialized within this
   * long. They describe one match; restoring them into a later match (a TELEOP run without an AUTO
   * before it) would start that match believing in TIPS that never happened.
   */
  public static final long HANDOFF_MAX_AGE_MS = 120_000L;

  private OpModeUtil() {}

  /**
   * Match state handed from one OpMode to the next through the SDK blackboard. {@code inventory} is
   * -1 when the sender did not know how many SCORING ELEMENTS it held.
   */
  public record Handoff(HiveCell upCell, int tips, int inventory, long savedAtMs) {

    public boolean isFresh(long nowMs) {
      return nowMs - savedAtMs >= 0 && nowMs - savedAtMs <= HANDOFF_MAX_AGE_MS;
    }
  }

  public static FieldManager initPanelsField() {
    FieldManager field = PanelsField.INSTANCE.getField();
    field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    return field;
  }

  public static String getBlackboardPoseKey(Alliance alliance) {
    return alliance == Alliance.RED ? "RED_POSE" : "BLUE_POSE";
  }

  public static String getBlackboardHandoffKey(Alliance alliance) {
    return alliance == Alliance.RED ? "RED_HANDOFF" : "BLUE_HANDOFF";
  }

  public static Pose getSavedPose(Alliance alliance, Pose defaultPose) {
    return getSavedPose(OpMode.blackboard, alliance, defaultPose);
  }

  public static Pose getSavedPose(Map<String, Object> board, Alliance alliance, Pose defaultPose) {
    Object savedPose = board.get(getBlackboardPoseKey(alliance));
    return savedPose instanceof Pose pose ? pose : defaultPose;
  }

  public static void savePose(Alliance alliance, Pose pose) {
    OpMode.blackboard.put(getBlackboardPoseKey(alliance), pose);
  }

  public static void saveHandoff(
      Map<String, Object> board, Alliance alliance, HiveTracker hive, int inventory, long nowMs) {
    board.put(
        getBlackboardHandoffKey(alliance),
        new Handoff(hive.upCell(), hive.tips(), inventory, nowMs));
  }

  /**
   * The handoff saved for {@code alliance}, or null if there is none or it is from a stale match.
   */
  public static @Nullable Handoff getFreshHandoff(
      Map<String, Object> board, Alliance alliance, long nowMs) {
    Object saved = board.get(getBlackboardHandoffKey(alliance));
    if (saved instanceof Handoff handoff && handoff.isFresh(nowMs)) {
      return handoff;
    }
    return null;
  }

  public record TeleOpStart(
      boolean allianceConfirmed, boolean poseRestored, Pose pose, @Nullable Handoff handoff) {}

  /** Alliance, pose and match state are trusted together, only within the handoff window. */
  public static TeleOpStart resolveTeleOpStart(
      Map<String, Object> board, MatchProfile profile, long nowMs) {
    Handoff handoff =
        board.get("ALLIANCE") == profile.alliance()
            ? getFreshHandoff(board, profile.alliance(), nowMs)
            : null;
    Object savedPose = board.get(getBlackboardPoseKey(profile.alliance()));
    Pose pose = profile.startPose();
    boolean restorePose = false;
    if (handoff != null && savedPose instanceof Pose saved) {
      pose = saved;
      restorePose = true;
    }
    return new TeleOpStart(handoff != null, restorePose, pose, handoff);
  }

  public static void setupTurretAndShooter(Turret turret, Shooter shooter) {
    shooter.setShooterPIDFCoefficients();
    turret.setHoldAngle(0.0);
    turret.setAimMode(Turret.AimMode.IDLE);
  }

  public static void drawRobot(
      FieldManager field, Follower follower, Turret turret, CellTarget target) {
    DrawingUtil.drawRobotOnField(
        field,
        follower.pose().x(),
        follower.pose().y(),
        follower.pose().heading(),
        Math.toRadians(turret.getCurrentTurnAngle()),
        target.x(),
        target.y());
  }
}
