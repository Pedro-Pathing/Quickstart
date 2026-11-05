package org.firstinspires.ftc.teamcode.records;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;

/**
 * Everything alliance-specific a match needs, threaded explicitly into {@code Robot} (there is no
 * static "current alliance"). Poses are TeleOp hold points from {@code teleop.poses.<alliance>};
 * BLUE's are the 180-degree rotation of RED's unless overridden.
 */
public record MatchProfile(
    Alliance alliance,
    Pose startPose,
    Pose parkPose,
    Pose shootAPose,
    Pose shootBPose,
    Pose flowerPose) {

  /** The hold pose facing {@code cell}'s opening from the side it opens toward. */
  public Pose shootPose(HiveCell cell) {
    return cell == HiveCell.A ? shootAPose : shootBPose;
  }

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    String prefix = "teleop.poses." + (alliance == Alliance.RED ? "red" : "blue") + ".";
    return new MatchProfile(
        alliance,
        ConfigLoader.load(Pose.class, prefix + "start"),
        ConfigLoader.load(Pose.class, prefix + "park"),
        ConfigLoader.load(Pose.class, prefix + "shoot_a"),
        ConfigLoader.load(Pose.class, prefix + "shoot_b"),
        ConfigLoader.load(Pose.class, prefix + "flower"));
  }
}
