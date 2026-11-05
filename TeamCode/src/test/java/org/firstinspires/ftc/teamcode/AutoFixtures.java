package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.PathUtil.pcurve;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Loads every autonomous routine's poses for an alliance and rebuilds exactly the paths its OpMode
 * builds in {@code buildPaths()}, so tests can check what the robot will actually drive without an
 * OpMode or a hardware map. Keep this in step with {@code CellAAuto}, {@code CellBAuto} and {@code
 * ParkOnlyAuto}: those are the source of truth.
 */
final class AutoFixtures {

  private AutoFixtures() {}

  /** A place the routine shoots from, and the CELL it shoots into. */
  record Volley(String name, HiveCell cell, Pose pose) {}

  /** One routine for one alliance: its key poses, every path it may follow and its volleys. */
  record Routine(
      String name,
      Alliance alliance,
      Pose start,
      Pose park,
      Map<String, Path> paths,
      List<Volley> volleys) {

    @Override
    public String toString() {
      return name + "/" + alliance;
    }
  }

  static String side(Alliance alliance) {
    return alliance == Alliance.RED ? "red" : "blue";
  }

  static config.CellAAuto cellAPoses(Alliance alliance) {
    return ConfigLoader.loadMerged(
        config.CellAAuto.class, "auto_poses.cell_a." + side(alliance), "auto");
  }

  static config.CellBAuto cellBPoses(Alliance alliance) {
    return ConfigLoader.loadMerged(
        config.CellBAuto.class, "auto_poses.cell_b." + side(alliance), "auto");
  }

  static config.ParkOnlyAuto parkOnlyPoses(Alliance alliance) {
    return ConfigLoader.loadMerged(
        config.ParkOnlyAuto.class, "auto_poses.park_only." + side(alliance), "auto");
  }

  /**
   * Mirrors {@code CellAAuto.buildPaths()}; disabling collection still follows the same safe lane.
   * With {@code auto.cell_a_second_volley} off the routine does not follow a fixed path from
   * score_a to the park pose (a straight line clips the HIVE frame): it follows {@code
   * Sentinel.parkRoute(score_a, park)}, which tests check separately.
   */
  static Routine cellA(Alliance alliance) {
    config.CellAAuto p = cellAPoses(alliance);
    Map<String, Path> paths = new LinkedHashMap<>();
    paths.put("start->score_a", pline(p.start, p.scoreA));
    paths.put("score_a->flower", pline(p.scoreA, p.flower));
    paths.put("flower->score_b", pcurve(p.flower, p.scoreBCp, p.scoreB));
    paths.put("score_b->park", pline(p.scoreB, p.park));
    return new Routine(
        "cell_a",
        alliance,
        p.start,
        p.park,
        paths,
        List.of(
            new Volley("score_a", HiveCell.A, p.scoreA),
            new Volley("score_b", HiveCell.B, p.scoreB)));
  }

  /** Mirrors {@code CellBAuto.buildPaths()}, including the alt_via/alt_a fallback legs. */
  static Routine cellB(Alliance alliance) {
    config.CellBAuto p = cellBPoses(alliance);
    Map<String, Path> paths = new LinkedHashMap<>();
    paths.put("start->score_b", pcurve(p.start, p.scoreBCp, p.scoreB));
    paths.put("score_b->flower", pline(p.scoreB, p.flower));
    paths.put("flower->score_b", pline(p.flower, p.scoreB));
    paths.put("score_b->alt_via", pline(p.scoreB, p.altVia));
    paths.put("alt_via->alt_a", pline(p.altVia, p.altA));
    paths.put("score_b->park", pline(p.scoreB, p.park));
    paths.put("alt_a->park", pline(p.altA, p.park));
    return new Routine(
        "cell_b",
        alliance,
        p.start,
        p.park,
        paths,
        List.of(
            new Volley("score_b", HiveCell.B, p.scoreB), new Volley("alt_a", HiveCell.A, p.altA)));
  }

  /** Mirrors {@code ParkOnlyAuto.buildPaths()}. */
  static Routine parkOnly(Alliance alliance) {
    config.ParkOnlyAuto p = parkOnlyPoses(alliance);
    Map<String, Path> paths = new LinkedHashMap<>();
    paths.put("start->park", pline(p.start, p.park));
    return new Routine("park_only", alliance, p.start, p.park, paths, List.of());
  }

  static List<Routine> routines(Alliance alliance) {
    return List.of(cellA(alliance), cellB(alliance), parkOnly(alliance));
  }

  /** The TeleOp hold points that shoot into CELL A and CELL B. */
  static List<Volley> teleopVolleys(Alliance alliance) {
    MatchProfile profile = MatchProfile.loadMatchProfile(alliance);
    return List.of(
        new Volley("teleop shoot_a", HiveCell.A, profile.shootAPose()),
        new Volley("teleop shoot_b", HiveCell.B, profile.shootBPose()));
  }

  /** Every volley pose in the codebase for {@code alliance}: each routine's plus TeleOp's. */
  static List<Volley> allVolleys(Alliance alliance) {
    List<Volley> all = new ArrayList<>();
    for (Routine routine : routines(alliance)) {
      for (Volley v : routine.volleys()) {
        all.add(new Volley(routine.name() + " " + v.name(), v.cell(), v.pose()));
      }
    }
    all.addAll(teleopVolleys(alliance));
    return all;
  }
}
