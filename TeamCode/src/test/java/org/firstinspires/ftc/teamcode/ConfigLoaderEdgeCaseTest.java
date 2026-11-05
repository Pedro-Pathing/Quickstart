package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

/**
 * Proves that reflection-based config binding loads non-null instances, binds Pose structures, and
 * resolves match profiles correctly using lowercase {@link config}.
 */
public class ConfigLoaderEdgeCaseTest {

  @Test
  public void deepMerge_staleOverrideKeepsSectionsItDoesNotMention() {
    // Reproduces the failure that hard-crashed the robot app: an ADB-pushed config.yaml written
    // before turret.travel existed. Before the override was merged onto the bundled baseline it
    // replaced it outright, so turret.travel came back null and the generated facade NPE'd inside
    // a static initializer -- killing the app before any OpMode could run.
    java.util.Map<String, Object> bundled = new java.util.HashMap<>();
    java.util.Map<String, Object> bundledTurret = new java.util.HashMap<>();
    java.util.Map<String, Object> bundledTravel = new java.util.HashMap<>();
    bundledTravel.put("min_angle", -45.0);
    bundledTravel.put("max_angle", 45.0);
    bundledTurret.put("travel", bundledTravel);
    bundledTurret.put("ks", 0.08);
    bundled.put("turret", bundledTurret);

    // A stale push: has turret, tweaks one value, knows nothing about travel.
    java.util.Map<String, Object> stale = new java.util.HashMap<>();
    java.util.Map<String, Object> staleTurret = new java.util.HashMap<>();
    staleTurret.put("ks", 0.12);
    stale.put("turret", staleTurret);

    ConfigLoader.deepMerge(bundled, stale);

    @SuppressWarnings("unchecked")
    java.util.Map<String, Object> turret = (java.util.Map<String, Object>) bundled.get("turret");
    assertNotNull(turret.get("travel"), "stale override must not delete unmentioned sections");

    @SuppressWarnings("unchecked")
    java.util.Map<String, Object> travel = (java.util.Map<String, Object>) turret.get("travel");
    assertEquals(-45.0, (Double) travel.get("min_angle"), 1e-9);
    // ...while the value the override *did* specify still wins.
    assertEquals(0.12, (Double) turret.get("ks"), 1e-9);
  }

  @Test
  public void loadMerged_autoPoses_bindsEveryPoseAndGlobalsForEveryRoutineAndAlliance() {
    for (Alliance alliance : Alliance.values()) {
      config.CellAAuto a = AutoFixtures.cellAPoses(alliance);
      assertNotNull(a.start, "start");
      assertNotNull(a.scoreA, "score_a");
      assertNotNull(a.flower, "flower");
      assertNotNull(a.scoreBCp, "score_b_cp (a snake-case key with a trailing letter pair)");
      assertNotNull(a.scoreB, "score_b");
      assertNotNull(a.park, "park");
      // Shared "auto" globals are merged in after the per-routine poses.
      assertEquals(config.auto.tip_wait_ms, a.tipWaitMs);
      assertEquals(config.auto.balls_per_volley_count, a.ballsPerVolleyCount);
      assertEquals(config.auto.cell_a_second_volley, a.cellASecondVolley);

      config.CellBAuto b = AutoFixtures.cellBPoses(alliance);
      assertNotNull(b.start, "start");
      assertNotNull(b.scoreBCp, "score_b_cp");
      assertNotNull(b.scoreB, "score_b");
      assertNotNull(b.flower, "flower");
      assertNotNull(b.altVia, "alt_via");
      assertNotNull(b.altA, "alt_a");
      assertNotNull(b.park, "park");
      assertEquals(config.auto.park_reserve_ms, b.parkReserveMs);

      config.ParkOnlyAuto park = AutoFixtures.parkOnlyPoses(alliance);
      assertNotNull(park.start, "start");
      assertNotNull(park.park, "park");
    }
  }

  @Test
  public void loadPoseByPath_blueStart() {
    Pose p = ConfigLoader.load(Pose.class, "teleop.poses.blue.start");
    assertNotNull(p, "Pose should be loaded");
  }

  @Test
  public void loadDoubleByPath_scalar() {
    Double v = ConfigLoader.load(Double.class, "field.size_in");
    assertNotNull(v, "Scalar value should be loaded");
    assertEquals(config.field.size_in, v, 1e-12);
    assertEquals(
        config.sentinel.robot_width,
        ConfigLoader.load(Double.class, "sentinel.robot_width"),
        1e-12);
  }

  @Test
  public void loadMatchProfile_hasEveryPoseForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      MatchProfile profile = config.loadMatchProfile(alliance);
      assertNotNull(profile, alliance + " MatchProfile should load");
      assertEquals(alliance, profile.alliance());
      assertNotNull(profile.startPose(), "start");
      assertNotNull(profile.parkPose(), "park");
      assertNotNull(profile.shootAPose(), "shoot_a");
      assertNotNull(profile.shootBPose(), "shoot_b");
      assertNotNull(profile.flowerPose(), "flower");
    }
  }

  @Test
  public void matchProfile_shootPoseSelectsThePoseForTheCell() {
    for (Alliance alliance : Alliance.values()) {
      MatchProfile profile = config.loadMatchProfile(alliance);
      assertSame(profile.shootAPose(), profile.shootPose(HiveCell.A));
      assertSame(profile.shootBPose(), profile.shootPose(HiveCell.B));
      assertNotEquals(profile.shootAPose().y(), profile.shootBPose().y(), 1.0);
    }
  }

  @Test
  public void matchProfile_matchesTheMatchProfileHelperOnTheConfigFacade() {
    MatchProfile viaFacade = config.loadMatchProfile(Alliance.BLUE);
    MatchProfile direct = MatchProfile.loadMatchProfile(Alliance.BLUE);
    assertEquals(viaFacade.startPose().x(), direct.startPose().x(), 0.0);
    assertEquals(viaFacade.shootBPose().y(), direct.shootBPose().y(), 0.0);
  }
}
