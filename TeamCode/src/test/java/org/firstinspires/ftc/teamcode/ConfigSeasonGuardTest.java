package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import java.util.HashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.junit.jupiter.api.Test;

/** The stale-season ADB override guard and the alliance transform behind "m" values. */
public class ConfigSeasonGuardTest {

  private static Map<String, Object> withSeason(Object season) {
    Map<String, Object> cfg = new HashMap<>();
    Map<String, Object> meta = new HashMap<>();
    meta.put("season", season);
    cfg.put("meta", meta);
    return cfg;
  }

  @Test
  public void sameSeasonOverrideIsAccepted() {
    assertNull(ConfigLoader.overrideRejection(withSeason("biobuzz"), withSeason("biobuzz")));
    assertNull(ConfigLoader.overrideRejection(withSeason("biobuzz"), withSeason("BIOBUZZ")));
  }

  @Test
  public void lastSeasonsOverrideIsRejected() {
    String why = ConfigLoader.overrideRejection(withSeason("biobuzz"), withSeason("decode"));
    assertNotNull(why);
    assertTrue(why.contains("decode") && why.contains("biobuzz"), why);
  }

  @Test
  public void overrideWithoutSeasonIsRejected() {
    // Every DECODE config.yaml pushed with ./gradlew pushConfig predates meta.season.
    Map<String, Object> decodeFile = new HashMap<>();
    decodeFile.put("shooter", new HashMap<>());
    assertNotNull(ConfigLoader.overrideRejection(withSeason("biobuzz"), decodeFile));
  }

  @Test
  public void noOverrideOrNoBundledSeasonIsNotRejected() {
    assertNull(ConfigLoader.overrideRejection(withSeason("biobuzz"), null));
    assertNull(ConfigLoader.overrideRejection(new HashMap<>(), withSeason("decode")));
  }

  @Test
  public void bundledConfigDeclaresTheSeason() {
    ConfigLoader.reload();
    assertNull(ConfigLoader.overrideRejection(), "bundled config must not reject itself");
    assertEquals("biobuzz", ConfigLoader.load(String.class, "meta.season"));
  }

  @Test
  public void allianceTransformIsAHalfTurnAboutTheFieldCentre() {
    double size = ConfigLoader.fieldSize();
    Pose red = new Pose(58.0, 9.0, Math.toRadians(90));
    Pose blue = ConfigLoader.allianceTransform(red);
    assertEquals(size - 58.0, blue.x(), 1e-9);
    assertEquals(size - 9.0, blue.y(), 1e-9);
    assertEquals(-Math.PI / 2, AngleUnit.normalizeRadians(blue.heading()), 1e-9);

    Pose back = ConfigLoader.allianceTransform(blue);
    assertEquals(red.x(), back.x(), 1e-9);
    assertEquals(red.y(), back.y(), 1e-9);
    assertEquals(
        AngleUnit.normalizeRadians(red.heading()),
        AngleUnit.normalizeRadians(back.heading()),
        1e-9);
  }

  /** A bundled config shaped like the real one: a number, a boolean, a RED pose and a BLUE "m". */
  private static Map<String, Object> bundledShape() {
    Map<String, Object> cfg = withSeason("biobuzz");
    cfg.put("match", new HashMap<>(Map.of("auto_ms", 30000)));
    cfg.put("shooter", new HashMap<>(Map.of("calibrated", false)));
    cfg.put("poses", new HashMap<>(Map.of("red", java.util.List.of(16.0, 99.0, 90), "blue", "m")));
    return cfg;
  }

  private static Map<String, Object> override(String section, String key, Object value) {
    Map<String, Object> cfg = withSeason("biobuzz");
    cfg.put(section, new HashMap<>(Map.of(key, value)));
    return cfg;
  }

  @Test
  public void aWellFormedPartialOverrideIsAccepted() {
    assertNull(ConfigLoader.overrideRejection(bundledShape(), override("match", "auto_ms", 29000)));
    assertNull(
        ConfigLoader.overrideRejection(bundledShape(), override("shooter", "calibrated", true)));
    assertNull(
        ConfigLoader.overrideRejection(
            bundledShape(), override("poses", "blue", java.util.List.of(125.5, 42.5, 270))));
  }

  /** "30s" used to load as 0 ms: AUTO would park on its first loop. */
  @Test
  public void aNumberWrittenAsTextRejectsTheOverride() {
    String why =
        ConfigLoader.overrideRejection(bundledShape(), override("match", "auto_ms", "30s"));
    assertNotNull(why);
    assertTrue(why.contains("match.auto_ms") && why.contains("30s"), why);
  }

  /** A quoted boolean used to throw from every OpMode's init. */
  @Test
  public void aQuotedBooleanRejectsTheOverride() {
    String why =
        ConfigLoader.overrideRejection(bundledShape(), override("shooter", "calibrated", "true"));
    assertNotNull(why);
    assertTrue(why.contains("shooter.calibrated"), why);
  }

  /** A typo'd key used to be ignored silently, so the change "did nothing". */
  @Test
  public void anUnknownKeyRejectsTheOverride() {
    String why = ConfigLoader.overrideRejection(bundledShape(), override("match", "auto_mss", 1));
    assertNotNull(why);
    assertTrue(why.contains("match.auto_mss"), why);
  }

  @Test
  public void replacingASectionWithAValueRejectsTheOverride() {
    Map<String, Object> bad = withSeason("biobuzz");
    bad.put("match", 5);
    assertNotNull(ConfigLoader.overrideRejection(bundledShape(), bad));
  }

  /** pushConfig sends the whole file, so the bundled config pushed unchanged must be accepted. */
  @Test
  @SuppressWarnings("unchecked")
  public void pushingTheBundledFileUnchangedIsAccepted() throws Exception {
    java.io.File file =
        new java.io.File("src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    Map<String, Object> bundled;
    Map<String, Object> pushed;
    try (java.io.InputStream a = new java.io.FileInputStream(file);
        java.io.InputStream b = new java.io.FileInputStream(file)) {
      bundled = (Map<String, Object>) new org.yaml.snakeyaml.Yaml().load(a);
      pushed = (Map<String, Object>) new org.yaml.snakeyaml.Yaml().load(b);
    }
    assertNull(ConfigLoader.overrideRejection(bundled, pushed));
  }
}
