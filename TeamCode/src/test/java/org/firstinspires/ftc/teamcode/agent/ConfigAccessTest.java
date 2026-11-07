package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

/**
 * {@link ConfigAccess}: live config reads and sets by dotted path. {@code FreshConfigExtension}
 * reloads the bundled config after each test, so the sets here do not leak.
 */
class ConfigAccessTest {

  @Test
  void readsLeavesAndSections() {
    assertEquals(config.shooter.pidf.p, ConfigAccess.get("shooter.pidf.p").getAsDouble());
    assertTrue(ConfigAccess.get("shooter.pidf").isJsonObject());
    assertEquals(config.meta.season, ConfigAccess.get("meta.season").getAsString());
  }

  @Test
  void setsADoubleTheShooterReadsEveryLoop() {
    double before = config.shooter.pidf.p;
    ConfigAccess.Change c = ConfigAccess.set("shooter.pidf.p", "0.0123");
    assertEquals(0.0123, config.shooter.pidf.p);
    assertEquals(before, c.before().getAsDouble());
    assertEquals(0.0123, c.after().getAsDouble());
  }

  @Test
  void setsBooleansAndInts() {
    ConfigAccess.set("agent_bridge.bridge_enabled", "false");
    assertEquals(false, config.agent_bridge.bridge_enabled);

    ConfigAccess.set("agent_bridge.deadman_ms", "1500");
    assertEquals(1500, config.agent_bridge.deadman_ms);
    assertThrows(
        IllegalArgumentException.class, () -> ConfigAccess.set("agent_bridge.deadman_ms", "1.5"));
  }

  @Test
  void setsTheFlattenedStaticCopyToo() {
    ConfigAccess.Change c = ConfigAccess.set("agent_bridge.deadman_ms", "2500");
    assertEquals("AGENT_BRIDGE_DEADMAN_MS", c.mirror());
    assertEquals(2500, config.AGENT_BRIDGE_DEADMAN_MS);
  }

  @Test
  void rejectsWrongTypesNonFiniteAndNonLeaves() {
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf.p", "true"));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf.p", "\"x\""));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf.p", "1e999"));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf.p", "nope{"));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf", "{}"));
  }

  @Test
  void rejectsUnknownKeys() {
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.get("shooter.nope"));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.get("nope"));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.get(""));
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set("shooter.pidf.p.x", "1"));
  }

  @Test
  void setsADoubleArrayLeafWhenTheConfigHasOne() {
    String path = findDoubleArray();
    if (path == null) return; // config has no double[] leaf today
    ConfigAccess.set(path, "[1.5, 2.5]");
    com.google.gson.JsonArray a = ConfigAccess.get(path).getAsJsonArray();
    assertArrayEquals(
        new double[] {1.5, 2.5}, new double[] {a.get(0).getAsDouble(), a.get(1).getAsDouble()});
    assertThrows(IllegalArgumentException.class, () -> ConfigAccess.set(path, "[1.0, null]"));
  }

  private static String findDoubleArray() {
    for (java.lang.reflect.Field section : config.class.getFields()) {
      if (!java.lang.reflect.Modifier.isStatic(section.getModifiers())) continue;
      try {
        Object o = section.get(null);
        if (o == null) continue;
        for (java.lang.reflect.Field f : o.getClass().getFields()) {
          if (f.getType() == double[].class) return section.getName() + "." + f.getName();
        }
      } catch (IllegalAccessException e) {
        return null;
      }
    }
    return null;
  }

  @Test
  void theShooterGainsFlattenedCopyFollowsTheSet() {
    ConfigAccess.Change c = ConfigAccess.set("shooter.pidf.p", "0.01");
    assertEquals("SHOOTER_PIDF_P", c.mirror());
    assertEquals(0.01, config.SHOOTER_PIDF_P);
  }
}
