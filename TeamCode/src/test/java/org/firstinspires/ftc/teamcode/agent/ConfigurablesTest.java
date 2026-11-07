package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.tests.ShooterVelocityTunerOpMode;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/** {@link Configurables}: Panels' static fields, read and set by key. */
class ConfigurablesTest {

  enum Motors {
    LEFT,
    RIGHT,
    BOTH
  }

  /** The kinds of field the repo's @Configurable OpModes have. */
  static final class Tuner {
    public static double targetRPM = 870.0;
    public static int frames = 5;
    public static boolean forward = true;
    public static Double power = 1.0;
    public static String name = "shooter";
    public static Motors enabled = Motors.BOTH;
    public static Gains gains = new Gains();
  }

  static final class Gains {
    public double p = 0.01;
    public final double locked = 1.0;
  }

  /** Same simple name as {@link Tuner}, to make "Tuner.frames" ambiguous. */
  static final class Twin {
    static final class Tuner {
      public static int frames = 9;
    }
  }

  private final List<Class<?>> refreshed = new ArrayList<>();

  private static List<Field> fieldsOf(Class<?>... classes) {
    List<Field> out = new ArrayList<>();
    for (Class<?> c : classes) out.addAll(List.of(c.getFields()));
    return out;
  }

  private Configurables of(Class<?>... classes) {
    return new Configurables(() -> fieldsOf(classes), refreshed::add);
  }

  @AfterEach
  void restore() {
    Tuner.targetRPM = 870.0;
    Tuner.frames = 5;
    Tuner.forward = true;
    Tuner.power = 1.0;
    Tuner.name = "shooter";
    Tuner.enabled = Motors.BOTH;
    Tuner.gains = new Gains();
    ShooterVelocityTunerOpMode.targetRPM = 870.0;
  }

  @Test
  void setsEachKindOfFieldAndRefreshesPanels() {
    Configurables c = of(Tuner.class);
    Configurables.Change ch = c.set("Tuner.targetRPM", "1300");
    assertEquals(1300.0, Tuner.targetRPM);
    assertEquals(870.0, ch.before().getAsDouble());
    assertEquals(1300.0, ch.after().getAsDouble());
    assertEquals(List.of(Tuner.class), refreshed, "Panels shows the new value");

    c.set("Tuner.frames", "8");
    c.set("Tuner.forward", "false");
    c.set("Tuner.power", "0.5");
    c.set("Tuner.name", "\"shooter2\"");
    c.set("Tuner.enabled", "\"LEFT\"");
    assertEquals(8, Tuner.frames);
    assertEquals(false, Tuner.forward);
    assertEquals(0.5, Tuner.power);
    assertEquals("shooter2", Tuner.name);
    assertEquals(Motors.LEFT, Tuner.enabled);
  }

  @Test
  void reachesIntoObjectFields() {
    Configurables c = of(Tuner.class);
    c.set("Tuner.gains.p", "0.02");
    assertEquals(0.02, Tuner.gains.p);
    assertEquals(0.02, c.get("Tuner.gains").getAsJsonObject().get("p").getAsDouble());
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.gains.locked", "2"));
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.gains.nope", "2"));
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.gains..p", "2"));
  }

  @Test
  void rejectsWrongTypesUnknownKeysAndBadEnumNames() {
    Configurables c = of(Tuner.class);
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.frames", "1.5"));
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.targetRPM", "true"));
    assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.targetRPM", "1e999"));
    IllegalArgumentException e =
        assertThrows(IllegalArgumentException.class, () -> c.set("Tuner.enabled", "\"UP\""));
    assertTrue(e.getMessage().contains("BOTH"), "the error lists the options: " + e.getMessage());
    assertThrows(IllegalArgumentException.class, () -> c.get("Tuner.nope"));
    assertThrows(IllegalArgumentException.class, () -> c.get("Nope.targetRPM"));
    assertEquals(870.0, Tuner.targetRPM, "a refused set changes nothing");
  }

  @Test
  void anAmbiguousSimpleNameNeedsTheFullClassName() {
    Configurables c = of(Tuner.class, Twin.Tuner.class);
    assertThrows(IllegalArgumentException.class, () -> c.get("Tuner.frames"));
    assertEquals(9, c.get(Twin.Tuner.class.getName() + ".frames").getAsInt());
    assertEquals(870.0, c.get("Tuner.targetRPM").getAsDouble(), "unique fields stay short");
  }

  @Test
  void listFiltersAndDescribesTypes() {
    JsonArray all = of(Tuner.class).list(null);
    assertEquals(7, all.size());
    JsonArray rpm = of(Tuner.class).list("targetrpm");
    assertEquals(1, rpm.size());
    JsonObject o = rpm.get(0).getAsJsonObject();
    assertEquals("Tuner.targetRPM", o.get("key").getAsString());
    assertEquals("number", o.get("type").getAsString());
    JsonObject en = of(Tuner.class).list("enabled").get(0).getAsJsonObject();
    assertEquals("enum", en.get("type").getAsString());
    assertEquals(3, en.get("options").getAsJsonArray().size());
  }

  @Test
  void worksOnTheRealShooterVelocityTuner() {
    Configurables c = of(ShooterVelocityTunerOpMode.class);
    c.set("ShooterVelocityTunerOpMode.targetRPM", "1300");
    assertEquals(1300.0, ShooterVelocityTunerOpMode.targetRPM);
  }
}
