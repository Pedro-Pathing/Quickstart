package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import net.jqwik.api.ForAll;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.IntRange;
import net.jqwik.api.constraints.Size;
import org.junit.jupiter.api.Test;

/** {@link GamepadState} parsing and {@link GamepadScript} timing. */
class GamepadScriptTest {

  private static JsonObject json(String s) {
    return new JsonParser().parse(s).getAsJsonObject();
  }

  @Test
  void stateParsesAxesButtonsAndAliases() {
    GamepadState s =
        GamepadState.fromJson(json("{\"left_stick_y\": -0.5, \"cross\": true, \"dpad_up\": true}"));
    assertEquals(-0.5f, s.axis(GamepadState.Axis.LEFT_STICK_Y));
    assertEquals(0f, s.axis(GamepadState.Axis.RIGHT_TRIGGER));
    assertTrue(s.pressed(GamepadState.Button.A));
    assertTrue(s.pressed(GamepadState.Button.DPAD_UP));
    assertFalse(s.pressed(GamepadState.Button.B));
  }

  @Test
  void stateRejectsTyposWrongTypesAndOutOfRange() {
    assertThrows(
        IllegalArgumentException.class, () -> GamepadState.fromJson(json("{\"aa\": true}")));
    assertThrows(IllegalArgumentException.class, () -> GamepadState.fromJson(json("{\"a\": 1}")));
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadState.fromJson(json("{\"left_trigger\": -0.1}")));
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadState.fromJson(json("{\"right_stick_x\": 1.5}")));
  }

  @Test
  void falseButtonsAndZeroAxesAreNeutral() {
    assertTrue(GamepadState.fromJson(json("{\"a\": false, \"left_stick_x\": 0}")).isNeutral());
  }

  @Test
  void jsonRoundTrips() {
    GamepadState s = GamepadState.fromJson(json("{\"right_trigger\": 0.75, \"y\": true}"));
    assertEquals(s, GamepadState.fromJson(s.toJson()));
  }

  @Test
  void buttonMaskUsesEachButtonsFixedBit() {
    GamepadState s = GamepadState.fromJson(json("{\"a\": true, \"touchpad\": true}"));
    assertEquals(
        (1 << GamepadState.Button.A.bit) | (1 << GamepadState.Button.TOUCHPAD.bit), s.buttonMask());
    assertEquals(1 | (1 << 15), s.buttonMask());
  }

  @Test
  void bitsAreDistinctAndFitTheMask() {
    int seen = 0;
    for (GamepadState.Button b : GamepadState.Button.values()) {
      assertTrue(b.bit >= 0 && b.bit < 24, "a float column holds 24 bits exactly");
      assertEquals(0, seen & (1 << b.bit), "duplicate bit " + b.bit);
      seen |= 1 << b.bit;
    }
  }

  @Test
  void scriptHoldsEachStepUntilTheNextAndEndsNeutral() {
    GamepadScript s =
        GamepadScript.fromJson(
            json(
                "{\"pad\": 2, \"end_ms\": 500, \"steps\": [{\"at_ms\": 100, \"state\": {\"a\":"
                    + " true}}, {\"at_ms\": 300, \"state\": {\"b\": true}}]}"),
            1000);
    assertEquals(2, s.pad());
    assertTrue(s.stateAt(0).isNeutral(), "neutral before the first step");
    assertTrue(s.stateAt(100).pressed(GamepadState.Button.A));
    assertTrue(s.stateAt(299).pressed(GamepadState.Button.A));
    assertTrue(s.stateAt(300).pressed(GamepadState.Button.B));
    assertTrue(s.stateAt(499).pressed(GamepadState.Button.B));
    assertTrue(s.stateAt(500).isNeutral(), "neutral at end_ms");
    assertFalse(s.finishedAt(499));
    assertTrue(s.finishedAt(500));
  }

  @Test
  void scriptValidation() {
    long max = 1000;
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadScript.fromJson(json("{\"end_ms\": 10, \"steps\": []}"), max));
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadScript.fromJson(json("{\"steps\": [{\"at_ms\": 0}]}"), max),
        "end_ms is required");
    assertThrows(
        IllegalArgumentException.class,
        () ->
            GamepadScript.fromJson(
                json("{\"pad\": 3, \"end_ms\": 10, \"steps\": [{\"at_ms\": 0}]}"), max));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            GamepadScript.fromJson(
                json("{\"end_ms\": 10, \"steps\": [{\"at_ms\": 5}, {\"at_ms\": 5}]}"), max),
        "at_ms strictly increasing");
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadScript.fromJson(json("{\"end_ms\": 5, \"steps\": [{\"at_ms\": 5}]}"), max),
        "end_ms after the last step");
    assertThrows(
        IllegalArgumentException.class,
        () -> GamepadScript.fromJson(json("{\"end_ms\": 1001, \"steps\": [{\"at_ms\": 0}]}"), max),
        "bounded by max_script_ms");
    IllegalArgumentException e =
        assertThrows(
            IllegalArgumentException.class,
            () ->
                GamepadScript.fromJson(
                    json(
                        "{\"end_ms\": 10, \"steps\": [{\"at_ms\": 0, \"state\": {\"zz\": true}}]}"),
                    max));
    assertTrue(e.getMessage().startsWith("steps[0]"), e.getMessage());
  }

  /**
   * Oracle: the state at t is the floor entry of a map from step time to state, neutral before the
   * first step and from end_ms on.
   */
  @Property
  void stateAtIsTheLatestStepNotAfterT(
      @ForAll @Size(min = 1, max = 12) List<@IntRange(min = 1, max = 400) Integer> gaps,
      @ForAll @IntRange(min = 1, max = 400) int tail,
      @ForAll @IntRange(min = -50, max = 6000) int t) {
    JsonArray steps = new JsonArray();
    TreeMap<Long, GamepadState> oracle = new TreeMap<>();
    long at = 0;
    for (int i = 0; i < gaps.size(); i++) {
      at += gaps.get(i);
      JsonObject state = new JsonObject();
      state.addProperty("left_stick_x", (i % 5) / 5.0);
      state.addProperty("a", i % 2 == 0);
      JsonObject step = new JsonObject();
      step.addProperty("at_ms", at);
      step.add("state", state);
      steps.add(step);
      oracle.put(at, GamepadState.fromJson(state));
    }
    long end = at + tail;
    JsonObject script = new JsonObject();
    script.add("steps", steps);
    script.addProperty("end_ms", end);
    GamepadScript s = GamepadScript.fromJson(script, 100_000);

    Map.Entry<Long, GamepadState> floor = oracle.floorEntry((long) t);
    GamepadState expected =
        t < 0 || t >= end || floor == null ? GamepadState.NEUTRAL : floor.getValue();
    assertEquals(expected, s.stateAt(t));
  }
}
