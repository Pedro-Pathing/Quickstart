package org.firstinspires.ftc.teamcode.agent;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A timed sequence of gamepad states for one pad: each step holds from its {@code at_ms} until the
 * next step, and the pad returns to neutral at {@code end_ms}. Ramps are written as many short
 * steps; nothing is interpolated, so what is played is exactly what was written.
 *
 * <p>JSON form: {@code {"pad": 1, "end_ms": 1500, "steps": [{"at_ms": 0, "state": {"a": true}},
 * {"at_ms": 200, "state": {}}]}}.
 */
public final class GamepadScript {

  public record Step(long atMs, GamepadState state) {}

  private final int pad;
  private final List<Step> steps;
  private final long endMs;

  private GamepadScript(int pad, List<Step> steps, long endMs) {
    this.pad = pad;
    this.steps = Collections.unmodifiableList(steps);
    this.endMs = endMs;
  }

  public int pad() {
    return pad;
  }

  public List<Step> steps() {
    return steps;
  }

  public long endMs() {
    return endMs;
  }

  /**
   * The state at {@code tMs} after the script started: the last step at or before it, neutral
   * before the first step, and neutral from {@code end_ms} on.
   */
  public GamepadState stateAt(long tMs) {
    if (tMs < 0 || tMs >= endMs) {
      return GamepadState.NEUTRAL;
    }
    GamepadState current = GamepadState.NEUTRAL;
    for (Step s : steps) {
      if (s.atMs() > tMs) break;
      current = s.state();
    }
    return current;
  }

  public boolean finishedAt(long tMs) {
    return tMs >= endMs;
  }

  /**
   * Parses and checks the JSON form: pad 1 or 2, at least one step, {@code at_ms} non-negative and
   * strictly increasing, {@code end_ms} after the last step and at most {@code maxMs}.
   */
  public static GamepadScript fromJson(JsonObject json, long maxMs) {
    int pad = json.has("pad") ? json.get("pad").getAsInt() : 1;
    if (pad != 1 && pad != 2) {
      throw new IllegalArgumentException("pad must be 1 or 2");
    }
    JsonElement stepsJson = json.get("steps");
    if (stepsJson == null || !stepsJson.isJsonArray() || stepsJson.getAsJsonArray().size() == 0) {
      throw new IllegalArgumentException("steps must be a non-empty array");
    }
    if (!json.has("end_ms")) {
      throw new IllegalArgumentException("end_ms is required: when the pad returns to neutral");
    }
    long endMs = json.get("end_ms").getAsLong();

    List<Step> steps = new ArrayList<>();
    long previous = -1;
    JsonArray array = stepsJson.getAsJsonArray();
    for (int i = 0; i < array.size(); i++) {
      JsonObject step = array.get(i).getAsJsonObject();
      if (!step.has("at_ms")) {
        throw new IllegalArgumentException("steps[" + i + "] needs at_ms");
      }
      long at = step.get("at_ms").getAsLong();
      if (at < 0 || at <= previous) {
        throw new IllegalArgumentException(
            "steps[" + i + "].at_ms must be >= 0 and greater than the previous step's");
      }
      JsonElement state = step.get("state");
      GamepadState parsed;
      try {
        parsed =
            state == null ? GamepadState.NEUTRAL : GamepadState.fromJson(state.getAsJsonObject());
      } catch (IllegalArgumentException e) {
        throw new IllegalArgumentException("steps[" + i + "]: " + e.getMessage(), e);
      }
      steps.add(new Step(at, parsed));
      previous = at;
    }
    if (endMs <= previous) {
      throw new IllegalArgumentException("end_ms must be after the last step's at_ms");
    }
    if (endMs > maxMs) {
      throw new IllegalArgumentException(
          "end_ms " + endMs + " exceeds agent_bridge.max_script_ms " + maxMs);
    }
    return new GamepadScript(pad, steps, endMs);
  }
}
