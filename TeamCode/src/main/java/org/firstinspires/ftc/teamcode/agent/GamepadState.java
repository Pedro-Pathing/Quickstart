package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

/**
 * One gamepad's sticks, triggers and buttons as the agent bridge plays them into an OpMode. Pure
 * data, so scripts can be parsed and checked off the robot; {@link GamepadInjector} turns it into
 * an SDK {@code Gamepad}.
 *
 * <p>JSON form: an object with any of the {@link Axis} keys (numbers) and {@link Button} keys
 * (booleans), in the SDK's lower-case field names ({@code left_stick_y}, {@code a}, {@code
 * dpad_up}, ...). The PlayStation names {@code cross}, {@code circle}, {@code square}, {@code
 * triangle}, {@code share}, {@code options} and {@code ps} are accepted as aliases. Missing keys
 * are neutral. As on a real gamepad, pushing a stick forward makes its {@code _y} negative.
 */
public final class GamepadState {

  /** Analog inputs with their legal range. */
  public enum Axis {
    LEFT_STICK_X(-1f, 1f),
    LEFT_STICK_Y(-1f, 1f),
    RIGHT_STICK_X(-1f, 1f),
    RIGHT_STICK_Y(-1f, 1f),
    LEFT_TRIGGER(0f, 1f),
    RIGHT_TRIGGER(0f, 1f);

    public final float min;
    public final float max;

    Axis(float min, float max) {
      this.min = min;
      this.max = max;
    }

    public String key() {
      return name().toLowerCase(Locale.ROOT);
    }
  }

  /**
   * Digital inputs, in the SDK's Logitech-style names. {@link #bit} is the button's bit in {@link
   * #buttonMask()} and in the recorder's {@code gp*.buttons} columns, fixed so recordings stay
   * readable if the list is reordered.
   */
  public enum Button {
    A(0),
    B(1),
    X(2),
    Y(3),
    DPAD_UP(4),
    DPAD_DOWN(5),
    DPAD_LEFT(6),
    DPAD_RIGHT(7),
    LEFT_BUMPER(8),
    RIGHT_BUMPER(9),
    LEFT_STICK_BUTTON(10),
    RIGHT_STICK_BUTTON(11),
    BACK(12),
    START(13),
    GUIDE(14),
    TOUCHPAD(15);

    public final int bit;

    Button(int bit) {
      this.bit = bit;
    }

    public String key() {
      return name().toLowerCase(Locale.ROOT);
    }
  }

  private static final Map<String, Button> ALIASES =
      Map.of(
          "cross", Button.A,
          "circle", Button.B,
          "square", Button.X,
          "triangle", Button.Y,
          "share", Button.BACK,
          "options", Button.START,
          "ps", Button.GUIDE);

  public static final GamepadState NEUTRAL =
      new GamepadState(new EnumMap<>(Axis.class), EnumSet.noneOf(Button.class));

  /** Non-zero axes only. */
  private final EnumMap<Axis, Float> axes;

  private final EnumSet<Button> pressed;

  private GamepadState(EnumMap<Axis, Float> axes, EnumSet<Button> pressed) {
    this.axes = axes;
    this.pressed = pressed;
  }

  public float axis(Axis axis) {
    Float v = axes.get(axis);
    return v == null ? 0f : v;
  }

  public boolean pressed(Button button) {
    return pressed.contains(button);
  }

  public boolean isNeutral() {
    return equals(NEUTRAL);
  }

  /** Bit {@code ordinal} set for each pressed button: one CSV column for every button. */
  public int buttonMask() {
    int mask = 0;
    for (Button b : pressed) {
      mask |= 1 << b.bit;
    }
    return mask;
  }

  /**
   * Parses the JSON form. Throws {@link IllegalArgumentException} for an unknown key, a value of
   * the wrong type, or an axis outside its range, so a typo in a script never silently plays
   * neutral.
   */
  public static GamepadState fromJson(JsonObject json) {
    EnumMap<Axis, Float> axes = new EnumMap<>(Axis.class);
    EnumSet<Button> pressed = EnumSet.noneOf(Button.class);
    for (Map.Entry<String, JsonElement> e : json.entrySet()) {
      String key = e.getKey().toLowerCase(Locale.ROOT);
      Axis axis = axisFor(key);
      if (axis != null) {
        double v = number(e.getValue(), key);
        if (!(v >= axis.min && v <= axis.max)) {
          throw new IllegalArgumentException(
              key + " must be in [" + axis.min + ", " + axis.max + "], got " + v);
        }
        if (v != 0.0) axes.put(axis, (float) v);
        continue;
      }
      Button button = buttonFor(key);
      if (button == null) {
        throw new IllegalArgumentException("unknown gamepad input '" + e.getKey() + "'");
      }
      if (bool(e.getValue(), key)) {
        pressed.add(button);
      }
    }
    return new GamepadState(axes, pressed);
  }

  /** The JSON form with only the non-neutral inputs. */
  public JsonObject toJson() {
    JsonObject json = new JsonObject();
    for (Map.Entry<Axis, Float> a : axes.entrySet()) {
      json.add(a.getKey().key(), new JsonPrimitive(a.getValue()));
    }
    for (Button b : pressed) {
      json.add(b.key(), new JsonPrimitive(true));
    }
    return json;
  }

  @Nullable
  private static Axis axisFor(String key) {
    for (Axis a : Axis.values()) {
      if (a.key().equals(key)) return a;
    }
    return null;
  }

  @Nullable
  private static Button buttonFor(String key) {
    for (Button b : Button.values()) {
      if (b.key().equals(key)) return b;
    }
    return ALIASES.get(key);
  }

  private static double number(JsonElement v, String key) {
    if (v.isJsonPrimitive() && v.getAsJsonPrimitive().isNumber()) {
      return v.getAsDouble();
    }
    throw new IllegalArgumentException(key + " must be a number");
  }

  private static boolean bool(JsonElement v, String key) {
    if (v.isJsonPrimitive() && v.getAsJsonPrimitive().isBoolean()) {
      return v.getAsBoolean();
    }
    throw new IllegalArgumentException(key + " must be true or false");
  }

  @Override
  public boolean equals(@Nullable Object o) {
    return o instanceof GamepadState other
        && axes.equals(other.axes)
        && pressed.equals(other.pressed);
  }

  @Override
  public int hashCode() {
    return Objects.hash(axes, pressed);
  }

  @Override
  public String toString() {
    return toJson().toString();
  }
}
