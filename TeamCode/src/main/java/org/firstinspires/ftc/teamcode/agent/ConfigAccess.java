package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Reads and sets values of the loaded {@code config} by dotted path ({@code shooter.pidf.p}), in
 * memory only.
 *
 * <p>A set takes effect wherever code reads {@code config} each loop (for example {@code
 * Shooter.periodic()} reads {@code shooter.pidf.*}, {@code ks} and {@code kick.*} every loop); a
 * value copied into a field at construction changes only after the OpMode is re-initialized, and
 * {@code config.reload()} (which every OpMode's init calls) discards every set. Persist a tuned
 * value by writing it to {@code config.yaml} and running {@code pushConfig}.
 *
 * <p>Range limits live in {@code config-docs.yaml}, which is not on the robot: the MCP server
 * checks them before it calls the bridge. This class checks types and finiteness only.
 */
public final class ConfigAccess {

  private static final Gson GSON = new GsonBuilder().serializeSpecialFloatingPointValues().create();

  /** A completed set: the value before and after, as JSON. */
  public record Change(
      String path, JsonElement before, JsonElement after, @Nullable String mirror) {}

  private ConfigAccess() {}

  /** The value at {@code path} as JSON (a whole section when the path names one). */
  public static JsonElement get(String path) {
    Target t = resolve(path);
    return GSON.toJsonTree(t.read());
  }

  /**
   * Sets the leaf at {@code path} from a JSON literal ({@code 0.5}, {@code 12}, {@code true},
   * {@code "text"}, {@code [1.0, 2.0]}). Leaves of type double, int, long, boolean, String and
   * double[] can be set; sections and Pedro poses cannot. When {@code config} also holds the value
   * as a flattened static ({@code inventory.preload_count} is also {@code
   * INVENTORY_PRELOAD_COUNT}), that copy is set too.
   */
  public static synchronized Change set(String path, String jsonValue) {
    Target t = resolve(path);
    JsonElement parsed;
    try {
      parsed = new JsonParser().parse(jsonValue);
    } catch (JsonSyntaxException e) {
      throw new IllegalArgumentException("value is not a JSON literal: " + jsonValue, e);
    }
    Object value = convert(parsed, t.field.getType(), path);
    JsonElement before = GSON.toJsonTree(t.read());
    t.write(value);

    String mirrorName = path.toUpperCase(Locale.ROOT).replace('.', '_');
    @Nullable String mirror = null;
    try {
      Field m = config.class.getField(mirrorName);
      if (Modifier.isStatic(m.getModifiers()) && m.getType() == t.field.getType()) {
        m.set(null, value);
        mirror = mirrorName;
      }
    } catch (NoSuchFieldException ignored) {
      // Most values have no flattened copy.
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
    return new Change(path, before, GSON.toJsonTree(t.read()), mirror);
  }

  /**
   * A JSON literal as a value of {@code type}: double, int, long, boolean (and their boxes),
   * String, an enum (by constant name) or double[]. Used for config leaves and Panels configurables
   * alike.
   */
  @SuppressWarnings({"unchecked", "rawtypes"}) // Enum.valueOf on a runtime enum class
  static Object convert(JsonElement v, Class<?> type, String path) {
    try {
      if (type.isEnum()) {
        String name = v.getAsString();
        try {
          return Enum.valueOf((Class) type, name);
        } catch (IllegalArgumentException e) {
          throw new IllegalArgumentException(
              path
                  + " is one of "
                  + java.util.Arrays.toString(type.getEnumConstants())
                  + ", got "
                  + name,
              e);
        }
      }
      if (type == double.class || type == Double.class) {
        return finite(v.getAsJsonPrimitive().getAsNumber().doubleValue(), path);
      }
      if (type == float.class || type == Float.class) {
        return (float) finite(v.getAsJsonPrimitive().getAsNumber().doubleValue(), path);
      }
      if (type == int.class || type == Integer.class) {
        double d = v.getAsJsonPrimitive().getAsNumber().doubleValue();
        if (d != Math.rint(d) || Math.abs(d) > Integer.MAX_VALUE) {
          throw new IllegalArgumentException(path + " is an integer, got " + d);
        }
        return (int) d;
      }
      if (type == long.class || type == Long.class) {
        double d = v.getAsJsonPrimitive().getAsNumber().doubleValue();
        if (d != Math.rint(d)) throw new IllegalArgumentException(path + " is an integer");
        return (long) d;
      }
      if (type == boolean.class || type == Boolean.class) {
        if (!v.getAsJsonPrimitive().isBoolean()) {
          throw new IllegalArgumentException(path + " is true or false");
        }
        return v.getAsBoolean();
      }
      if (type == String.class) {
        return v.getAsString();
      }
      if (type == double[].class) {
        JsonArray a = v.getAsJsonArray();
        double[] out = new double[a.size()];
        for (int i = 0; i < out.length; i++) {
          out[i] = finite(a.get(i).getAsDouble(), path + "[" + i + "]");
        }
        return out;
      }
    } catch (IllegalStateException
        | UnsupportedOperationException
        | NumberFormatException
        | ClassCastException e) {
      throw new IllegalArgumentException(
          path + " needs a " + type.getSimpleName() + ", got " + v, e);
    }
    throw new IllegalArgumentException(
        path
            + " is a "
            + type.getSimpleName()
            + "; only number, integer, boolean, string, enum and number-array leaves can be set");
  }

  static double finite(double d, String path) {
    if (!Double.isFinite(d)) throw new IllegalArgumentException(path + " must be finite");
    return d;
  }

  /** The field a path names and the object holding it (null for the static sections). */
  private record Target(@Nullable Object owner, Field field) {
    @Nullable
    Object read() {
      try {
        return field.get(owner);
      } catch (IllegalAccessException e) {
        throw new IllegalStateException(e);
      }
    }

    void write(Object value) {
      try {
        field.set(owner, value);
      } catch (IllegalAccessException e) {
        throw new IllegalStateException(e);
      }
    }
  }

  private static Target resolve(String path) {
    if (path == null || path.isBlank()) {
      throw new IllegalArgumentException("empty config path");
    }
    String[] parts = path.split("\\.");
    Target target = new Target(null, field(config.class, parts[0], path, true));
    for (int i = 1; i < parts.length; i++) {
      Object owner = target.read();
      if (owner == null) {
        throw new IllegalArgumentException(String.join(".", Arrays.copyOf(parts, i)) + " is null");
      }
      target = new Target(owner, field(owner.getClass(), parts[i], path, false));
    }
    return target;
  }

  /** A public, non-final field: static for a top-level section, instance below it. */
  private static Field field(Class<?> type, String name, String path, boolean isStatic) {
    Field f;
    try {
      f = type.getField(name);
    } catch (NoSuchFieldException e) {
      throw new IllegalArgumentException("no config key '" + path + "' (at '" + name + "')", e);
    }
    int mods = f.getModifiers();
    if (Modifier.isStatic(mods) != isStatic || Modifier.isFinal(mods)) {
      throw new IllegalArgumentException("no config key '" + path + "'");
    }
    return f;
  }
}
