package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * The Panels configurables ({@code public static} non-final fields of {@code @Configurable}
 * classes, the ones the Panels dashboard edits), read and set by key: {@code
 * <SimpleClassName>.<field>}, or the fully qualified class name when two classes share a simple
 * name. A field holding an object can be reached into: {@code Class.field.member}.
 *
 * <p>The field list is the one Panels built when it scanned the app ({@code
 * GlobalConfigurables.jvmFields}), so the agent sees exactly what the dashboard shows; after a set
 * the class is refreshed in Panels so an open dashboard shows the new value.
 *
 * <p>Unlike {@code config}, these statics are not reset when an OpMode initializes: a value set
 * here stays until it is set again or the robot app restarts. An OpMode may also assign some of
 * them in its init (copying from config, for example), overwriting a value set before init.
 */
final class Configurables {

  private static final Gson GSON = new GsonBuilder().serializeSpecialFloatingPointValues().create();

  record Change(String key, JsonElement before, JsonElement after) {}

  private final Supplier<List<Field>> registry;
  private final Consumer<Class<?>> refresh;

  Configurables(Supplier<List<Field>> registry, Consumer<Class<?>> refresh) {
    this.registry = registry;
    this.refresh = refresh;
  }

  /** The configurables Panels found, refreshing its dashboard after each set. */
  static Configurables fromPanels() {
    return new Configurables(Configurables::panelsFields, Configurables::refreshPanels);
  }

  private static List<Field> panelsFields() {
    List<Field> out = new ArrayList<>();
    try {
      for (var fields :
          com.bylazar.configurables.GlobalConfigurables.INSTANCE.getJvmFields().values()) {
        for (var generic : fields) out.add(generic.getReference());
      }
    } catch (Throwable t) { // Panels missing or changed: report an empty list, not a crash
      return out;
    }
    return out;
  }

  private static void refreshPanels(Class<?> cls) {
    try {
      com.bylazar.configurables.PanelsConfigurables.refreshClass(cls);
    } catch (Throwable ignored) {
      // the value is set either way; only the dashboard's display is stale
    }
  }

  static String key(Field f) {
    return f.getDeclaringClass().getSimpleName() + "." + f.getName();
  }

  /** Every configurable whose key contains {@code filter} (case-insensitive), with its value. */
  JsonArray list(@Nullable String filter) {
    JsonArray out = new JsonArray();
    String needle = filter == null ? "" : filter.toLowerCase(Locale.ROOT);
    for (Field f : registry.get()) {
      String key = key(f);
      if (!key.toLowerCase(Locale.ROOT).contains(needle)) continue;
      JsonObject o = new JsonObject();
      o.addProperty("key", key);
      o.addProperty("class", f.getDeclaringClass().getName());
      o.addProperty("type", typeName(f.getType()));
      o.add("value", GSON.toJsonTree(read(f, null)));
      if (f.getType().isEnum()) {
        JsonArray options = new JsonArray();
        for (Object c : f.getType().getEnumConstants()) options.add(((Enum<?>) c).name());
        o.add("options", options);
      }
      out.add(o);
    }
    return out;
  }

  JsonElement get(String key) {
    Target t = resolve(key);
    return GSON.toJsonTree(t.read());
  }

  /** Sets the leaf {@code key} names from a JSON literal and refreshes it in Panels. */
  synchronized Change set(String key, String jsonValue) {
    Target t = resolve(key);
    JsonElement parsed;
    try {
      parsed = new JsonParser().parse(jsonValue);
    } catch (JsonSyntaxException e) {
      throw new IllegalArgumentException("value is not a JSON literal: " + jsonValue, e);
    }
    Object value = ConfigAccess.convert(parsed, t.field.getType(), key);
    JsonElement before = GSON.toJsonTree(t.read());
    t.write(value);
    refresh.accept(t.root);
    return new Change(key, before, GSON.toJsonTree(t.read()));
  }

  // --- resolution ----------------------------------------------------------------------------

  private record Target(Class<?> root, @Nullable Object owner, Field field) {
    @Nullable
    Object read() {
      return Configurables.read(field, owner);
    }

    void write(Object value) {
      try {
        field.set(owner, value);
      } catch (IllegalAccessException e) {
        throw new IllegalStateException(e);
      }
    }
  }

  @Nullable
  private static Object read(Field f, @Nullable Object owner) {
    try {
      f.setAccessible(true);
      return f.get(owner);
    } catch (IllegalAccessException e) {
      throw new IllegalStateException(e);
    }
  }

  private Target resolve(String key) {
    if (key == null || key.isBlank()) throw new IllegalArgumentException("empty configurable key");
    Field root = null;
    String rootKey = "";
    for (Field f : registry.get()) {
      for (String candidate :
          List.of(key(f), f.getDeclaringClass().getName() + "." + f.getName())) {
        boolean matches = key.equals(candidate) || key.startsWith(candidate + ".");
        if (!matches || candidate.length() < rootKey.length()) continue;
        if (candidate.equals(rootKey) && root != null && !root.equals(f)) {
          throw new IllegalArgumentException(
              "'" + candidate + "' is ambiguous: use the class's full name");
        }
        root = f;
        rootKey = candidate;
      }
    }
    if (root == null) {
      throw new IllegalArgumentException(
          "no configurable '" + key + "' (list_configurables shows the keys)");
    }
    root.setAccessible(true);
    Target t = new Target(root.getDeclaringClass(), null, root);
    String rest = key.substring(rootKey.length());
    if (rest.isEmpty()) return t;
    for (String part : rest.substring(1).split("\\.", -1)) {
      if (part.isEmpty()) throw new IllegalArgumentException("empty member name in " + key);
      Object owner = t.read();
      if (owner == null) throw new IllegalArgumentException(key + ": '" + part + "' is on a null");
      Field f;
      try {
        f = owner.getClass().getField(part);
      } catch (NoSuchFieldException e) {
        throw new IllegalArgumentException("no member '" + part + "' in " + key, e);
      }
      if (Modifier.isStatic(f.getModifiers()) || Modifier.isFinal(f.getModifiers())) {
        throw new IllegalArgumentException(key + ": '" + part + "' cannot be set");
      }
      t = new Target(t.root, owner, f);
    }
    return t;
  }

  private static String typeName(Class<?> type) {
    if (type.isEnum()) return "enum";
    if (type == double.class || type == Double.class || type == float.class) return "number";
    if (type == int.class || type == Integer.class || type == long.class) return "integer";
    if (type == boolean.class || type == Boolean.class) return "boolean";
    if (type == String.class) return "string";
    if (type == double[].class) return "number[]";
    return type.getSimpleName();
  }
}
