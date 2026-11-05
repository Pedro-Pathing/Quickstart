package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import java.io.InputStream;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.yaml.snakeyaml.Yaml;

public final class ConfigLoader {
  private static final String RELATIVE_EXTERNAL_PATH = "FIRST/teamcode/config.yaml";
  private static final String CLASSPATH_RESOURCE =
      "/org/firstinspires/ftc/teamcode/robot/config/config.yaml";

  private static Map<String, Object> config = new HashMap<>();

  private static String lastLoadedSource = "";
  private static String lastOverrideRejection = null;

  static {
    reload();
  }

  /** Where the active configuration came from, for OpMode telemetry. */
  public static String loadedSource() {
    return lastLoadedSource;
  }

  /** Why the last ADB-pushed override was ignored, or null if none was ignored. */
  public static String overrideRejection() {
    return lastOverrideRejection;
  }

  /**
   * Returns why an ADB-pushed override must not be layered onto the bundled config, or null if it
   * may. An override whose {@code meta.season} differs from (or lacks) the bundled season is a file
   * pushed for a previous game: merging it would silently replace this season's poses, shot tables
   * and field geometry with last season's.
   *
   * <p>An override is also rejected, whole, when any value it sets is not the kind of value the
   * bundled file has there (a number written as {@code "30s"}, a quoted boolean, a section replaced
   * by a scalar) or names a key the bundled file does not have (a typo). Merging those used to load
   * a silent 0 for the number, throw from every OpMode's init for the boolean, and ignore the typo,
   * so a tuning push could quietly zero the AUTO period or stop every OpMode from starting.
   */
  @SuppressWarnings("unchecked")
  public static String overrideRejection(Map<String, Object> bundled, Map<String, Object> override) {
    if (override == null) {
      return null;
    }
    Object bundledSeason = seasonOf(bundled);
    if (bundledSeason == null) {
      return null;
    }
    Object overrideSeason = seasonOf(override);
    if (overrideSeason == null) {
      return "override has no meta.season (expected '" + bundledSeason + "')";
    }
    if (!bundledSeason.toString().equalsIgnoreCase(overrideSeason.toString())) {
      return "override is for season '" + overrideSeason + "', robot code is '" + bundledSeason + "'";
    }
    return firstMismatch(bundled, override, "");
  }

  /** The first override value whose kind differs from the bundled one, as a reason, or null. */
  @SuppressWarnings("unchecked")
  private static String firstMismatch(
      Map<String, Object> bundled, Map<String, Object> override, String path) {
    for (Map.Entry<String, Object> entry : override.entrySet()) {
      String key = path + entry.getKey();
      if (!bundled.containsKey(entry.getKey())) {
        return key + " is not a config key (typo, or from another version of the code)";
      }
      Object base = bundled.get(entry.getKey());
      Object value = entry.getValue();
      String problem = null;
      if (base instanceof Map) {
        problem =
            value instanceof Map
                ? firstMismatch((Map<String, Object>) base, (Map<String, Object>) value, key + ".")
                : key + " must be a section, got " + describe(value);
      } else if (base instanceof Number) {
        problem = value instanceof Number ? null : key + " must be a number, got " + describe(value);
      } else if (base instanceof Boolean) {
        problem =
            value instanceof Boolean ? null : key + " must be true or false, got " + describe(value);
      } else if (base instanceof List || base instanceof String) {
        // Poses are lists, or strings such as "m" (mirror the RED pose); either may replace either.
        if (value instanceof List) {
          for (Object element : (List<Object>) value) {
            if (!(element instanceof Number) && !(element instanceof String)) {
              problem = key + " must list numbers, got " + describe(element);
              break;
            }
          }
        } else if (!(value instanceof String)) {
          problem = key + " must be a list or text, got " + describe(value);
        }
      }
      if (problem != null) {
        return problem;
      }
    }
    return null;
  }

  private static String describe(Object value) {
    return value instanceof String ? "\"" + value + "\"" : String.valueOf(value);
  }

  @SuppressWarnings("unchecked")
  private static Object seasonOf(Map<String, Object> cfg) {
    if (cfg == null || !(cfg.get("meta") instanceof Map)) {
      return null;
    }
    return ((Map<String, Object>) cfg.get("meta")).get("season");
  }

  /**
   * Reloads configuration from the highest-priority available source.
   *
   * <p>Priority: external ADB-pushed file → classpath resource. Call this at the top of {@code
   * OpMode.init()} after pushing a new YAML via ADB.
   */
  public static synchronized void reload() {
    InputStream inputStream = null;
    String loadedSource = null;

    java.util.List<java.io.File> candidates = new java.util.ArrayList<>();
    try {
      java.io.File storageDir = android.os.Environment.getExternalStorageDirectory();
      if (storageDir != null) {
        candidates.add(new java.io.File(storageDir, RELATIVE_EXTERNAL_PATH));
        candidates.add(new java.io.File(storageDir, "FIRST/config.yaml"));
      }
    } catch (Throwable ignored) {
      // android.os.Environment is unmocked in pure JVM unit tests
    }
    candidates.add(new java.io.File("/sdcard/FIRST/teamcode/config.yaml"));
    candidates.add(new java.io.File("/sdcard/FIRST/config.yaml"));
    candidates.add(new java.io.File("/storage/emulated/0/FIRST/teamcode/config.yaml"));
    candidates.add(new java.io.File("/storage/emulated/0/FIRST/config.yaml"));

    for (java.io.File candidate : candidates) {
      if (candidate != null && candidate.exists()) {
        try {
          inputStream = new java.io.FileInputStream(candidate);
          loadedSource = "ADB override: " + candidate.getAbsolutePath();
          break;
        } catch (Exception ignored) {
          // Keep trying remaining candidates
        }
      }
    }

    // The bundled resource always provides the baseline. An ADB-pushed override is layered on
    // top of it rather than replacing it: a pushed file written before a new key was added would
    // otherwise leave that key's whole section null, and the generated facade reads those fields
    // in a static initializer -- so one stale push hard-crashes the robot app with an opaque NPE
    // before any OpMode can run. Merging means a stale or partial override degrades to bundled
    // defaults for whatever it does not mention.
    Map<String, Object> merged = bundledCopy();
    Yaml yaml = new Yaml();

    if (inputStream == null && merged.isEmpty()) {
      throw new RuntimeException("Could not find config.yaml on classpath or external storage");
    }

    if (inputStream != null) {
      try (InputStream is = inputStream) {
        Map<String, Object> override = null;
        String rejection;
        try {
          Object parsed = yaml.load(is);
          if (parsed == null || parsed instanceof Map) {
            @SuppressWarnings("unchecked")
            Map<String, Object> asMap = (Map<String, Object>) parsed;
            override = asMap;
            rejection = overrideRejection(merged, override);
          } else {
            rejection = "override is not a YAML mapping";
          }
        } catch (RuntimeException e) {
          // A syntax error in a pushed file must not stop every OpMode from initializing.
          String message = String.valueOf(e.getMessage());
          int newline = message.indexOf('\n');
          rejection =
              "override is not valid YAML: " + (newline < 0 ? message : message.substring(0, newline));
        }
        if (rejection != null) {
          loadedSource =
              "bundled classpath resource: " + CLASSPATH_RESOURCE + " (IGNORED " + loadedSource
                  + ": " + rejection + ")";
          lastOverrideRejection = rejection;
        } else if (override != null) {
          deepMerge(merged, override);
          lastOverrideRejection = null;
        }
      } catch (Exception e) {
        throw new RuntimeException("Failed to load config.yaml from " + loadedSource, e);
      }
    } else {
      loadedSource = "bundled classpath resource: " + CLASSPATH_RESOURCE;
      lastOverrideRejection = null;
    }

    config = merged;
    lastLoadedSource = loadedSource;
    try {
      loadDocs();
    } catch (Throwable ignored) {
    }
    try {
      android.util.Log.i("ConfigLoader", "Config loaded from " + loadedSource);
    } catch (Throwable ignored) {
      System.out.println("ConfigLoader: Config loaded from " + loadedSource);
    }
  }

  /**
   * The bundled config.yaml, parsed on first use: it is packaged with the app and cannot change
   * while it runs, and parsing it on every reload made reloads (once per OpMode init, and around
   * every unit test) cost more than the code they configure.
   */
  private static Map<String, Object> bundledTree;

  /** A fresh, mutable copy of the bundled tree, so a reload never sees earlier changes. */
  @SuppressWarnings("unchecked")
  private static synchronized Map<String, Object> bundledCopy() {
    if (bundledTree == null) {
      try (InputStream bundled = ConfigLoader.class.getResourceAsStream(CLASSPATH_RESOURCE)) {
        Map<String, Object> base = bundled == null ? null : new Yaml().load(bundled);
        bundledTree = base == null ? new HashMap<>() : base;
      } catch (Exception e) {
        throw new RuntimeException("Failed to load bundled config.yaml", e);
      }
    }
    return (Map<String, Object>) deepCopy(bundledTree);
  }

  @SuppressWarnings("unchecked")
  private static Object deepCopy(Object value) {
    if (value instanceof Map) {
      Map<String, Object> copy = new java.util.LinkedHashMap<>();
      for (Map.Entry<String, Object> entry : ((Map<String, Object>) value).entrySet()) {
        copy.put(entry.getKey(), deepCopy(entry.getValue()));
      }
      return copy;
    }
    if (value instanceof List) {
      List<Object> copy = new java.util.ArrayList<>();
      for (Object item : (List<Object>) value) {
        copy.add(deepCopy(item));
      }
      return copy;
    }
    return value;
  }

  /**
   * Recursively layers {@code override} onto {@code base}, mutating {@code base}. Nested maps are
   * merged key by key so an override may specify only the values it wants to change; scalars and
   * lists replace wholesale.
   */
  @SuppressWarnings("unchecked")
  public static void deepMerge(Map<String, Object> base, Map<String, Object> override) {
    for (Map.Entry<String, Object> entry : override.entrySet()) {
      String key = entry.getKey();
      Object overrideValue = entry.getValue();
      Object baseValue = base.get(key);

      if (baseValue instanceof Map && overrideValue instanceof Map) {
        deepMerge((Map<String, Object>) baseValue, (Map<String, Object>) overrideValue);
      } else {
        base.put(key, overrideValue);
      }
    }
  }

  private static class DocConstraint {
    Double min;
    Double max;
  }

  private static Map<String, DocConstraint> docConstraints = new HashMap<>();

  @SuppressWarnings("unchecked")
  private static void loadDocs() {
    docConstraints.clear();
    InputStream is = null;

    java.util.List<java.io.File> candidates = new java.util.ArrayList<>();
    try {
      java.io.File storageDir = android.os.Environment.getExternalStorageDirectory();
      if (storageDir != null) {
        candidates.add(new java.io.File(storageDir, "FIRST/teamcode/config-docs.yaml"));
        candidates.add(new java.io.File(storageDir, "FIRST/config-docs.yaml"));
      }
    } catch (Throwable ignored) {}
    candidates.add(new java.io.File("/sdcard/FIRST/teamcode/config-docs.yaml"));
    candidates.add(new java.io.File("/sdcard/FIRST/config-docs.yaml"));
    candidates.add(new java.io.File("/storage/emulated/0/FIRST/teamcode/config-docs.yaml"));
    candidates.add(new java.io.File("/storage/emulated/0/FIRST/config-docs.yaml"));

    for (java.io.File candidate : candidates) {
      if (candidate != null && candidate.exists()) {
        try {
          is = new java.io.FileInputStream(candidate);
          break;
        } catch (Exception ignored) {}
      }
    }

    if (is == null) {
      // The bundled docs cannot change while the app runs: parse them once.
      if (bundledDocConstraints == null) {
        String docsResource = CLASSPATH_RESOURCE.replace("config.yaml", "config-docs.yaml");
        parseDocs(ConfigLoader.class.getResourceAsStream(docsResource));
        bundledDocConstraints = new HashMap<>(docConstraints);
      } else {
        docConstraints.putAll(bundledDocConstraints);
      }
      return;
    }
    parseDocs(is);
  }

  /** Doc constraints from the bundled config-docs.yaml, once parsed. */
  private static Map<String, DocConstraint> bundledDocConstraints;

  @SuppressWarnings("unchecked")
  private static void parseDocs(InputStream is) {
    if (is != null) {
      try (InputStream stream = is) {
        Yaml yaml = new Yaml();
        Object loaded = yaml.load(stream);
        if (loaded instanceof Map) {
          Map<String, Object> map = (Map<String, Object>) loaded;
          for (Map.Entry<String, Object> entry : map.entrySet()) {
            if (entry.getValue() instanceof Map) {
              Map<String, Object> sub = (Map<String, Object>) entry.getValue();
              DocConstraint doc = new DocConstraint();
              if (sub.get("min") instanceof Number) {
                doc.min = ((Number) sub.get("min")).doubleValue();
              }
              if (sub.get("max") instanceof Number) {
                doc.max = ((Number) sub.get("max")).doubleValue();
              }
              docConstraints.put(entry.getKey().toLowerCase(), doc);
            }
          }
        }
      } catch (Exception ignored) {}
    }
  }

  private static double enforceBounds(String leaf, double val) {
    if (leaf != null && !docConstraints.isEmpty()) {
      DocConstraint doc = docConstraints.get(leaf.toLowerCase());
      if (doc != null) {
        if (doc.min != null && val < doc.min) {
          return doc.min;
        }
        if (doc.max != null && val > doc.max) {
          return doc.max;
        }
      }
    }
    return val;
  }

  private static final String[][] MIRROR_PAIRS = {
    {"red", "blue"}, {"blue", "red"},
    {"high", "low"}, {"low", "high"},
    {"short", "long"}, {"long", "short"},
    {"left", "right"}, {"right", "left"},
    {"front", "back"}, {"back", "front"},
    {"top", "bottom"}, {"bottom", "top"},
    {"near", "far"}, {"far", "near"},
    {"min", "max"}, {"max", "min"},
    {"open", "close"}, {"close", "open"},
    {"in", "out"}, {"out", "in"},
    {"up", "down"}, {"down", "up"}
  };

  private static boolean isMirrorSignal(Object val) {
    if (val instanceof String) {
      String s = ((String) val).trim();
      if (s.equalsIgnoreCase("m") || s.equalsIgnoreCase("mirror") || s.equalsIgnoreCase("k")) {
        return true;
      }
      String lower = s.toLowerCase();
      if (lower.startsWith("m") || lower.startsWith("mirror") || lower.startsWith("k")) {
        String rest;
        if (lower.startsWith("mirror")) {
          rest = lower.substring(6).trim();
        } else {
          rest = lower.substring(1).trim();
        }
        if (rest.isEmpty()) return true;
        if (rest.startsWith("+") || rest.startsWith("-") || rest.startsWith("*") || rest.startsWith("/")) {
          String operandStr = rest.substring(1).trim();
          try {
            Double.parseDouble(operandStr);
            return true;
          } catch (NumberFormatException e) {
            return false;
          }
        }
      }
      if (s.startsWith("+") || s.startsWith("-") || s.startsWith("*") || s.startsWith("/")) {
        String rest = s.substring(1).trim();
        try {
          Double.parseDouble(rest);
          return true;
        } catch (NumberFormatException e) {
          return false;
        }
      }
    }
    return false;
  }

  private static String getMirrorKey(String key) {
    if (key == null || key.isEmpty()) return key;
    String lower = key.toLowerCase();
    for (String[] pair : MIRROR_PAIRS) {
      String name1 = pair[0], name2 = pair[1];
      String[][] variants = new String[][]{
        {name1 + "_", name2 + "_"},
        {"_" + name1, "_" + name2},
        {name1.toUpperCase() + "_", name2.toUpperCase() + "_"},
        {"_" + name1.toUpperCase(), "_" + name2.toUpperCase()}
      };
      for (String[] var : variants) {
        if (lower.contains(var[0])) {
          return lower.replace(var[0], var[1]);
        }
      }
      if (lower.equals(name1)) return name2;
    }
    return key;
  }

  private static String getMirrorPath(String path) {
    if (path == null || path.isEmpty()) return path;
    String[] segments = path.split("\\.");

    // 1. Check path segment replacement
    for (String[] pair : MIRROR_PAIRS) {
      String a = pair[0], m = pair[1];
      boolean matched = false;
      String[] mirrorSegs = new String[segments.length];
      for (int i = 0; i < segments.length; i++) {
        if (segments[i].equalsIgnoreCase(a)) {
          mirrorSegs[i] = m;
          matched = true;
        } else {
          mirrorSegs[i] = segments[i];
        }
      }
      if (matched) {
        return String.join(".", mirrorSegs);
      }
    }

    // 2. Leaf key replacement
    String leaf = segments[segments.length - 1];
    String mirrorLeaf = getMirrorKey(leaf);
    if (!mirrorLeaf.equals(leaf)) {
      segments[segments.length - 1] = mirrorLeaf;
      return String.join(".", segments);
    }

    return path;
  }

  @SuppressWarnings("unchecked")
  private static Object rawLoad(String path) {

    String[] parts = path.split("\\.");
    Object current = config;
    for (String part : parts) {
      if (current instanceof Map) {
        current = ((Map<String, Object>) current).get(part);
      } else {
        return null;
      }
    }
    return current;
  }

  private static Object evaluateMirrorOperation(Object baseVal, Object rawSignal, Class<?> targetType, String leafKey) {
    if (!(rawSignal instanceof String)) {
      return coerce(baseVal, targetType);
    }
    String s = ((String) rawSignal).trim();
    boolean shouldMirrorPose = true; // default: mirror pose unless 'k' (original) is specified
    String expr = s;

    String lower = expr.toLowerCase();
    if (lower.startsWith("k")) {
      shouldMirrorPose = false; // 'k' means keep original un-mirrored value
      if (lower.startsWith("key")) {
        expr = expr.substring(3).trim();
      } else {
        expr = expr.substring(1).trim();
      }
    } else if (lower.startsWith("m")) {
      shouldMirrorPose = true; // 'm' means mirror
      if (lower.startsWith("mirror")) {
        expr = expr.substring(6).trim();
      } else {
        expr = expr.substring(1).trim();
      }
    }

    boolean isPose = (targetType == Pose.class || (baseVal instanceof List && ((List<?>) baseVal).size() == 3));

    if (isPose) {
      Pose pose = (Pose) coerce(baseVal, Pose.class);
      if (pose != null) {
        if (shouldMirrorPose) {
          pose = mirrorPose(pose);
        }
        if (expr.isEmpty()) {
          return pose;
        }
        char op = expr.charAt(0);
        String operandStr = expr.substring(1).trim();
        try {
          double operand = Double.parseDouble(operandStr);
          double nx = pose.x();
          double ny = pose.y();
          switch (op) {
            case '+': nx += operand; ny += operand; break;
            case '-': nx -= operand; ny -= operand; break;
            case '*': nx *= operand; ny *= operand; break;
            case '/': if (operand != 0) { nx /= operand; ny /= operand; } break;
          }
          return new Pose(nx, ny, pose.heading());
        } catch (NumberFormatException e) {
          return pose;
        }
      }
    }

    if (expr.isEmpty()) {
      return coerce(baseVal, targetType);
    }

    char op = expr.charAt(0);
    String operandStr = expr.substring(1).trim();
    double operand;
    try {
      operand = Double.parseDouble(operandStr);
    } catch (NumberFormatException e) {
      return coerce(baseVal, targetType);
    }

    double baseNum = 0.0;
    if (baseVal instanceof Number) {
      baseNum = ((Number) baseVal).doubleValue();
    } else {
      Object numObj = coerce(baseVal, double.class);
      if (numObj instanceof Number) {
        baseNum = ((Number) numObj).doubleValue();
      }
    }

    double result;
    switch (op) {
      case '+':
        result = baseNum + operand;
        break;
      case '-':
        result = baseNum - operand;
        break;
      case '*':
        result = baseNum * operand;
        break;
      case '/':
        result = operand != 0 ? baseNum / operand : baseNum;
        break;
      default:
        result = baseNum;
        break;
    }

    result = enforceBounds(leafKey, result);
    return coerce(result, targetType);
  }

  private static Object resolveMirrorValue(String currentPath, Class<?> targetType, Object rawSignal) {
    String mirrorPath = getMirrorPath(currentPath);
    if (mirrorPath == null || mirrorPath.equals(currentPath)) {
      String mirrorKey = getMirrorKey(currentPath);
      if (mirrorKey != null && !mirrorKey.equals(currentPath)) {
        mirrorPath = mirrorKey;
      }
    }
    if (mirrorPath == null || mirrorPath.equals(currentPath)) {
      return null;
    }
    Object baseVal = rawLoad(mirrorPath);
    if (baseVal == null || isMirrorSignal(baseVal)) {
      return null;
    }
    String[] parts = currentPath.split("\\.");
    String leafKey = parts[parts.length - 1];
    return evaluateMirrorOperation(baseVal, rawSignal, targetType, leafKey);
  }

  @SuppressWarnings("unchecked")
  public static <T> T load(Class<T> type, String path) {
    try {
      String[] parts = path.split("\\.");
      Object current = config;
      for (String part : parts) {
        if (current instanceof Map) {
          current = ((Map<String, Object>) current).get(part);
        } else {
          current = null;
          break;
        }
      }

      if (current instanceof Map) {
        // Nested configuration object: bind each public field from the map.
        T instance = type.getDeclaredConstructor().newInstance();
        Map<String, Object> map = (Map<String, Object>) current;
        for (Field field : type.getFields()) {
          Object val = map.get(field.getName());
          if (val == null) {
            val = map.get(toSnakeCase(field.getName()));
          }
          if (val != null) {
            field.setAccessible(true);
            if (isMirrorSignal(val)) {
              Object resolved = resolveMirrorValue(path + "." + toSnakeCase(field.getName()), field.getType(), val);
              if (resolved != null) {
                field.set(instance, resolved);
              }
            } else {
              field.set(instance, coerce(val, field.getType(), path + "." + toSnakeCase(field.getName())));
            }
          }
        }
        return instance;
      }

      if (current != null) {
        // Leaf node: a scalar, pose, array, or "m" mirror signal.
        if (isMirrorSignal(current)) {
          Object resolved = resolveMirrorValue(path, type, current);
          if (resolved != null) {
            return (T) resolved;
          }
        }
        return (T) coerce(current, type, path);
      }

      return defaultInstance(type);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load configuration into " + type.getName(), e);
    }
  }

  private static <T> T defaultInstance(Class<T> type) {
    try {
      return type.getDeclaredConstructor().newInstance();
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Loads a configuration class by merging multiple YAML paths. Fields are populated from each
   * path in order; later paths fill in fields not set by earlier ones (first path wins).
   */
  @SuppressWarnings("unchecked")
  public static <T> T loadMerged(Class<T> type, String... paths) {
    try {
      T instance = type.getDeclaredConstructor().newInstance();
      for (String path : paths) {
        String[] parts = path.split("\\.");
        Object current = config;
        for (String part : parts) {
          if (current instanceof Map) {
            current = ((Map<String, Object>) current).get(part);
          } else {
            current = null;
            break;
          }
        }
        if (!(current instanceof Map)) continue;
        Map<String, Object> map = (Map<String, Object>) current;
        for (Field field : type.getFields()) {
          // Don't overwrite a field already populated by an earlier path
          field.setAccessible(true);
          if (field.get(instance) != null && !field.getType().isPrimitive()) continue;
          Object val = map.get(field.getName());
          if (val == null) {
            val = map.get(toSnakeCase(field.getName()));
          }
          if (val != null) {
            if (isMirrorSignal(val)) {
              String snakeName = toSnakeCase(field.getName());
              Object resolved = resolveMirrorValue(path + "." + snakeName, field.getType(), val);
              if (resolved == null) {
                String mirrorKey = getMirrorKey(snakeName);
                if (map.containsKey(mirrorKey)) {
                  Object rawMirrorVal = map.get(mirrorKey);
                  if (rawMirrorVal != null && !isMirrorSignal(rawMirrorVal)) {
                    if (field.getType() == Pose.class || (rawMirrorVal instanceof List && ((List<?>) rawMirrorVal).size() == 3)) {
                      Pose basePose = (Pose) coerce(rawMirrorVal, Pose.class);
                      resolved = basePose != null ? mirrorPose(basePose) : null;
                    } else {
                      resolved = coerce(rawMirrorVal, field.getType());
                    }
                  }
                }
              }
              if (resolved != null) {
                field.set(instance, resolved);
              }
            } else {
              field.set(instance, coerce(val, field.getType()));
            }
          }
        }
      }
      return instance;
    } catch (Exception e) {
      throw new RuntimeException("Failed to loadMerged configuration into " + type.getName(), e);
    }
  }

  private static String toSnakeCase(String name) {
    return name.replaceAll("([A-Z])([A-Z][a-z])", "$1_$2")
        .replaceAll("([a-z0-9])([A-Z])", "$1_$2").toLowerCase(java.util.Locale.ROOT);
  }

  /** Field width used when config has no {@code field.size_in}. */
  public static final double DEFAULT_FIELD_SIZE_IN = 141.5;

  /**
   * Maps a pose to the opposite alliance. The BIOBUZZ field is 180-degree rotationally symmetric
   * (RED's LOADING ZONE, GARDEN and initially-upward CELL rotate onto BLUE's), not mirror
   * symmetric, so an {@code m} pose is RED's pose rotated half a turn about the field centre.
   */
  public static Pose allianceTransform(Pose pose) {
    double center = fieldSize() / 2.0;
    return PoseFactory.radians().mirrorAroundPoint(center, center).of(pose.x(), pose.y(), pose.heading());
  }

  /** {@code field.size_in} from the active config, or {@link #DEFAULT_FIELD_SIZE_IN}. */
  public static double fieldSize() {
    Object size = rawLoad("field.size_in");
    return size instanceof Number ? ((Number) size).doubleValue() : DEFAULT_FIELD_SIZE_IN;
  }

  private static Pose mirrorPose(Pose pose) {
    return allianceTransform(pose);
  }

  private static Object coerce(Object value, Class<?> type) {
    return coerce(value, type, "");
  }

  @SuppressWarnings("unchecked")
  private static Object coerce(Object value, Class<?> type, String currentPath) {
    if (type.isInstance(value)) {
      return value;
    }
    if (type == Pose.class && value instanceof List) {
      List<Number> list = (List<Number>) value;
      if (list.size() >= 3) {
        double x = list.get(0).doubleValue();
        double y = list.get(1).doubleValue();
        double heading = list.get(2).doubleValue();
        return new Pose(x, y, Math.toRadians(heading));
      }
    }
    if (type == Pose.class && value instanceof Map) {
      Map<String, Object> m = (Map<String, Object>) value;
      double x = ((Number) Objects.requireNonNull(m.get("x"))).doubleValue();
      double y = ((Number) Objects.requireNonNull(m.get("y"))).doubleValue();
      double heading = ((Number) Objects.requireNonNull(m.get("heading"))).doubleValue();
      return new Pose(x, y, Math.toRadians(heading));
    }
    if (type.getName().equals("com.qualcomm.robotcore.hardware.PIDFCoefficients")
        && value instanceof Map) {
      Map<String, Object> m = (Map<String, Object>) value;
      double p = m.containsKey("p") ? ((Number) Objects.requireNonNull(m.get("p"))).doubleValue() : 0;
      double i = m.containsKey("i") ? ((Number) Objects.requireNonNull(m.get("i"))).doubleValue() : 0;
      double d = m.containsKey("d") ? ((Number) Objects.requireNonNull(m.get("d"))).doubleValue() : 0;
      double f = m.containsKey("f") ? ((Number) Objects.requireNonNull(m.get("f"))).doubleValue() : 0;
      if (type.getName().equals("com.qualcomm.robotcore.hardware.PIDFCoefficients")) {
        try {
          return Class.forName("com.qualcomm.robotcore.hardware.PIDFCoefficients")
              .getConstructor(double.class, double.class, double.class, double.class)
              .newInstance(p, i, d, f);
        } catch (Exception e1) {
          try {
            Class<?> algEnum = Class.forName("com.qualcomm.robotcore.hardware.MotorControlAlgorithm");
            Object defaultAlg = algEnum.getEnumConstants()[0];
            return Class.forName("com.qualcomm.robotcore.hardware.PIDFCoefficients")
                .getConstructor(double.class, double.class, double.class, double.class, algEnum)
                .newInstance(p, i, d, f, defaultAlg);
          } catch (Exception e2) {
            throw new RuntimeException("Failed to instantiate com.qualcomm.robotcore.hardware.PIDFCoefficients", e1);
          }
        }
      }
    }
    if (type == double.class || type == Double.class) {
      if (value instanceof Number) {
        return ((Number) value).doubleValue();
      }
      if (value instanceof String) {
        try {
          return Double.parseDouble(((String) value).trim());
        } catch (NumberFormatException e) {
          return 0.0;
        }
      }
      return 0.0;
    }
    if (type == int.class || type == Integer.class) {
      if (value instanceof Number) {
        return ((Number) value).intValue();
      }
      if (value instanceof String) {
        try {
          return Integer.parseInt(((String) value).trim());
        } catch (NumberFormatException e) {
          return 0;
        }
      }
      return 0;
    }
    if (type == long.class || type == Long.class) {
      if (value instanceof Number) {
        return ((Number) value).longValue();
      }
      if (value instanceof String) {
        try {
          return Long.parseLong(((String) value).trim());
        } catch (NumberFormatException e) {
          return 0L;
        }
      }
      return 0L;
    }
    if (type == float.class || type == Float.class) {
      if (value instanceof Number) {
        return ((Number) value).floatValue();
      }
      if (value instanceof String) {
        try {
          return Float.parseFloat(((String) value).trim());
        } catch (NumberFormatException e) {
          return 0.0f;
        }
      }
      return 0.0f;
    }
    if (type == boolean.class || type == Boolean.class) {
      return value;
    }
    if (type == String.class) {
      return value.toString();
    }
    // Arrays, for any YAML sequence the generator did not type as a Pose (i.e. any length but 3).
    // Without this a double[] field silently received the raw List, the assignment threw, and the
    // enclosing nested-object bind fell through to returning its map — surfacing far away as
    // "cannot set <NestedType> field to LinkedHashMap" rather than pointing at the real key.
    if (type.isArray() && value instanceof List) {
      List<?> list = (List<?>) value;
      Class<?> component = type.getComponentType();
      Object array = Array.newInstance(component, list.size());
      for (int i = 0; i < list.size(); i++) {
        Object element = list.get(i);
        if (component == double.class) {
          Array.setDouble(array, i, element instanceof Number ? ((Number) element).doubleValue() : 0.0);
        } else if (component == int.class) {
          Array.setInt(array, i, element instanceof Number ? ((Number) element).intValue() : 0);
        } else {
          Array.set(array, i, coerce(element, component, currentPath));
        }
      }
      return array;
    }
    // Recursively handle nested configuration objects (e.g. Turret$Orientation, Shooter$Hood)
    if (value instanceof Map && !type.isPrimitive()) {
      try {
        @SuppressWarnings("unchecked")
        Map<String, Object> nestedMap = (Map<String, Object>) value;
        Object nestedInstance = type.getDeclaredConstructor().newInstance();
        for (Field field : type.getFields()) {
          Object fieldVal = nestedMap.get(field.getName());
          String snakeName = toSnakeCase(field.getName());
          if (fieldVal == null) {
            fieldVal = nestedMap.get(snakeName);
          }
          if (fieldVal != null) {
            field.setAccessible(true);
            String childPath = (currentPath != null && !currentPath.isEmpty()) ? currentPath + "." + snakeName : snakeName;
            if (isMirrorSignal(fieldVal)) {
              Object resolved = resolveMirrorValue(childPath, field.getType(), fieldVal);
              if (resolved != null) {
                field.set(nestedInstance, resolved);
              } else {
                field.set(nestedInstance, coerce(fieldVal, field.getType(), childPath));
              }
            } else {
              field.set(nestedInstance, coerce(fieldVal, field.getType(), childPath));
            }
          }
        }
        return nestedInstance;
      } catch (Exception e) {
        // Fall through and return the raw map as a last resort
      }
    }
    return value;
  }
}
