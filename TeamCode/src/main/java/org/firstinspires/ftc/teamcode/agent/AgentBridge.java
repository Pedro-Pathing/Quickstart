package org.firstinspires.ftc.teamcode.agent;

import android.content.Context;
import androidx.annotation.Nullable;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD;
import java.io.File;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * HTTP endpoints for an agent (the {@code tools/robot-mcp} MCP server) on the Robot Controller's
 * own web server, port 8080, under {@code /agent/}.
 *
 * <p>What it can do: list, init, start and stop OpModes; play gamepad scripts into the running
 * OpMode ({@link GamepadInjector}); record every telemetry update ({@link TelemetryTap}); sample
 * all hub hardware independently of the OpMode ({@link HardwareRecorder}); read and set config
 * values in memory ({@link ConfigAccess}). What it never does: write a motor or servo. OpModes
 * remain the only writers (single-writer rule), so every safety layer they run still applies.
 *
 * <p>Safety: {@code agent_bridge.bridge_enabled} gates every endpoint but {@code /agent/status}; an
 * OpMode the bridge started is stopped after {@code agent_bridge.deadman_ms} without a bridge
 * request; gamepad scripts are bounded by {@code agent_bridge.max_script_ms} and end in neutral.
 * The Driver Station's stop button works as always.
 */
public final class AgentBridge {

  private static final String TAG = "AgentBridge";
  private static final String STOP_OPMODE = "$Stop$Robot$";
  private static final Gson GSON =
      new GsonBuilder().serializeSpecialFloatingPointValues().serializeNulls().create();

  private static final int STATE_NONE = 0;
  private static final int STATE_INIT = 1;
  private static final int STATE_RUNNING = 2;

  @Nullable private static volatile OpModeManagerImpl manager;
  @Nullable private static volatile OpMode activeOpMode;
  private static volatile String activeName = "";
  private static volatile int state = STATE_NONE;
  private static volatile long stateSinceMs = System.currentTimeMillis();
  private static volatile boolean startedByBridge;
  private static volatile long lastRequestNanos = System.nanoTime();

  @Nullable private static volatile TelemetryLog telemetry;
  private static final GamepadInjector GAMEPAD = new GamepadInjector(() -> activeOpMode);
  private static final Configurables CONFIGURABLES = Configurables.fromPanels();
  @Nullable private static volatile HardwareRecorder recorder;
  @Nullable private static volatile FlightRecorder flight;

  /** Flights live on the robot under /sdcard/FIRST/flight/, one directory each. */
  private static final File FLIGHT_ROOT = new File(AppUtil.FIRST_FOLDER, "flight");

  private static final ScheduledExecutorService DEADMAN =
      Executors.newSingleThreadScheduledExecutor(
          r -> {
            Thread t = new Thread(r, "AgentBridge-Deadman");
            t.setDaemon(true);
            return t;
          });

  private AgentBridge() {}

  // --- SDK hooks -----------------------------------------------------------------------------

  @OnCreateEventLoop
  public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
    OpModeManagerImpl m = eventLoop.getOpModeManager();
    manager = m;
    m.registerListener(LISTENER);
    synchronized (AgentBridge.class) {
      if (telemetry == null) {
        telemetry = new TelemetryLog((int) config.agent_bridge.telemetry_buffer_entries);
        long period = (long) Math.max(50, Math.min(1000, config.agent_bridge.deadman_ms / 20));
        ScheduledFuture<?> unused =
            DEADMAN.scheduleWithFixedDelay(
                AgentBridge::checkDeadman, period, period, TimeUnit.MILLISECONDS);
      }
    }
    RobotLog.ii(TAG, "attached to the event loop");
  }

  @WebHandlerRegistrar
  public static void attachWebServer(Context context, WebHandlerManager web) {
    Map<String, Route> routes = new HashMap<>();
    routes.put("/agent/status", AgentBridge::status);
    routes.put("/agent/opmodes", s -> listOpModes());
    routes.put("/agent/opmode/init", AgentBridge::initOpMode);
    routes.put("/agent/opmode/start", s -> startOpMode());
    routes.put("/agent/opmode/stop", s -> stopOpMode("agent request"));
    routes.put("/agent/gamepad", AgentBridge::playGamepad);
    routes.put("/agent/gamepad/stop", s -> stopGamepad());
    routes.put("/agent/telemetry", AgentBridge::readTelemetry);
    routes.put("/agent/hardware", s -> hardwareSnapshot());
    routes.put("/agent/record/start", AgentBridge::startRecording);
    routes.put("/agent/record/stop", s -> stopRecording());
    routes.put("/agent/record/csv", s -> recordingCsv());
    routes.put("/agent/config", AgentBridge::configValue);
    routes.put("/agent/configurables", s -> CONFIGURABLES.list(param(s, "filter")));
    routes.put("/agent/configurable", AgentBridge::configurableValue);
    routes.put("/agent/flight/start", AgentBridge::startFlight);
    routes.put("/agent/flight/stop", s -> stopFlight("agent request"));
    routes.put("/agent/flights", s -> listFlights());
    routes.put("/agent/flight/file", AgentBridge::flightFile);
    for (Map.Entry<String, Route> r : routes.entrySet()) {
      web.register(r.getKey(), handler(r.getKey(), r.getValue()));
    }
    RobotLog.ii(TAG, "registered %d endpoints under /agent/", routes.size());
  }

  private static final OpModeManagerNotifier.Notifications LISTENER =
      new OpModeManagerNotifier.Notifications() {
        @Override
        public void onOpModePreInit(OpMode opMode) {
          String name = manager == null ? "" : manager.getActiveOpModeName();
          activeOpMode = opMode;
          activeName = name;
          if (STOP_OPMODE.equals(name)) {
            setState(STATE_NONE);
            return;
          }
          setState(STATE_INIT);
          TelemetryLog log = telemetry;
          if (log != null) TelemetryTap.install(opMode, name, log);
          FlightRecorder f = flight;
          if ((f == null || !f.isRunning()) && config.agent_bridge.flight_auto) {
            try {
              startFlightRecorder(name, true);
            } catch (Exception e) {
              RobotLog.ee(TAG, e, "automatic flight did not start");
            }
          }
          flightEvent("opmode_init", "opmode", name);
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
          if (STOP_OPMODE.equals(activeName)) return;
          setState(STATE_RUNNING);
          flightEvent("opmode_start", "opmode", activeName);
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
          GAMEPAD.abort("OpMode stopped");
          flightEvent("opmode_stop", "opmode", activeName);
          FlightRecorder f = flight;
          if (f != null && f.isRunning() && f.isAutomatic()) f.stop("OpMode stopped");
          activeOpMode = null;
          startedByBridge = false;
          setState(STATE_NONE);
        }
      };

  private static void flightEvent(String what, String key, String value) {
    JsonObject data = new JsonObject();
    data.addProperty(key, value);
    flightEvent(what, data);
  }

  private static void flightEvent(String what, JsonObject data) {
    FlightRecorder f = flight;
    if (f != null && f.isRunning()) f.event(what, data);
  }

  private static void setState(int s) {
    state = s;
    stateSinceMs = System.currentTimeMillis();
  }

  private static void checkDeadman() {
    long limit = (long) config.agent_bridge.deadman_ms;
    if (limit <= 0 || !startedByBridge || state == STATE_NONE) return;
    long idleMs = (System.nanoTime() - lastRequestNanos) / 1_000_000L;
    if (idleMs > limit) {
      RobotLog.ww(TAG, "deadman: no agent request for %d ms, stopping %s", idleMs, activeName);
      stopOpMode("deadman");
    }
  }

  // --- HTTP plumbing -------------------------------------------------------------------------

  /** A route returns JSON, or a {@link NanoHTTPD.Response} for anything else. */
  private interface Route {
    Object handle(NanoHTTPD.IHTTPSession session) throws Exception;
  }

  private static WebHandler handler(String path, Route route) {
    return session -> {
      lastRequestNanos = System.nanoTime();
      try {
        if (!path.equals("/agent/status") && !config.agent_bridge.bridge_enabled) {
          return error(NanoHTTPD.Response.Status.FORBIDDEN, "agent_bridge.bridge_enabled is false");
        }
        Object out = route.handle(session);
        if (out instanceof NanoHTTPD.Response r) return r;
        return NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK, "application/json", GSON.toJson(out));
      } catch (IllegalArgumentException e) {
        return error(NanoHTTPD.Response.Status.BAD_REQUEST, String.valueOf(e.getMessage()));
      } catch (IllegalStateException e) {
        return error(NanoHTTPD.Response.Status.CONFLICT, String.valueOf(e.getMessage()));
      } catch (Exception e) {
        RobotLog.ee(TAG, e, "%s failed", path);
        return error(NanoHTTPD.Response.Status.INTERNAL_ERROR, e.toString());
      }
    };
  }

  private static NanoHTTPD.Response error(NanoHTTPD.Response.Status status, String message) {
    JsonObject e = new JsonObject();
    e.addProperty("error", message);
    return NanoHTTPD.newFixedLengthResponse(status, "application/json", GSON.toJson(e));
  }

  @Nullable
  private static String param(NanoHTTPD.IHTTPSession s, String name) {
    List<String> v = s.getParameters().get(name);
    return v == null || v.isEmpty() ? null : v.get(0);
  }

  private static String required(NanoHTTPD.IHTTPSession s, String name) {
    String v = param(s, name);
    if (v == null) throw new IllegalArgumentException("missing parameter '" + name + "'");
    return v;
  }

  private static void requirePost(NanoHTTPD.IHTTPSession s) {
    if (s.getMethod() != NanoHTTPD.Method.POST) {
      throw new IllegalArgumentException("use POST");
    }
  }

  private static String body(NanoHTTPD.IHTTPSession s) throws Exception {
    Map<String, String> files = new HashMap<>();
    s.parseBody(files);
    String body = files.get("postData");
    if (body == null) throw new IllegalArgumentException("missing JSON body");
    return body;
  }

  private static OpModeManagerImpl manager() {
    OpModeManagerImpl m = manager;
    if (m == null) throw new IllegalStateException("event loop not running");
    return m;
  }

  // --- routes --------------------------------------------------------------------------------

  private static Object status(NanoHTTPD.IHTTPSession s) {
    JsonObject out = new JsonObject();
    out.addProperty("bridge_enabled", config.agent_bridge.bridge_enabled);
    out.addProperty("season", config.meta.season);
    String rejection = ConfigLoader.overrideRejection();
    if (rejection != null) out.addProperty("config_override_rejected", rejection);

    JsonObject op = new JsonObject();
    op.addProperty("name", state == STATE_NONE ? "" : activeName);
    op.addProperty("state", new String[] {"none", "init", "running"}[state]);
    op.addProperty("since_ms", System.currentTimeMillis() - stateSinceMs);
    op.addProperty("started_by_bridge", startedByBridge);
    out.add("opmode", op);

    out.addProperty("deadman_ms", config.agent_bridge.deadman_ms);
    TelemetryLog log = telemetry;
    out.addProperty("telemetry_seq", log == null ? 0 : log.latestSeq());
    out.add("gamepad", GAMEPAD.status());
    HardwareRecorder r = recorder;
    out.add("recorder", r == null ? new JsonObject() : r.status());
    FlightRecorder f = flight;
    JsonObject fl = new JsonObject();
    fl.addProperty("auto", config.agent_bridge.flight_auto);
    if (f != null) {
      fl.addProperty("id", f.id());
      fl.addProperty("recording", f.isRunning());
      fl.addProperty("rows", f.meta().get("rows").getAsLong());
    }
    out.add("flight", fl);
    return out;
  }

  private static Object listOpModes() {
    JsonArray list = new JsonArray();
    for (OpModeMeta meta : RegisteredOpModes.getInstance().getOpModes()) {
      if (meta.flavor == OpModeMeta.Flavor.SYSTEM) continue;
      JsonObject o = new JsonObject();
      o.addProperty("name", meta.name);
      o.addProperty("group", meta.group);
      o.addProperty("flavor", meta.flavor.name());
      list.add(o);
    }
    return list;
  }

  private static Object initOpMode(NanoHTTPD.IHTTPSession s) {
    requirePost(s);
    String name = required(s, "name");
    boolean known =
        RegisteredOpModes.getInstance().getOpModes().stream().anyMatch(m -> m.name.equals(name));
    if (!known) throw new IllegalArgumentException("no OpMode named '" + name + "'");
    if (state != STATE_NONE) {
      throw new IllegalStateException("'" + activeName + "' is active: stop it first");
    }
    manager().initOpMode(name);
    startedByBridge = true;
    RobotLog.ii(TAG, "init %s", name);
    return status(s);
  }

  private static Object startOpMode() {
    if (state != STATE_INIT) throw new IllegalStateException("no OpMode is in INIT");
    manager().startActiveOpMode();
    RobotLog.ii(TAG, "start %s", activeName);
    return okay();
  }

  private static Object stopOpMode(String why) {
    GAMEPAD.abort(why);
    if (state != STATE_NONE) {
      manager().stopActiveOpMode();
      RobotLog.ii(TAG, "stop %s (%s)", activeName, why);
    }
    return okay();
  }

  private static Object playGamepad(NanoHTTPD.IHTTPSession s) throws Exception {
    requirePost(s);
    if (state == STATE_NONE) throw new IllegalStateException("no OpMode is active: init one first");
    GamepadScript script =
        GamepadScript.fromJson(
            new JsonParser().parse(body(s)).getAsJsonObject(),
            (long) config.agent_bridge.max_script_ms);
    GAMEPAD.play(script, (long) config.agent_bridge.script_tick_ms);
    JsonObject e = new JsonObject();
    e.addProperty("pad", script.pad());
    e.addProperty("end_ms", script.endMs());
    e.addProperty("steps", script.steps().size());
    flightEvent("gamepad_script", e);
    return GAMEPAD.status();
  }

  private static Object stopGamepad() {
    GAMEPAD.abort("agent request");
    return GAMEPAD.status();
  }

  private static Object readTelemetry(NanoHTTPD.IHTTPSession s) {
    TelemetryLog log = telemetry;
    if (log == null) throw new IllegalStateException("event loop not running");
    String since = param(s, "since");
    String max = param(s, "max");
    JsonObject out = new JsonObject();
    out.addProperty("latest_seq", log.latestSeq());
    List<TelemetryLog.Entry> entries =
        since == null
            ? (log.latest() == null ? List.of() : List.of(log.latest()))
            : log.since(Long.parseLong(since), max == null ? 500 : Integer.parseInt(max));
    out.add("entries", GSON.toJsonTree(entries));
    return out;
  }

  private static HardwareRecorder newRecorder() {
    return newRecorder(HardwareRecorder.Source.ACTIVE);
  }

  private static HardwareRecorder newRecorder(HardwareRecorder.Source source) {
    return new HardwareRecorder(
        manager().getHardwareMap(),
        new HardwareRecorder.Context(
            () -> activeOpMode, () -> state, () -> telemetry == null ? 0 : telemetry.latestSeq()),
        (int) config.agent_bridge.current_every_n_samples,
        source);
  }

  private static Object hardwareSnapshot() {
    HardwareRecorder r = newRecorder();
    JsonObject out = r.layout();
    out.add("values", r.snapshot());
    return out;
  }

  private static synchronized Object startRecording(NanoHTTPD.IHTTPSession s) {
    requirePost(s);
    HardwareRecorder current = recorder;
    if (current != null && current.isRunning()) {
      throw new IllegalStateException("already recording: stop it first");
    }
    String rate = param(s, "rate_hz");
    double hz = rate == null ? config.agent_bridge.record_rate_hz : Double.parseDouble(rate);
    if (!(hz >= 1 && hz <= 200)) throw new IllegalArgumentException("rate_hz must be in [1, 200]");
    HardwareRecorder r = newRecorder();
    r.start(hz, (int) config.agent_bridge.record_max_rows);
    recorder = r;
    JsonObject out = r.layout();
    out.add("status", r.status());
    return out;
  }

  private static synchronized Object stopRecording() {
    HardwareRecorder r = recorder;
    if (r == null) throw new IllegalStateException("nothing recorded yet");
    r.stop();
    return r.status();
  }

  private static Object recordingCsv() throws Exception {
    HardwareRecorder r = recorder;
    if (r == null || r.table() == null) throw new IllegalStateException("nothing recorded yet");
    StringWriter out = new StringWriter();
    r.table().writeCsv(out);
    return NanoHTTPD.newFixedLengthResponse(
        NanoHTTPD.Response.Status.OK, "text/csv", out.toString());
  }

  private static Object configValue(NanoHTTPD.IHTTPSession s) {
    String key = required(s, "key");
    if (s.getMethod() == NanoHTTPD.Method.GET) {
      JsonObject out = new JsonObject();
      out.addProperty("key", key);
      out.add("value", ConfigAccess.get(key));
      return out;
    }
    requirePost(s);
    ConfigAccess.Change c = ConfigAccess.set(key, required(s, "value"));
    RobotLog.ii(TAG, "config %s: %s -> %s", key, c.before(), c.after());
    JsonObject e = new JsonObject();
    e.addProperty("key", key);
    e.add("before", c.before());
    e.add("after", c.after());
    flightEvent("config_set", e);
    return c;
  }

  private static Object configurableValue(NanoHTTPD.IHTTPSession s) {
    String key = required(s, "key");
    if (s.getMethod() == NanoHTTPD.Method.GET) {
      JsonObject out = new JsonObject();
      out.addProperty("key", key);
      out.add("value", CONFIGURABLES.get(key));
      return out;
    }
    requirePost(s);
    Configurables.Change c = CONFIGURABLES.set(key, required(s, "value"));
    RobotLog.ii(TAG, "configurable %s: %s -> %s", key, c.before(), c.after());
    JsonObject e = new JsonObject();
    e.addProperty("key", key);
    e.add("before", c.before());
    e.add("after", c.after());
    flightEvent("configurable_set", e);
    return c;
  }

  // --- flight recorder -----------------------------------------------------------------------

  private static synchronized FlightRecorder startFlightRecorder(String label, boolean automatic)
      throws Exception {
    FlightRecorder current = flight;
    if (current != null && current.isRunning()) {
      throw new IllegalStateException("flight " + current.id() + " is recording: stop it first");
    }
    TelemetryLog log = telemetry;
    if (log == null) throw new IllegalStateException("event loop not running");
    FlightRecorder f =
        FlightRecorder.start(
            newRecorder(HardwareRecorder.Source.PASSIVE),
            log,
            FLIGHT_ROOT,
            label,
            automatic,
            config.agent_bridge.flight_poll_hz,
            (long) config.agent_bridge.flight_flush_ms,
            (int) config.agent_bridge.flight_max_rows,
            (int) config.agent_bridge.flight_keep);
    flight = f;
    if (state != STATE_NONE) flightEvent("opmode_active", "opmode", activeName);
    return f;
  }

  private static Object startFlight(NanoHTTPD.IHTTPSession s) throws Exception {
    requirePost(s);
    String label = param(s, "label");
    return startFlightRecorder(label == null ? "agent" : label, false).meta();
  }

  private static synchronized Object stopFlight(String reason) {
    FlightRecorder f = flight;
    if (f == null) throw new IllegalStateException("no flight recorded since the robot started");
    return f.isRunning() ? f.stop(reason) : f.meta();
  }

  private static Object listFlights() throws Exception {
    JsonArray out = new JsonArray();
    FlightRecorder live = flight;
    for (File dir : FlightLog.flights(FLIGHT_ROOT)) {
      JsonObject o = new JsonObject();
      o.addProperty("id", dir.getName());
      File meta = new File(dir, FlightLog.META);
      if (live != null && live.isRunning() && live.id().equals(dir.getName())) {
        o.add("meta", live.meta());
      } else if (meta.isFile()) {
        try (java.io.Reader r =
            new java.io.InputStreamReader(
                new java.io.FileInputStream(meta), java.nio.charset.StandardCharsets.UTF_8)) {
          JsonObject m = new JsonParser().parse(r).getAsJsonObject();
          m.remove("columns");
          m.remove("devices");
          m.remove("skipped");
          o.add("meta", m);
        } catch (RuntimeException e) {
          o.addProperty("meta_error", e.toString());
        }
      }
      o.addProperty("hardware_bytes", new File(dir, FlightLog.HARDWARE).length());
      out.add(o);
    }
    return out;
  }

  /**
   * Part of one flight file from byte {@code offset} (default 0), at most 4 MiB. The response
   * carries {@code X-Next-Offset} and {@code X-File-Size}, so a client can follow a recording
   * flight's growing files by asking again from the next offset.
   */
  private static Object flightFile(NanoHTTPD.IHTTPSession s) throws Exception {
    String id = required(s, "id");
    String name = required(s, "name");
    if (!FlightLog.safeName(id) || !FlightLog.FILES.contains(name)) {
      throw new IllegalArgumentException("bad flight id or file name");
    }
    File file = new File(new File(FLIGHT_ROOT, id), name);
    if (!file.isFile()) throw new IllegalArgumentException("no " + name + " in flight " + id);
    String off = param(s, "offset");
    long offset = off == null ? 0 : Long.parseLong(off);
    long size = file.length();
    if (offset < 0 || offset > size) throw new IllegalArgumentException("offset out of range");
    int length = (int) Math.min(size - offset, 4L << 20);
    byte[] bytes = new byte[length];
    try (java.io.RandomAccessFile raf = new java.io.RandomAccessFile(file, "r")) {
      raf.seek(offset);
      raf.readFully(bytes);
    }
    NanoHTTPD.Response r =
        NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK,
            "text/plain; charset=utf-8",
            new java.io.ByteArrayInputStream(bytes),
            length);
    r.addHeader("X-Next-Offset", Long.toString(offset + length));
    r.addHeader("X-File-Size", Long.toString(size));
    return r;
  }

  private static JsonElement okay() {
    JsonObject o = new JsonObject();
    o.addProperty("ok", true);
    return o;
  }
}
