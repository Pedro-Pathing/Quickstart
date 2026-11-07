package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.LockSupport;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/**
 * Samples every hub-connected device independently of the running OpMode, into a {@link
 * SampleTable}.
 *
 * <p>Each sample sends its own {@code LynxGetBulkInputDataCommand} per hub. {@code
 * LynxModule.getBulkData()} is avoided on purpose: it clears and replaces the hub's bulk cache, so
 * a recorder using it would change the readings the OpMode it watches sees within a loop. The cost
 * of the independent read is one hub transaction per hub per sample, which the OpMode's own
 * commands wait behind; {@code hub<N>.bulk_ms} records it so its effect on the loop is visible.
 *
 * <p>Columns, all from the hub unless noted:
 *
 * <ul>
 *   <li>motors: {@code <name>.pos} and {@code <name>.vel} (ticks and ticks/s, signed as {@code
 *       DcMotorEx} reports them), {@code <name>.power} (the last power the OpMode commanded, cached
 *       by the SDK, no bus read), {@code <name>.amps} (every {@code current_every_n_samples}th
 *       sample, held in between);
 *   <li>servos: {@code <name>.pos}; continuous servos: {@code <name>.power} (last commanded,
 *       cached);
 *   <li>analog inputs: {@code <name>.volts}; digital channels: {@code <name>.state} (raw level);
 *   <li>hubs: {@code hub<N>.bulk_ms}, and with currents {@code hub<N>.volts} and {@code
 *       hub<N>.amps};
 *   <li>context: {@code opmode_state} (0 none, 1 init, 2 running), {@code gp1.*}/{@code gp2.*}
 *       (sticks, triggers and a button bitmask, read from the OpMode's own gamepads, so they show
 *       Driver Station and injected input alike) and {@code telemetry_seq}.
 * </ul>
 *
 * <p>The goBILDA Pinpoint is recorded from its driver's cached fields ({@code <name>.x_mm}, {@code
 * y_mm}, {@code h_rad}, {@code vx_mm_s}, {@code vy_mm_s}, {@code vh_rad_s}, {@code enc_x}, {@code
 * enc_y}, {@code loop_us}, {@code status}): the values the OpMode's last {@code update()} read,
 * never a new I2C read, whose getters can trigger. Other I2C devices are not recorded.
 *
 * <p>{@link Source#PASSIVE} is the flight recorder's mode: the hub values come from each hub's bulk
 * cache, i.e. exactly what the OpMode's own last bulk read returned, at no bus cost, and {@code
 * hub<N>.fresh} is 1 on the first sample after the OpMode refreshed that hub. It needs the OpMode
 * to use bulk caching (AUTO or MANUAL); without it the hub columns stay empty. Currents and hub
 * voltage are not in this mode, since reading them is a hub command.
 */
final class HardwareRecorder {

  /** Where hub readings come from. */
  enum Source {
    /** The recorder's own bulk command each sample: ground truth, one transaction per hub. */
    ACTIVE,
    /** The hub's bulk cache as the OpMode last filled it: what the OpMode saw, free. */
    PASSIVE
  }

  private static final String TAG = "AgentBridge";

  /** What the recorder needs from the bridge besides the hardware map. */
  record Context(Supplier<OpMode> opMode, IntSupplier opModeState, LongSupplier telemetrySeq) {}

  private interface Reader {
    /** Fills its columns of {@code row} from this sample's bulk responses. */
    void read(LynxGetBulkInputDataResponse[] bulk, float[] row, boolean slow);
  }

  private final List<String> columns = new ArrayList<>();
  private final List<Reader> readers = new ArrayList<>();
  private final List<LynxModule> hubs = new ArrayList<>();
  private int[] bulkColumns = new int[0];
  private final JsonArray devices = new JsonArray();
  private final JsonArray skipped = new JsonArray();
  private final Context context;
  private final int currentEvery;
  private final Source source;

  /** PASSIVE: each hub's last-seen bulk cache object, to tell a new OpMode read from the last. */
  @Nullable private Object[] lastSeenBulk = new Object[0];

  private int[] freshColumns = new int[0];

  /** PASSIVE: values whose change marks a new Pinpoint read, one array per Pinpoint. */
  private final List<Pinpoint> pinpoints = new ArrayList<>();

  @Nullable private volatile SampleTable table;
  @Nullable private Thread thread;
  private volatile boolean running;
  @Nullable private volatile String error;
  private long startedAtMs;
  private double rateHz;

  HardwareRecorder(HardwareMap hw, Context context, int currentEvery, Source source) {
    this.context = context;
    this.source = source;
    this.currentEvery = source == Source.PASSIVE ? 0 : currentEvery;
    plan(hw);
  }

  // --- planning ------------------------------------------------------------------------------

  private void plan(HardwareMap hw) {
    hubs.addAll(hw.getAll(LynxModule.class));
    hubs.sort(Comparator.comparingInt(LynxModule::getModuleAddress));

    for (DcMotor m : hw.getAll(DcMotor.class)) {
      String name = nameOf(hw, m);
      int hub = hubIndex(m.getController());
      if (!(m instanceof DcMotorImpl impl) || hub < 0) {
        skip(name, "motor not on a Lynx hub");
        continue;
      }
      int port = impl.getPortNumber();
      int sign = operationalDirection(impl) == DcMotorSimple.Direction.REVERSE ? -1 : 1;
      int pos = column(name + ".pos");
      int vel = column(name + ".vel");
      int power = column(name + ".power");
      int amps = currentEvery > 0 && m instanceof DcMotorEx ? column(name + ".amps") : -1;
      readers.add(
          (bulk, row, slow) -> {
            LynxGetBulkInputDataResponse b = bulk[hub];
            if (b != null) {
              row[pos] = sign * b.getEncoder(port);
              row[vel] = sign * b.getVelocity(port);
            }
            row[power] = (float) m.getPower();
            if (amps >= 0 && slow) {
              row[amps] = (float) ((DcMotorEx) m).getCurrent(CurrentUnit.AMPS);
            }
          });
      device(name, "motor", hub, port);
    }

    for (Servo s : hw.getAll(Servo.class)) {
      String name = nameOf(hw, s);
      int pos = column(name + ".pos");
      readers.add((bulk, row, slow) -> row[pos] = (float) s.getPosition());
      device(
          name,
          "servo",
          hubIndex(s.getController()),
          s instanceof ServoImpl i ? i.getPortNumber() : -1);
    }

    for (CRServo s : hw.getAll(CRServo.class)) {
      String name = nameOf(hw, s);
      int power = column(name + ".power");
      readers.add((bulk, row, slow) -> row[power] = (float) s.getPower());
      device(
          name,
          "crservo",
          hubIndex(s.getController()),
          s instanceof CRServoImpl i ? i.getPortNumber() : -1);
    }

    for (AnalogInput a : hw.getAll(AnalogInput.class)) {
      String name = nameOf(hw, a);
      Object controller = privateField(a, "controller");
      int hub = hubIndex(controller);
      Integer channel = (Integer) privateField(a, "channel");
      if (hub < 0 || channel == null) {
        skip(name, "analog input not on a Lynx hub");
        continue;
      }
      int volts = column(name + ".volts");
      int ch = channel;
      readers.add(
          (bulk, row, slow) -> {
            if (bulk[hub] != null) row[volts] = bulk[hub].getAnalogInput(ch) / 1000f;
          });
      device(name, "analog", hub, ch);
    }

    for (DigitalChannel d : hw.getAll(DigitalChannel.class)) {
      String name = nameOf(hw, d);
      int hub = hubIndex(privateField(d, "controller"));
      Integer channel = (Integer) privateField(d, "channel");
      if (hub < 0 || channel == null) {
        skip(name, "digital channel not on a Lynx hub");
        continue;
      }
      int state = column(name + ".state");
      int ch = channel;
      readers.add(
          (bulk, row, slow) -> {
            if (bulk[hub] != null) row[state] = bulk[hub].getDigitalInput(ch) ? 1f : 0f;
          });
      device(name, "digital", hub, ch);
    }

    for (GoBildaPinpointDriver p : hw.getAll(GoBildaPinpointDriver.class)) {
      String name = nameOf(hw, p);
      Pinpoint pin = Pinpoint.of(p);
      if (pin == null) {
        skip(name, "Pinpoint driver fields not found");
        continue;
      }
      pinpoints.add(pin);
      int base = columns.size();
      for (String c : Pinpoint.COLUMNS) column(name + "." + c);
      readers.add((bulk, row, slow) -> pin.read(row, base));
      device(name, "pinpoint", -1, -1);
    }

    bulkColumns = new int[hubs.size()];
    freshColumns = new int[hubs.size()];
    lastSeenBulk = new Object[hubs.size()];
    for (int h = 0; h < hubs.size(); h++) {
      LynxModule hub = hubs.get(h);
      String prefix = "hub" + hub.getModuleAddress();
      if (source == Source.ACTIVE) {
        bulkColumns[h] = column(prefix + ".bulk_ms"); // filled by sample()
      } else {
        freshColumns[h] = column(prefix + ".fresh"); // filled by sampleIfChanged()
      }
      if (currentEvery > 0) {
        int volts = column(prefix + ".volts");
        int amps = column(prefix + ".amps");
        readers.add(
            (bulk, row, slow) -> {
              if (slow) {
                row[volts] = (float) hub.getInputVoltage(VoltageUnit.VOLTS);
                row[amps] = (float) hub.getCurrent(CurrentUnit.AMPS);
              }
            });
      }
    }

    int state = column("opmode_state");
    int seq = column("telemetry_seq");
    readers.add(
        (bulk, row, slow) -> {
          row[state] = context.opModeState().getAsInt();
          row[seq] = context.telemetrySeq().getAsLong();
        });
    gamepadColumns(1);
    gamepadColumns(2);
  }

  private void gamepadColumns(int pad) {
    String p = "gp" + pad + ".";
    int lx = column(p + "lx");
    int ly = column(p + "ly");
    int rx = column(p + "rx");
    int ry = column(p + "ry");
    int lt = column(p + "lt");
    int rt = column(p + "rt");
    int buttons = column(p + "buttons");
    readers.add(
        (bulk, row, slow) -> {
          OpMode op = context.opMode().get();
          Gamepad g = op == null ? null : pad == 1 ? op.gamepad1 : op.gamepad2;
          if (g == null) return;
          row[lx] = g.left_stick_x;
          row[ly] = g.left_stick_y;
          row[rx] = g.right_stick_x;
          row[ry] = g.right_stick_y;
          row[lt] = g.left_trigger;
          row[rt] = g.right_trigger;
          row[buttons] = buttonMask(g);
        });
  }

  /** Same bits as {@link GamepadState#buttonMask()}. */
  static int buttonMask(Gamepad g) {
    return bit(g.a, GamepadState.Button.A)
        | bit(g.b, GamepadState.Button.B)
        | bit(g.x, GamepadState.Button.X)
        | bit(g.y, GamepadState.Button.Y)
        | bit(g.dpad_up, GamepadState.Button.DPAD_UP)
        | bit(g.dpad_down, GamepadState.Button.DPAD_DOWN)
        | bit(g.dpad_left, GamepadState.Button.DPAD_LEFT)
        | bit(g.dpad_right, GamepadState.Button.DPAD_RIGHT)
        | bit(g.left_bumper, GamepadState.Button.LEFT_BUMPER)
        | bit(g.right_bumper, GamepadState.Button.RIGHT_BUMPER)
        | bit(g.left_stick_button, GamepadState.Button.LEFT_STICK_BUTTON)
        | bit(g.right_stick_button, GamepadState.Button.RIGHT_STICK_BUTTON)
        | bit(g.back, GamepadState.Button.BACK)
        | bit(g.start, GamepadState.Button.START)
        | bit(g.guide, GamepadState.Button.GUIDE)
        | bit(g.touchpad, GamepadState.Button.TOUCHPAD);
  }

  private static int bit(boolean pressed, GamepadState.Button b) {
    return pressed ? 1 << b.bit : 0;
  }

  private int column(String name) {
    columns.add(name);
    return columns.size() - 1;
  }

  private void device(String name, String kind, int hub, int port) {
    JsonObject d = new JsonObject();
    d.addProperty("name", name);
    d.addProperty("kind", kind);
    if (hub >= 0) d.addProperty("hub", hubs.get(hub).getModuleAddress());
    if (port >= 0) d.addProperty("port", port);
    devices.add(d);
  }

  private void skip(String name, String why) {
    JsonObject d = new JsonObject();
    d.addProperty("name", name);
    d.addProperty("reason", why);
    skipped.add(d);
  }

  private int hubIndex(@Nullable Object controller) {
    if (!(controller instanceof LynxController lc)) return -1;
    for (int i = 0; i < hubs.size(); i++) {
      if (hubs.get(i).getSerialNumber().equals(lc.getSerialNumber())) return i;
    }
    return -1;
  }

  private static String nameOf(HardwareMap hw, HardwareDevice d) {
    Set<String> names = hw.getNamesOf(d);
    return names.isEmpty() ? d.getDeviceName() : names.iterator().next();
  }

  private static DcMotorSimple.Direction operationalDirection(DcMotorImpl m) {
    try {
      Method get = DcMotorImpl.class.getDeclaredMethod("getOperationalDirection");
      get.setAccessible(true);
      return (DcMotorSimple.Direction) get.invoke(m);
    } catch (ReflectiveOperationException e) {
      return m.getDirection();
    }
  }

  @Nullable
  private static Object privateField(Object owner, String name) {
    for (Class<?> c = owner.getClass(); c != null; c = c.getSuperclass()) {
      try {
        Field f = c.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(owner);
      } catch (NoSuchFieldException ignored) {
        // keep looking in the superclass
      } catch (IllegalAccessException e) {
        return null;
      }
    }
    return null;
  }

  /** Cached-field reader for a goBILDA Pinpoint: never triggers an I2C read. */
  private static final class Pinpoint {
    static final String[] COLUMNS = {
      "x_mm",
      "y_mm",
      "h_rad",
      "vx_mm_s",
      "vy_mm_s",
      "vh_rad_s",
      "enc_x",
      "enc_y",
      "loop_us",
      "status"
    };
    private static final String[] FIELDS = {
      "xPosition",
      "yPosition",
      "hOrientation",
      "xVelocity",
      "yVelocity",
      "hVelocity",
      "xEncoderValue",
      "yEncoderValue",
      "loopTime",
      "deviceStatus"
    };

    private final GoBildaPinpointDriver driver;
    private final Field[] fields;
    private final float[] last;

    private Pinpoint(GoBildaPinpointDriver driver, Field[] fields) {
      this.driver = driver;
      this.fields = fields;
      this.last = new float[fields.length];
      Arrays.fill(last, Float.NaN);
    }

    @Nullable
    static Pinpoint of(GoBildaPinpointDriver driver) {
      Field[] fields = new Field[FIELDS.length];
      try {
        for (int i = 0; i < FIELDS.length; i++) {
          fields[i] = GoBildaPinpointDriver.class.getDeclaredField(FIELDS[i]);
          fields[i].setAccessible(true);
        }
      } catch (NoSuchFieldException e) {
        return null;
      }
      return new Pinpoint(driver, fields);
    }

    /** Copies the cached values into {@code row} from {@code base}; true if any changed. */
    boolean read(float[] row, int base) {
      boolean changed = false;
      for (int i = 0; i < fields.length; i++) {
        float v;
        try {
          v = ((Number) fields[i].get(driver)).floatValue();
        } catch (IllegalAccessException e) {
          v = Float.NaN;
        }
        if (Float.compare(v, last[i]) != 0) changed = true;
        last[i] = v;
        row[base + i] = v;
      }
      return changed;
    }

    /** True if the driver's cache differs from the last {@link #read}. */
    boolean changedSinceRead() {
      for (int i = 0; i < fields.length; i++) {
        try {
          float v = ((Number) fields[i].get(driver)).floatValue();
          if (Float.compare(v, last[i]) != 0) return true;
        } catch (IllegalAccessException e) {
          return false;
        }
      }
      return false;
    }
  }

  // --- sampling ------------------------------------------------------------------------------

  @Nullable private static volatile Field lastBulkDataField;
  @Nullable private static volatile Field bulkResponseField;

  /** The hub's bulk cache as the OpMode last filled it, or null without caching. */
  @Nullable
  private static Object cachedBulk(LynxModule hub) {
    try {
      Field f = lastBulkDataField;
      if (f == null) {
        f = LynxModule.class.getDeclaredField("lastBulkData");
        f.setAccessible(true);
        lastBulkDataField = f;
      }
      return f.get(hub);
    } catch (ReflectiveOperationException e) {
      return null;
    }
  }

  @Nullable
  private static LynxGetBulkInputDataResponse responseOf(@Nullable Object bulkData) {
    if (bulkData == null) return null;
    try {
      Field f = bulkResponseField;
      if (f == null) {
        f = bulkData.getClass().getDeclaredField("resp");
        f.setAccessible(true);
        bulkResponseField = f;
      }
      return (LynxGetBulkInputDataResponse) f.get(bulkData);
    } catch (ReflectiveOperationException | ClassCastException e) {
      return null;
    }
  }

  /**
   * PASSIVE mode: a row if the OpMode read anything new since the last call (a hub's bulk cache was
   * replaced, or a Pinpoint updated), else null. Reads no hardware.
   */
  @Nullable
  @SuppressWarnings("ReferenceEquality") // a new cache object is exactly "the OpMode read again"
  float[] sampleIfChanged() {
    Object[] seen = lastSeenBulk;
    if (seen == null) return null;
    Object[] now = new Object[hubs.size()];
    boolean changed = false;
    for (int h = 0; h < hubs.size(); h++) {
      now[h] = cachedBulk(hubs.get(h));
      if (now[h] != seen[h]) changed = true;
    }
    if (!changed) {
      for (Pinpoint p : pinpoints) {
        if (p.changedSinceRead()) {
          changed = true;
          break;
        }
      }
    }
    if (!changed) return null;

    float[] row = new float[columns.size()];
    Arrays.fill(row, Float.NaN);
    LynxGetBulkInputDataResponse[] bulk = new LynxGetBulkInputDataResponse[hubs.size()];
    for (int h = 0; h < hubs.size(); h++) {
      bulk[h] = responseOf(now[h]);
      row[freshColumns[h]] = now[h] != seen[h] && now[h] != null ? 1f : 0f;
      seen[h] = now[h];
    }
    readAll(bulk, row, false);
    return row;
  }

  List<String> columns() {
    return columns;
  }

  /** One sample, not stored: the current reading of every column. */
  JsonObject snapshot() {
    if (source == Source.PASSIVE) throw new IllegalStateException("snapshot needs ACTIVE");
    float[] row = sample(true);
    JsonObject out = new JsonObject();
    for (int i = 0; i < columns.size(); i++) {
      out.addProperty(columns.get(i), Float.isNaN(row[i]) ? null : row[i]);
    }
    return out;
  }

  private float[] sample(boolean slow) {
    float[] row = new float[columns.size()];
    Arrays.fill(row, Float.NaN);
    LynxGetBulkInputDataResponse[] bulk = new LynxGetBulkInputDataResponse[hubs.size()];
    for (int h = 0; h < hubs.size(); h++) {
      long t0 = System.nanoTime();
      try {
        bulk[h] = new LynxGetBulkInputDataCommand(hubs.get(h)).sendReceive();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return row;
      } catch (Exception e) {
        error = "hub " + hubs.get(h).getModuleAddress() + ": " + e;
      }
      row[bulkColumns[h]] = (System.nanoTime() - t0) / 1e6f;
    }
    readAll(bulk, row, slow);
    return row;
  }

  private void readAll(LynxGetBulkInputDataResponse[] bulk, float[] row, boolean slow) {
    for (Reader r : readers) {
      try {
        r.read(bulk, row, slow);
      } catch (RuntimeException e) {
        error = e.toString();
      }
    }
  }

  synchronized void start(double rateHz, int maxRows) {
    if (source == Source.PASSIVE) throw new IllegalStateException("use FlightRecorder");
    if (running) throw new IllegalStateException("already recording");
    this.rateHz = rateHz;
    this.table = new SampleTable(columns, maxRows);
    this.error = null;
    this.startedAtMs = System.currentTimeMillis();
    running = true;
    long periodNanos = (long) (1e9 / rateHz);
    SampleTable t = table;
    thread =
        new Thread(
            () -> {
              long start = System.nanoTime();
              long next = start;
              long n = 0;
              while (running) {
                boolean slow = currentEvery > 0 && n % currentEvery == 0;
                long now = System.nanoTime();
                if (!t.add((now - start) / 1_000_000L, sample(slow))) {
                  running = false;
                  break;
                }
                n++;
                next += periodNanos;
                long wait = next - System.nanoTime();
                if (wait > 0) {
                  LockSupport.parkNanos(wait);
                } else {
                  next = System.nanoTime(); // fell behind: do not burst to catch up
                }
              }
            },
            "AgentBridge-Recorder");
    thread.setDaemon(true);
    thread.start();
    RobotLog.ii(TAG, "recording %d columns at %.0f Hz", columns.size(), rateHz);
  }

  synchronized void stop() {
    running = false;
    Thread t = thread;
    if (t != null) {
      try {
        t.join(1000);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
      thread = null;
    }
  }

  boolean isRunning() {
    return running;
  }

  @Nullable
  SampleTable table() {
    return table;
  }

  JsonObject status() {
    JsonObject s = new JsonObject();
    s.addProperty("recording", running);
    SampleTable t = table;
    if (t != null) {
      s.addProperty("rows", t.size());
      s.addProperty("full", t.isFull());
      s.addProperty("rate_hz", rateHz);
      s.addProperty("started_at_ms", startedAtMs);
    }
    if (error != null) s.addProperty("last_error", error);
    return s;
  }

  JsonObject layout() {
    JsonObject s = new JsonObject();
    JsonArray cols = new JsonArray();
    columns.forEach(cols::add);
    s.add("columns", cols);
    s.add("devices", devices);
    s.add("skipped", skipped);
    return s;
  }
}
