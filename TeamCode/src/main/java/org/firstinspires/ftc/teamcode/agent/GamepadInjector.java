package org.firstinspires.ftc.teamcode.agent;

import androidx.annotation.Nullable;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.reflect.Method;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * Plays a {@link GamepadScript} into the running OpMode as if it came from the Driver Station.
 *
 * <p>The SDK delivers Driver Station input by calling the OpMode's package-private {@code
 * newGamepadDataAvailable(gamepad1, gamepad2)} with fresh {@code Gamepad} objects: {@code
 * LinearOpMode} copies them into {@code gamepad1}/{@code gamepad2} at once, an iterative {@code
 * OpMode} copies them at the start of its next loop, and both copies run {@code
 * updateEdgeDetection()}, so {@code gamepad1.aWasPressed()} works. The injector makes the same
 * call, so an unmodified OpMode sees scripted input. The other pad keeps its current state.
 *
 * <p>Injected input holds until the next Driver Station gamepad packet, which replaces it: leave
 * the Driver Station's gamepads unassigned while a script runs. A script always ends by playing
 * neutral, and is aborted (also playing neutral) when the OpMode stops.
 */
final class GamepadInjector {

  private static final String TAG = "AgentBridge";

  private final Supplier<OpMode> activeOpMode;
  private final ScheduledExecutorService executor =
      Executors.newSingleThreadScheduledExecutor(
          r -> {
            Thread t = new Thread(r, "AgentBridge-Gamepad");
            t.setDaemon(true);
            return t;
          });

  @Nullable private GamepadScript script;
  @Nullable private OpMode target;
  private long startNanos;

  /** What the pad last received; null right after a script starts, so its first state is sent. */
  @Nullable private GamepadState lastPlayed = GamepadState.NEUTRAL;

  @Nullable private ScheduledFuture<?> tick;
  @Nullable private String lastError;

  GamepadInjector(Supplier<OpMode> activeOpMode) {
    this.activeOpMode = activeOpMode;
  }

  /** Starts {@code script} against the active OpMode, replacing any running script. */
  synchronized void play(GamepadScript script, long tickMs) {
    OpMode op = activeOpMode.get();
    if (op == null) {
      throw new IllegalStateException("no OpMode is active: init one first");
    }
    stopLocked("replaced");
    this.script = script;
    this.target = op;
    this.startNanos = System.nanoTime();
    this.lastPlayed = null;
    this.lastError = null;
    tick = executor.scheduleAtFixedRate(this::advance, 0, tickMs, TimeUnit.MILLISECONDS);
  }

  /** Stops a running script, playing neutral on its pad. */
  synchronized void abort(String reason) {
    stopLocked(reason);
  }

  @SuppressWarnings("ReferenceEquality") // the same OpMode instance, not an equal one
  private synchronized void advance() {
    GamepadScript script = this.script;
    OpMode target = this.target;
    if (script == null || target == null) return;
    if (activeOpMode.get() != target) {
      stopLocked("OpMode changed");
      return;
    }
    long t = (System.nanoTime() - startNanos) / 1_000_000L;
    GamepadState state = script.stateAt(t);
    try {
      if (!state.equals(lastPlayed)) {
        inject(target, script.pad(), state);
        lastPlayed = state;
      }
    } catch (RuntimeException e) {
      lastError = e.toString();
      RobotLog.ee(TAG, e, "gamepad injection failed");
      stopLocked("error");
      return;
    }
    if (script.finishedAt(t)) {
      stopLocked(null);
    }
  }

  private void stopLocked(@Nullable String reason) {
    if (tick != null) {
      tick.cancel(false);
      tick = null;
    }
    GamepadScript script = this.script;
    OpMode target = this.target;
    GamepadState last = lastPlayed;
    if (script != null && target != null && last != null && !last.isNeutral()) {
      try {
        inject(target, script.pad(), GamepadState.NEUTRAL);
      } catch (RuntimeException e) {
        RobotLog.ee(TAG, e, "could not return the gamepad to neutral");
      }
    }
    if (script != null && reason != null) {
      RobotLog.ii(TAG, "gamepad script stopped: %s", reason);
    }
    this.script = null;
    this.target = null;
    lastPlayed = GamepadState.NEUTRAL;
  }

  synchronized JsonObject status() {
    JsonObject s = new JsonObject();
    GamepadScript script = this.script;
    GamepadState last = lastPlayed;
    s.addProperty("active", script != null);
    if (script != null) {
      s.addProperty("pad", script.pad());
      s.addProperty("t_ms", (System.nanoTime() - startNanos) / 1_000_000L);
      s.addProperty("end_ms", script.endMs());
      s.add("state", (last == null ? GamepadState.NEUTRAL : last).toJson());
    }
    if (lastError != null) s.addProperty("last_error", lastError);
    return s;
  }

  // --- SDK injection -------------------------------------------------------------------------

  @Nullable private static volatile Method newGamepadData;

  private static void inject(OpMode op, int pad, GamepadState state) {
    Gamepad g1 = new Gamepad();
    Gamepad g2 = new Gamepad();
    g1.copy(op.gamepad1);
    g2.copy(op.gamepad2);
    write(pad == 1 ? g1 : g2, state);
    try {
      newGamepadDataMethod().invoke(op, g1, g2);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("SDK gamepad delivery changed: " + e, e);
    }
  }

  private static Method newGamepadDataMethod() throws ReflectiveOperationException {
    Method m = newGamepadData;
    if (m == null) {
      Class<?> internal =
          findClass("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal", OpMode.class);
      m = internal.getDeclaredMethod("newGamepadDataAvailable", Gamepad.class, Gamepad.class);
      m.setAccessible(true);
      newGamepadData = m;
    }
    return m;
  }

  private static Class<?> findClass(String name, Class<?> from) throws ClassNotFoundException {
    for (Class<?> c = from; c != null; c = c.getSuperclass()) {
      if (c.getName().equals(name)) return c;
    }
    throw new ClassNotFoundException(name + " is not a superclass of " + from.getName());
  }

  /** Writes every input, including the PlayStation aliases, so all field names agree. */
  static void write(Gamepad g, GamepadState s) {
    g.left_stick_x = s.axis(GamepadState.Axis.LEFT_STICK_X);
    g.left_stick_y = s.axis(GamepadState.Axis.LEFT_STICK_Y);
    g.right_stick_x = s.axis(GamepadState.Axis.RIGHT_STICK_X);
    g.right_stick_y = s.axis(GamepadState.Axis.RIGHT_STICK_Y);
    g.left_trigger = s.axis(GamepadState.Axis.LEFT_TRIGGER);
    g.right_trigger = s.axis(GamepadState.Axis.RIGHT_TRIGGER);
    g.a = g.cross = s.pressed(GamepadState.Button.A);
    g.b = g.circle = s.pressed(GamepadState.Button.B);
    g.x = g.square = s.pressed(GamepadState.Button.X);
    g.y = g.triangle = s.pressed(GamepadState.Button.Y);
    g.dpad_up = s.pressed(GamepadState.Button.DPAD_UP);
    g.dpad_down = s.pressed(GamepadState.Button.DPAD_DOWN);
    g.dpad_left = s.pressed(GamepadState.Button.DPAD_LEFT);
    g.dpad_right = s.pressed(GamepadState.Button.DPAD_RIGHT);
    g.left_bumper = s.pressed(GamepadState.Button.LEFT_BUMPER);
    g.right_bumper = s.pressed(GamepadState.Button.RIGHT_BUMPER);
    g.left_stick_button = s.pressed(GamepadState.Button.LEFT_STICK_BUTTON);
    g.right_stick_button = s.pressed(GamepadState.Button.RIGHT_STICK_BUTTON);
    g.back = g.share = s.pressed(GamepadState.Button.BACK);
    g.start = g.options = s.pressed(GamepadState.Button.START);
    g.guide = g.ps = s.pressed(GamepadState.Button.GUIDE);
    g.touchpad = s.pressed(GamepadState.Button.TOUCHPAD);
  }
}
