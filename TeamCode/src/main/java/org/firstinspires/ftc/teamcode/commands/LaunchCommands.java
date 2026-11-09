package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Launching. A launch waits until it can shoot well and then feeds once: no auto-collect is running
 * (one that is has the camera down), the camera is up with a distance (ShotReadiness), and the
 * flywheel is at that distance's RPM. The OpMode sets the flywheels' RPM every loop.
 */
@Configurable
public final class LaunchCommands {
    /** Fire everything: after a feed, each launcher waits this long before its next (for the gate and the next ball). */
    public static double fireAllIntervalMs = 300;

    private LaunchCommands() {
    }

    /** One launcher: its flywheel, how to feed it, whether its gate is open, and its gate's requirement. */
    public static class Launcher {
        final String name;
        final Flywheel flywheel;
        final Runnable feed;
        final BooleanSupplier gateOpen;
        final Object gate;

        public Launcher(String name, Flywheel flywheel, Runnable feed, BooleanSupplier gateOpen, Object gate) {
            this.name = name;
            this.flywheel = flywheel;
            this.feed = feed;
            this.gateOpen = gateOpen;
            this.gate = gate;
        }

        void fire() {
            feed.run();
            flywheel.shotFired();
        }
    }

    /** What every launch waits for apart from its flywheel. */
    public static class Conditions {
        final LimelightCamera camera;
        final ShotReadiness readiness;
        final BooleanSupplier collecting;

        public Conditions(LimelightCamera camera, ShotReadiness readiness, BooleanSupplier collecting) {
            this.camera = camera;
            this.readiness = readiness;
            this.collecting = collecting;
        }

        /** Ask for the camera up (unless a collect has it down) and say whether a shot can go. */
        boolean ready() {
            if (collecting.getAsBoolean()) return false;
            camera.setMode(LimelightCamera.Mode.TAGS);
            return readiness.isReady();
        }
    }

    /** One shot from one launcher, queued until it can go. */
    public static class Launch implements Command {
        private final Launcher launcher;
        private final Conditions conditions;
        private final Consumer<EndCondition> onEnd;
        private boolean fed;

        public Launch(Launcher launcher, Conditions conditions, Consumer<EndCondition> onEnd) {
            this.launcher = launcher;
            this.conditions = conditions;
            this.onEnd = onEnd;
        }

        /** Scheduled but not fed yet. */
        public boolean isWaiting() {
            return isScheduled() && !fed;
        }

        @Override
        public Set<Object> requirements() {
            return Set.of(launcher.gate);
        }

        @Override
        public int priority() {
            return Requirements.ASSIST;
        }

        @Override
        public InterruptedBehavior interruptedBehavior() {
            return InterruptedBehavior.END;
        }

        @Override
        public ConflictBehavior conflictBehavior() {
            return ConflictBehavior.OVERRIDE;
        }

        @Override
        public BlockedBehavior blockedBehavior() {
            return BlockedBehavior.QUEUE;
        }

        @Override
        public void start() {
            fed = false;
        }

        @Override
        public boolean done() {
            return fed && !launcher.gateOpen.getAsBoolean();
        }

        @Override
        public void execute() {
            if (fed) return;
            if (conditions.ready() && launcher.flywheel.isReady()) {
                launcher.fire();
                fed = true;
            }
        }

        @Override
        public void end(EndCondition endCondition) {
            onEnd.accept(endCondition);
        }
    }

    /** Fire everything: feeds each launcher whenever it's ready, until held() goes false. */
    public static class FireAll implements Command {
        private final Launcher[] launchers;
        private final Conditions conditions;
        private final BooleanSupplier held;
        /** Each launcher's last feed (System.nanoTime()), or 0 if it hasn't fed in this hold. */
        private final long[] lastFeedNanos;
        private final Set<Object> gates = new HashSet<>();
        private int feeds;

        public FireAll(Conditions conditions, BooleanSupplier held, Launcher... launchers) {
            this.launchers = launchers;
            this.conditions = conditions;
            this.held = held;
            lastFeedNanos = new long[launchers.length];
            for (Launcher launcher : launchers) gates.add(launcher.gate);
        }

        public int feeds() {
            return feeds;
        }

        @Override
        public Set<Object> requirements() {
            return gates;
        }

        @Override
        public int priority() {
            return Requirements.ASSIST;
        }

        @Override
        public InterruptedBehavior interruptedBehavior() {
            return InterruptedBehavior.END;
        }

        @Override
        public ConflictBehavior conflictBehavior() {
            return ConflictBehavior.OVERRIDE;  // takes over from a queued single launch
        }

        @Override
        public BlockedBehavior blockedBehavior() {
            return BlockedBehavior.CANCEL;
        }

        @Override
        public void start() {
            feeds = 0;
            Arrays.fill(lastFeedNanos, 0L);
        }

        @Override
        public boolean done() {
            return !held.getAsBoolean();
        }

        @Override
        public void execute() {
            if (!conditions.ready()) return;
            long now = System.nanoTime();
            for (int i = 0; i < launchers.length; i++) {
                Launcher l = launchers[i];
                boolean spaced = lastFeedNanos[i] == 0 || now - lastFeedNanos[i] >= (long) (fireAllIntervalMs * 1e6);
                if (spaced && !l.gateOpen.getAsBoolean() && l.flywheel.isReady()) {
                    l.fire();
                    lastFeedNanos[i] = now;
                    feeds++;
                }
            }
        }

        @Override
        public void end(EndCondition endCondition) {
        }
    }
}
