package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.vision.HiveAim;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * The driving commands for TeleOp. Call drive.readDriver() each loop before Scheduler.execute().
 */
public final class DriveCommands {
    private DriveCommands() {
    }

    /** The driver's left stick, all match. Suspended (and then resumed) by an auto-collect. */
    public static Command manualTranslation(TeleOpDrive drive, DriveMixer mixer) {
        return Command.build()
                .setExecute(() -> mixer.setTranslation(drive.translation()))
                .requiring(Requirements.TRANSLATION)
                .setPriority(Requirements.MANUAL)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /**
     * The driver's right stick, then holding the heading the robot settles at, all match. Suspended
     * by auto-aim and auto-collect; when it resumes it brakes and holds wherever they left the robot.
     */
    public static Command headingLock(TeleOpDrive drive, DriveMixer mixer) {
        boolean[] resumed = {false};
        return Command.build()
                .setExecute(() -> {
                    if (resumed[0]) {
                        drive.holdCurrentHeading();
                        resumed[0] = false;
                    }
                    mixer.setTurn(drive.driverTurn());
                })
                .setEnd(end -> resumed[0] = end == EndCondition.SUSPENDED)
                .requiring(Requirements.HEADING)
                .setPriority(Requirements.MANUAL)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /** Auto-aim: holds the aim heading (from HiveAim, updated every loop by the OpMode) until cancelled. */
    public static class Aim implements Command {
        private final TeleOpDrive drive;
        private final DriveMixer mixer;
        private final Follower follower;
        private final HiveAim hiveAim;
        private final DoubleSupplier aimHeading;
        private final LimelightCamera camera;
        private final Consumer<EndCondition> onEnd;
        private boolean locked;

        public Aim(TeleOpDrive drive, DriveMixer mixer, Follower follower, HiveAim hiveAim, DoubleSupplier aimHeading,
                   LimelightCamera camera, Consumer<EndCondition> onEnd) {
            this.drive = drive;
            this.mixer = mixer;
            this.follower = follower;
            this.hiveAim = hiveAim;
            this.aimHeading = aimHeading;
            this.camera = camera;
            this.onEnd = onEnd;
        }

        /** Within HiveAim's lock tolerance, settled, with a heading from the camera. */
        public boolean isLocked() {
            return isScheduled() && locked;
        }

        @Override
        public Set<Object> requirements() {
            return Set.of(Requirements.HEADING);
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
            return BlockedBehavior.QUEUE;  // waits for an auto-collect to finish
        }

        @Override
        public void start() {
            hiveAim.reset();
            locked = false;
        }

        @Override
        public boolean done() {
            return false;
        }

        @Override
        public void execute() {
            camera.setMode(LimelightCamera.Mode.TAGS);
            double target = aimHeading.getAsDouble();
            mixer.setTurn(drive.aimTurn(target));
            double error = Angle.normalizeSigned(target - follower.pose().heading());
            locked = hiveAim.isFromVision() && HiveAim.isLocked(error, follower.twist().omega);
        }

        @Override
        public void end(EndCondition endCondition) {
            locked = false;
            onEnd.accept(endCondition);
        }
    }
}
