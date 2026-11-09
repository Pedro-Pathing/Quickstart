package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.firstinspires.ftc.teamcode.drive.CollectAssist;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/** Auto-collect, the intake and the camera, as Ivy commands. */
public final class RobotCommands {
    private RobotCommands() {
    }

    /**
     * Auto-collect (CollectAssist), until it stops driving: picked up, reached, gave up, or cancelled.
     * The OpMode must still call collect.update() every loop. It suspends manual driving and the
     * manual intake, and ends a running auto-aim.
     */
    public static Command collect(CollectAssist collect, Consumer<EndCondition> onEnd) {
        return Command.build()
                .setStart(collect::start)
                .setDone(() -> !collect.isActive())
                .setEnd(end -> {
                    if (end == EndCondition.INTERRUPTED) collect.cancel();
                    onEnd.accept(end);
                })
                .requiring(Requirements.TRANSLATION, Requirements.HEADING, Requirements.CAMERA, Requirements.INTAKE)
                .setPriority(Requirements.DRIVER_ASSIST)
                .setInterruptedBehavior(InterruptedBehavior.END);
    }

    /** The intake running in until cancelled. Suspended by intake-out and auto-collect, then carries on. */
    public static Command intakeIn(Intake intake) {
        return Command.build()
                .setStart(intake::run)
                .setExecute(() -> {
                    if (!intake.isRunning()) intake.run();  // after a suspension
                    intake.update();
                })
                .setEnd(end -> {
                    if (end != EndCondition.SUSPENDED) intake.stop();
                })
                .requiring(Requirements.INTAKE)
                .setPriority(Requirements.MANUAL)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /** The intake running out while held() is true. */
    public static Command intakeOut(Intake intake, BooleanSupplier held) {
        return Command.build()
                .setStart(intake::reverse)
                .setDone(() -> !held.getAsBoolean())
                .setEnd(end -> intake.stop())
                .requiring(Requirements.INTAKE)
                .setPriority(Requirements.DRIVER_ASSIST)
                .setInterruptedBehavior(InterruptedBehavior.END)
                .setBlockedBehavior(BlockedBehavior.CANCEL);
    }

    /** Move the camera and wait for it to settle. Ends an auto-collect when raising it. */
    public static Command moveCamera(LimelightCamera camera, LimelightCamera.Mode mode) {
        return Command.build()
                .setStart(() -> camera.setMode(mode))
                .setDone(camera::isSettled)
                .requiring(Requirements.CAMERA)
                .setPriority(Requirements.OVERRIDE)
                .setInterruptedBehavior(InterruptedBehavior.END);
    }
}
