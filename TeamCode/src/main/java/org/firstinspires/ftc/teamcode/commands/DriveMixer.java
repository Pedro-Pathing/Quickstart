package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.drivetrain.DrivePowers;

import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;

/**
 * Combines the translation and the turn, which different commands supply (the driver's sticks,
 * the heading lock, auto-aim), into one drive call per loop. Call flush() after Scheduler.execute().
 * If no command supplied either this loop (an auto-collect is driving the follower itself), it
 * leaves the follower alone.
 */
public class DriveMixer {
    private final TeleOpDrive drive;
    private DrivePowers translation;
    private Double turn;

    public DriveMixer(TeleOpDrive drive) {
        this.drive = drive;
    }

    public void setTranslation(DrivePowers translation) {
        this.translation = translation;
    }

    public void setTurn(double turn) {
        this.turn = turn;
    }

    public void flush() {
        if (translation != null || turn != null) {
            drive.drive(translation != null ? translation : DrivePowers.zero(), turn != null ? turn : 0.0);
        }
        translation = null;
        turn = null;
    }
}
