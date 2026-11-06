package org.firstinspires.ftc.teamcode.leds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

/**
 * What the LEDs show the drivers. Each state is one Blinkin pattern across the whole strip.
 *
 * To add a state, add a constant here with its pattern. The full list of patterns is
 * RevBlinkinLedDriver.BlinkinPattern, and REV's pattern table shows what each looks like.
 */
public enum RobotState {
    /** Initialised and ready to drive. */
    READY(BlinkinPattern.YELLOW),
    /** Good shooting position. */
    GOOD_SHOT(BlinkinPattern.GREEN),
    /** Marginal shooting position. Blinkin has no amber, and ORANGE stands out from YELLOW better than GOLD. */
    MARGINAL_SHOT(BlinkinPattern.ORANGE),
    /** No shot possible from here. */
    NO_SHOT(BlinkinPattern.RED),
    /** Flashed while a launcher has a fault (Flywheel.Health). Not white: all 78 LEDs white is near the 5 A limit. */
    LAUNCHER_FAULT(BlinkinPattern.VIOLET);

    public final BlinkinPattern pattern;

    RobotState(BlinkinPattern pattern) {
        this.pattern = pattern;
    }
}
