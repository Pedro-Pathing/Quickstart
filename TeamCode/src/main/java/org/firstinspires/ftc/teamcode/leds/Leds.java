package org.firstinspires.ftc.teamcode.leds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Devices;

/**
 * The REV Blinkin LED driver on the strip round the bottom of the robot (78 x 5 V addressable LEDs).
 *
 * The Blinkin is plugged into a servo port and configured as a "REV Blinkin LED Driver" called "blinkin".
 * The strip length isn't set from code: it's stored on the Blinkin itself (see the setup notes below).
 *
 * The LEDs show one RobotState at a time:
 *  - setState() sets the state the robot is in, e.g. GOOD_SHOT. It stays until it's changed.
 *  - flash() shows a state for a short time, then goes back to the set state, for one-off events later.
 * Call update() every loop. The Blinkin is only sent a new pattern when what's shown changes.
 * If the Blinkin isn't in the configuration, the LEDs are disabled and the calls do nothing.
 *
 * Blinkin setup, once, with the strip connected and running (REV's Getting Started guide):
 *  1. If the 5V light isn't on, hold Strip Select until it is.
 *  2. Hold Mode for about 6 s, until the status LED turns from blue to yellow.
 *  3. Turn the right-hand potentiometer to set the number of LEDs (78).
 *  4. Hold Mode and Strip Select together for about 6 s to save.
 */
public class Leds {
    private static final String DEVICE_NAME = "blinkin";

    private final RevBlinkinLedDriver blinkin;  // null when the LEDs are disabled
    private final ElapsedTime flashTimer = new ElapsedTime();

    private RobotState state = RobotState.READY;
    private RobotState flashState = null;
    private double flashSeconds;

    // What the Blinkin is showing now, so it's only written when this changes
    private RobotState shown = null;

    public Leds(HardwareMap hw) {
        blinkin = Devices.get(hw, RevBlinkinLedDriver.class, DEVICE_NAME);
        update();
    }

    /** The state the robot is in. Shown until changed (or while a flash is on top of it). */
    public void setState(RobotState newState) {
        state = newState;
    }

    /** Show a state for a while, then go back to the set state. A new flash replaces the old one. */
    public void flash(RobotState flashState, double seconds) {
        this.flashState = flashState;
        flashSeconds = seconds;
        flashTimer.reset();
    }

    /** Call every loop. */
    public void update() {
        if (flashState != null && flashTimer.seconds() > flashSeconds) {
            flashState = null;
        }
        RobotState wanted = flashState != null ? flashState : state;
        if (wanted != shown && blinkin != null) {
            blinkin.setPattern(wanted.pattern);
            shown = wanted;
        }
    }

    /** Whether the Blinkin was found. */
    public boolean isAvailable() {
        return blinkin != null;
    }

    /** The Blinkin's name in quotes if it's missing, otherwise "". */
    public String missingDevices() {
        return Devices.missing(blinkin, DEVICE_NAME);
    }

    /** The state set with setState(). */
    public RobotState getState() {
        return state;
    }

    /** What the LEDs are showing now (a flash, or the set state). */
    public RobotState getShown() {
        return shown;
    }
}
