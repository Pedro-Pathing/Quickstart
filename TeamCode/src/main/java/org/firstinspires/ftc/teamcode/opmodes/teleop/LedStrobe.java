package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.Devices;

/**
 * Flashes the LED strip with one of the Blinkin's built-in strobe patterns, for measuring a camera's rolling
 * shutter readout time (calibrate_readout.py in the launcher simulation's tracking folder).
 *
 * The Control Hub can only change the Blinkin's pattern about every 20 ms, so the robot can't time flashes
 * itself. The Blinkin's strobe patterns flash at a steady rate from its own timer, which is what the
 * calibration needs; it doesn't need to know the rate.
 *
 * D-pad left/right  Change pattern (white strobe is the brightest)
 */
@TeleOp(name = "LED Strobe", group = "Tuning")
public class LedStrobe extends OpMode {
    private static final String DEVICE_NAME = "blinkin";
    private static final BlinkinPattern[] PATTERNS = {
            BlinkinPattern.STROBE_WHITE, BlinkinPattern.STROBE_GOLD, BlinkinPattern.STROBE_RED,
            BlinkinPattern.STROBE_BLUE, BlinkinPattern.WHITE, BlinkinPattern.BLACK
    };

    private RevBlinkinLedDriver blinkin;  // null if it isn't in the configuration
    private int index = 0;

    @Override
    public void init() {
        blinkin = Devices.get(hardwareMap, RevBlinkinLedDriver.class, DEVICE_NAME);
        telemetry.addLine(blinkin != null ? "Press START to flash the LEDs." : "No \"" + DEVICE_NAME + "\" in the configuration.");
        telemetry.update();
    }

    @Override
    public void start() {
        show();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) {
            index = (index + 1) % PATTERNS.length;
            show();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            index = (index + PATTERNS.length - 1) % PATTERNS.length;
            show();
        }
        telemetry.addData("Pattern", PATTERNS[index]);
        telemetry.addLine("D-pad left/right: change pattern");
        if (blinkin == null) telemetry.addLine("No \"" + DEVICE_NAME + "\" in the configuration.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (blinkin != null) blinkin.setPattern(BlinkinPattern.BLACK);
    }

    private void show() {
        if (blinkin != null) blinkin.setPattern(PATTERNS[index]);
    }
}
