package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.function.Consumer;

/**
 * Looks up devices without crashing the OpMode when one is missing, so the robot can be tested
 * with parts unplugged or not yet in the configuration. A missing device comes back as null, and
 * the class that uses it disables just that part of the robot (see SystemStatus).
 */
public final class Devices {
    private static final String TAG = "Devices";

    private Devices() {
    }

    /** The device, or null if there's no device of that type and name in the robot configuration. */
    public static <T> T get(HardwareMap hardwareMap, Class<T> type, String name) {
        return get(hardwareMap, type, name, null);
    }

    /**
     * The device, set up by setup, or null if it isn't in the robot configuration or setting it up
     * throws (e.g. it's configured but not responding). Either way the reason goes to the log.
     */
    public static <T> T get(HardwareMap hardwareMap, Class<T> type, String name, Consumer<T> setup) {
        try {
            T device = hardwareMap.get(type, name);
            if (setup != null) setup.accept(device);
            return device;
        } catch (RuntimeException e) {
            RobotLog.ww(TAG, "%s \"%s\" unavailable: %s", type.getSimpleName(), name, e.getMessage());
            return null;
        }
    }

    /** For missingDevices(): the name in quotes if the device is missing, otherwise "". */
    public static String missing(Object device, String name) {
        return device == null ? "\"" + name + "\"" : "";
    }

    /** Joins missing() results, e.g. "\"pollenGate\", \"nectarGate\"". */
    public static String join(String... missing) {
        StringBuilder out = new StringBuilder();
        for (String m : missing) {
            if (m.isEmpty()) continue;
            if (out.length() > 0) out.append(", ");
            out.append(m);
        }
        return out.toString();
    }
}
