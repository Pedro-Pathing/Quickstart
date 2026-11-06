package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Reads everything a hub can report (motor encoders and velocities, digital and analog inputs) in one
 * bulk read per hub per loop, instead of one slow read per call. Create it at INIT and call clear() at
 * the start of every loop, so each loop gets fresh readings.
 *
 * Not covered, so still one read per call: I2C devices (Pinpoint), battery voltage and motor current.
 */
public class BulkCache {
    private final List<LynxModule> hubs;

    public BulkCache(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /** Call at the start of every loop. The next read from each hub then fetches fresh data. */
    public void clear() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
