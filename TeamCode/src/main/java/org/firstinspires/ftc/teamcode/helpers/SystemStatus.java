package org.firstinspires.ftc.teamcode.helpers;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Which parts of the robot are disabled because a device is missing, for showing on telemetry.
 * Check each part once at INIT, e.g. status.check("Vision", camera.missingDevices()).
 */
public class SystemStatus {
    private final List<String> messages = new ArrayList<>();

    /** Notes a part as disabled if any of its devices are missing ("" = none missing). */
    public void check(String system, String missingDevices) {
        if (!missingDevices.isEmpty()) {
            messages.add(system + " DISABLED: " + missingDevices + " not found");
        }
    }

    /** Notes a part as disabled for another reason. */
    public void disable(String system, String reason) {
        messages.add(system + " DISABLED: " + reason);
    }

    public boolean allAvailable() {
        return messages.isEmpty();
    }

    public void addTelemetry(TelemetryManager telemetryM) {
        for (String message : messages) telemetryM.addLine(message);
    }

    public void addTelemetry(Telemetry telemetry) {
        for (String message : messages) telemetry.addLine(message);
    }
}
