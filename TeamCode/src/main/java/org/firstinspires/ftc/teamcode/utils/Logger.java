package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import dev.nextftc.ftc.ActiveOpMode;
// Make sure to import your PanelsTelemetry class here
// import ...PanelsTelemetry;

import java.util.LinkedHashMap;
import java.util.Map;

public final class Logger {

    public enum Level {
        INFO,
        DEBUG
    }

    private static final class LogEntry {
        final Level level;
        final String message;

        LogEntry(Level level, String message) {
            this.level = level;
            this.message = message;
        }
    }

    // 1. Integration of PanelsTelemetry
    private static final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private static final Map<String, StringBuilder> logs = new LinkedHashMap<>();
    private static final Map<String, Map<Integer, LogEntry>> structuredLogs = new LinkedHashMap<>();
    private static int logCounter = 0;

    private Logger() {
    }

    // --- Existing Add Methods ---

    public static void add(String subsystem, String message) {
        add(subsystem, Level.INFO, message);
    }

    public static void add(String subsystem, Level level, String message) {
        structuredLogs
                .computeIfAbsent(subsystem, k -> new LinkedHashMap<>())
                .put(logCounter++, new LogEntry(level, message));
    }

    // --- New Easy Logging Methods ---

    /**
     * Requirement 1: Easier logging with Key/Value pairs
     * Usage: Logger.log("Lift", "Current Amps", motor.getCurrent());
     */
    public static void log(String subsystem, String name, Object value) {
        add(subsystem, Level.INFO, name + ": " + value);
    }

    /**
     * Requirement 2: Dedicated Panels logging
     * Usage: Logger.panelsLog("Velo", currentVelocity);
     */
    public static void panelsLog(String name, Object value) {
        panelsTelemetry.getTelemetry().addData(name, value);
    }

    // --- Update Methods ---

    public static void update() {
        update(Level.INFO);
    }

    public static void update(Level verbosity) {
        // 1. Update Standard Driver Station Telemetry
        for (Map.Entry<String, Map<Integer, LogEntry>> entry : structuredLogs.entrySet()) {
            String subsystem = entry.getKey();
            Map<Integer, LogEntry> messages = entry.getValue();

            boolean printedHeader = false;

            for (LogEntry log : messages.values()) {
                // Only print if the log level matches or is more urgent than the requested verbosity
                // (Assuming you might want DEBUG to show INFO as well, but keeping your exact logic below)
                if (log.level == verbosity) {
                    if (!printedHeader) {
                        ActiveOpMode.telemetry().addLine(
                                subsystem.toUpperCase() + "---------------"
                        );
                        printedHeader = true;
                    }
                    ActiveOpMode.telemetry().addLine(log.message);
                }
            }
        }

        ActiveOpMode.telemetry().update();

        // Clear local logs for the next loop
        structuredLogs.clear();
        logCounter = 0;

        // 2. Update Panels/Dashboard Telemetry
        // This ensures you don't need to manually call update inside your subsystems
        panelsTelemetry.getTelemetry().update();
    }
}
