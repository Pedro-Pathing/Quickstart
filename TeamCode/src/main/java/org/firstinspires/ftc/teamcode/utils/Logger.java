package org.firstinspires.ftc.teamcode.utils;

import dev.nextftc.ftc.ActiveOpMode;

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

    private static final Map<String, StringBuilder> logs = new LinkedHashMap<>();
    private static final Map<String, Map<Integer, LogEntry>> structuredLogs = new LinkedHashMap<>();
    private static int logCounter = 0;

    private Logger() {
    }
    public static void add(String subsystem, String message) {
        add(subsystem, Level.INFO, message);
    }

    public static void add(String subsystem, Level level, String message) {
        structuredLogs
                .computeIfAbsent(subsystem, k -> new LinkedHashMap<>())
                .put(logCounter++, new LogEntry(level, message));
    }

    public static void update() {
        update(Level.INFO);
    }

    public static void update(Level verbosity) {
        for (Map.Entry<String, Map<Integer, LogEntry>> entry : structuredLogs.entrySet()) {
            String subsystem = entry.getKey();
            Map<Integer, LogEntry> messages = entry.getValue();

            boolean printedHeader = false;

            for (LogEntry log : messages.values()) {
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
        structuredLogs.clear();
        logCounter = 0;
    }
}
