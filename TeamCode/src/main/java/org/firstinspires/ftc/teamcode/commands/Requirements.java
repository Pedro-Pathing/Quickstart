package org.firstinspires.ftc.teamcode.commands;

/**
 * The parts of the robot's job that Ivy commands claim (see Ivy's requirements and priorities).
 * When a command needs one that another is using, their priorities decide which runs.
 *
 * Priorities: 0 manual driving, heading lock and manual intake; 1 auto-aim and launches;
 * 2 auto-collect and intake out; 3 the gunner's manual camera switch.
 *
 * Launches and auto-aim don't claim the camera, though both need it up. Requirements are
 * exclusive, so claiming it would make a launch end a running aim. Instead a launch waits by
 * itself while a collect runs, and aim queues behind a collect through the heading.
 */
public final class Requirements {
    public static final Object TRANSLATION = named("translation");
    public static final Object HEADING = named("heading");
    public static final Object CAMERA = named("camera");
    public static final Object INTAKE = named("intake");
    public static final Object POLLEN_GATE = named("pollen gate");
    public static final Object NECTAR_GATE = named("nectar gate");

    public static final int MANUAL = 0;
    public static final int ASSIST = 1;
    public static final int DRIVER_ASSIST = 2;
    public static final int OVERRIDE = 3;

    private Requirements() {
    }

    private static Object named(String name) {
        return new Object() {
            @Override
            public String toString() {
                return name;
            }
        };
    }
}
