package org.firstinspires.ftc.teamcode.records;

/**
 * The CELL opening the shooter aims at, in the field frame.
 *
 * <p>{@code x}/{@code y} are the opening centre (the SDK AprilTag cluster origin) and {@code
 * heightInches} its height above the TILES. {@code facingRad} is the field heading the opening
 * faces outward: a ball can only enter from that side, so it is also the heading from the opening
 * toward where the robot must stand. {@code epoch} identifies the HIVE state this target was
 * computed for; it changes on every TIP, so a shot solved for one state can never fire into the
 * next.
 */
public record CellTarget(
    HiveCell cell, double x, double y, double heightInches, double facingRad, int epoch) {}
