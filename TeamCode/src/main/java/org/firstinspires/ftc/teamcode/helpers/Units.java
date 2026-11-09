package org.firstinspires.ftc.teamcode.helpers;

/**
 * Metre and inch conversions, in one place. Pedro's field frame and poses are in inches; the
 * camera results (HiveTarget, FloorBall), the launch model and the robot's CAD measurements are in
 * metres. Converting through these, rather than writing 0.0254 or 39.37 inline, keeps every
 * conversion the same and makes it obvious which way each one goes.
 */
public final class Units {
    /** Exactly, by definition of the inch. */
    public static final double METRES_PER_INCH = 0.0254;
    public static final double INCHES_PER_METRE = 1.0 / METRES_PER_INCH;

    private Units() {
    }

    public static double metresToInches(double metres) {
        return metres * INCHES_PER_METRE;
    }

    public static double inchesToMetres(double inches) {
        return inches * METRES_PER_INCH;
    }
}
