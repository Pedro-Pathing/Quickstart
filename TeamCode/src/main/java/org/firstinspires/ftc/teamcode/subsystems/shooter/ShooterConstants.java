package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String MOTOR_NAME = "shooter";
    public static boolean MOTOR_REVERSED = false;
    public static double TICKS_PER_REV = 28;
    public static double MAX_RPM = 6000;

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(1, 0, 0, 0.00085);
    public static double SHOOTER_TOLERANCE_RPM = 40;
    public static double IDLE_POWER = 0.3;

    public static double[] SHOOTER_DIST_IN = {41.4, 50.8, 64.9, 70.4};
    public static double[] SHOOTER_RPM = {1675, 1755, 1835, 1885};
    public static double TUNING_RPM = 1950;
}
