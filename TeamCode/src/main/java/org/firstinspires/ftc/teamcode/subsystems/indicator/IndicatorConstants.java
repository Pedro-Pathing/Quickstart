package org.firstinspires.ftc.teamcode.subsystems.indicator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IndicatorConstants {
    public enum Color {
        OFF(0.000),
        RED(0.279),
        ORANGE(0.333),
        YELLOW(0.388),
        GREEN(0.500),
        BLUE(0.611),
        PURPLE(0.722),
        WHITE(1.000);

        public final double position;

        Color(double position) {
            this.position = position;
        }
    }

    public static String TURRET_LIGHT_NAME = "turretLight";
    public static String POLLEN_LIGHT_NAME = "pollenLight";

    public static Color TURRET_AIMED_COLOR = Color.GREEN;
    public static Color TURRET_NOT_AIMED_COLOR = Color.RED;

    public static Color POLLEN_IDLE_COLOR = Color.BLUE;
    public static Color POLLEN_BLINK_COLOR = Color.YELLOW;
    public static double BLINK_PERIOD_S = 0.1;
    public static double BLINK_TIME_S = 0.5;
}
