package Util;

public class Range {

    public static boolean isBetween(double value, double min, double max) {
        return value > min && value < max;
    }

    public static boolean isBetween(int value, int min, int max) {
        return value > min && value < max;
    }

    public static boolean isBetween(int value, double min, double max) {
        return value > min && value < max;
    }

    public static boolean isBetween(double value, int min, int max) {
        return value > min && value < max;
    }
}
