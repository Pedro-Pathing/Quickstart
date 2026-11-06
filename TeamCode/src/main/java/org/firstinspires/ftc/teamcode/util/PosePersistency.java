package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Field;

public final class PosePersistency {
    private static Pose pose = null;
    private static Field.Alliance alliance = null;
    private static double turretZeroTicks = Double.NaN;

    public static void savePose(Pose pose) {
        PosePersistency.pose = pose;
    }

    public static void saveAlliance(Field.Alliance alliance) {
        PosePersistency.alliance = alliance;
    }

    public static void saveTurretZero(double ticks) {
        turretZeroTicks = ticks;
    }

    public static double getTurretZero(double fallback) {
        return Double.isNaN(turretZeroTicks) ? fallback : turretZeroTicks;
    }

    public static Pose getPose(Pose fallback) {
        return pose != null ? pose : fallback;
    }

    public static Field.Alliance getAlliance(Field.Alliance fallback) {
        return alliance != null ? alliance : fallback;
    }

    public static boolean hasPose() {
        return pose != null;
    }

    public static void clear() {
        pose = null;
        alliance = null;
        turretZeroTicks = Double.NaN;
    }

    private PosePersistency() {
    }
}
